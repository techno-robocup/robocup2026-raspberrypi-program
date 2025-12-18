#!/bin/bash
set -e # Exit on error

# Configuration variables
PATH_TO_ROOTFS="${PATH_TO_ROOTFS:-/media/rotarymars/rootfs}"
PATH_TO_BOOTFS="${PATH_TO_BOOTFS:-/media/rotarymars/bootfs}"
USERNAME="${USERNAME:-robo}"
PASSWORD="${PASSWORD:-techno}"
WIFI_SSID="${WIFI_SSID:-rotarymars}"
WIFI_PASSWORD="${WIFI_PASSWORD:-rotarymars}"
WIFI_COUNTRY="${WIFI_COUNTRY:-JP}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper functions
print_success() {
  echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
  echo -e "${RED}[✗]${NC} $1"
}

print_info() {
  echo -e "${YELLOW}[i]${NC} $1"
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  print_error "This script must be run as root (use sudo)"
  exit 1
fi

# Check if paths exist
if [ ! -d "$PATH_TO_ROOTFS" ]; then
  print_error "Root filesystem not found at $PATH_TO_ROOTFS"
  exit 1
fi

if [ ! -d "$PATH_TO_BOOTFS" ]; then
  print_error "Boot filesystem not found at $PATH_TO_BOOTFS"
  exit 1
fi

print_info "Starting Raspberry Pi configuration..."
echo "Root FS: $PATH_TO_ROOTFS"
echo "Boot FS: $PATH_TO_BOOTFS"
echo "Username: $USERNAME"
echo ""

# ========================================
# 1. Configure User Account
# ========================================
print_info "Configuring user account: $USERNAME"

# Encrypt password
ENCRYPTED=$(openssl passwd -6 "$PASSWORD")

# Check if user already exists in passwd
if grep -q "^$USERNAME:" "${PATH_TO_ROOTFS}/etc/passwd" 2>/dev/null; then
  print_info "User $USERNAME already exists in passwd, skipping..."
else
  echo "$USERNAME:x:1001:1001:New User,,,:/home/$USERNAME:/bin/bash" >>"${PATH_TO_ROOTFS}/etc/passwd"
  print_success "Added user to /etc/passwd"
fi

# Check if user already exists in shadow
if grep -q "^$USERNAME:" "${PATH_TO_ROOTFS}/etc/shadow" 2>/dev/null; then
  print_info "User $USERNAME already exists in shadow, skipping..."
else
  echo "$USERNAME:$ENCRYPTED:19000:0:99999:7:::" >>"${PATH_TO_ROOTFS}/etc/shadow"
  print_success "Added user to /etc/shadow"
fi

# Check if group already exists
if grep -q "^$USERNAME:" "${PATH_TO_ROOTFS}/etc/group" 2>/dev/null; then
  print_info "Group $USERNAME already exists, skipping..."
else
  echo "$USERNAME:x:1001:" >>"${PATH_TO_ROOTFS}/etc/group"
  print_success "Added group to /etc/group"
fi

# Add to sudo group if not already there
if grep "^sudo:.*$USERNAME" "${PATH_TO_ROOTFS}/etc/group" >/dev/null 2>&1; then
  print_info "User already in sudo group, skipping..."
else
  sed -i "s/^\(sudo:x:27:.*\)/\1,$USERNAME/" "${PATH_TO_ROOTFS}/etc/group"
  print_success "Added user to sudo group"
fi

# Create home directory
if [ -d "${PATH_TO_ROOTFS}/home/$USERNAME" ]; then
  print_info "Home directory already exists, skipping..."
else
  mkdir -p "${PATH_TO_ROOTFS}/home/$USERNAME"
  cp -r "${PATH_TO_ROOTFS}/etc/skel/." "${PATH_TO_ROOTFS}/home/$USERNAME/" 2>/dev/null || true
  chown -R 1001:1001 "${PATH_TO_ROOTFS}/home/$USERNAME"
  chmod 700 "${PATH_TO_ROOTFS}/home/$USERNAME"
  print_success "Created home directory with proper permissions"
fi

# ========================================
# 2. Configure WiFi
# ========================================
print_info "Configuring WiFi network: $WIFI_SSID"

# Method 1: wpa_supplicant.conf in boot (for older Raspberry Pi OS)
WPA_CONF="${PATH_TO_BOOTFS}/wpa_supplicant.conf"

# Create wpa_supplicant.conf (overwrite if exists)
cat >"$WPA_CONF" <<WPAEOF
country=$WIFI_COUNTRY
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

WPAEOF

# Generate and append the network configuration
wpa_passphrase "$WIFI_SSID" "$WIFI_PASSWORD" >> "$WPA_CONF"
chmod 600 "$WPA_CONF"
print_success "Created wpa_supplicant.conf in boot partition"

# Method 2: Configure in rootfs for newer systems
if [ -d "${PATH_TO_ROOTFS}/etc/wpa_supplicant" ]; then
    WPA_CONF_ROOTFS="${PATH_TO_ROOTFS}/etc/wpa_supplicant/wpa_supplicant.conf"
    cat >"$WPA_CONF_ROOTFS" <<WPAEOF
country=$WIFI_COUNTRY
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

WPAEOF
    wpa_passphrase "$WIFI_SSID" "$WIFI_PASSWORD" >> "$WPA_CONF_ROOTFS"
    chmod 600 "$WPA_CONF_ROOTFS"
    print_success "Created wpa_supplicant.conf in rootfs"
fi

# Method 3: NetworkManager configuration (for Raspberry Pi OS Bookworm and newer)
NM_DIR="${PATH_TO_ROOTFS}/etc/NetworkManager/system-connections"
if [ -d "${PATH_TO_ROOTFS}/etc/NetworkManager" ]; then
    mkdir -p "$NM_DIR"

    # Generate PSK hash for NetworkManager
    PSK=$(wpa_passphrase "$WIFI_SSID" "$WIFI_PASSWORD" | grep '^\spsk=' | cut -d= -f2)

    # Generate UUID (use uuidgen if available, otherwise create one)
    if command -v uuidgen &> /dev/null; then
        UUID=$(uuidgen)
    else
        UUID=$(cat /proc/sys/kernel/random/uuid 2>/dev/null || echo "$(date +%s)-$(shuf -i 1000-9999 -n 1)")
    fi

    cat >"${NM_DIR}/${WIFI_SSID}.nmconnection" <<NMEOF
[connection]
id=$WIFI_SSID
uuid=$UUID
type=wifi
autoconnect=true
autoconnect-priority=999

[wifi]
mode=infrastructure
ssid=$WIFI_SSID

[wifi-security]
key-mgmt=wpa-psk
psk=$PSK

[ipv4]
method=auto

[ipv6]
method=auto
addr-gen-mode=stable-privacy
NMEOF

    chmod 600 "${NM_DIR}/${WIFI_SSID}.nmconnection"
    print_success "Created NetworkManager connection"
fi

# Method 4: dhcpcd configuration (for additional compatibility)
if [ -f "${PATH_TO_ROOTFS}/etc/dhcpcd.conf" ]; then
    if ! grep -q "interface wlan0" "${PATH_TO_ROOTFS}/etc/dhcpcd.conf"; then
        cat >>"${PATH_TO_ROOTFS}/etc/dhcpcd.conf" <<'DHCPEOF'

# WiFi configuration
interface wlan0
env ifwireless=1
env wpa_supplicant_driver=nl80211,wext
DHCPEOF
        print_success "Updated dhcpcd.conf for WiFi"
    fi
fi

print_success "WiFi configuration complete (multiple methods configured)"

# Method 5: Ensure WiFi is not blocked by rfkill
# Create a systemd service to unblock WiFi on boot
RFKILL_SERVICE="${PATH_TO_ROOTFS}/etc/systemd/system/unblock-wifi.service"
if [ -d "${PATH_TO_ROOTFS}/etc/systemd/system" ]; then
    cat >"$RFKILL_SERVICE" <<'RFKILLEOF'
[Unit]
Description=Unblock WiFi
Before=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/rfkill unblock wifi
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
RFKILLEOF

    # Enable the service
    ln -sf "../unblock-wifi.service" "${PATH_TO_ROOTFS}/etc/systemd/system/multi-user.target.wants/unblock-wifi.service" 2>/dev/null || true
    print_success "Created rfkill unblock service"
fi

# ========================================
# 3. Enable SSH
# ========================================
print_info "Enabling SSH..."

# Create ssh file in boot partition to enable SSH
touch "${PATH_TO_BOOTFS}/ssh"
print_success "SSH enabled (created /boot/ssh)"

# ========================================
# 4. Enable UART
# ========================================
print_info "Enabling UART..."

CONFIG_FILE="${PATH_TO_BOOTFS}/config.txt"

if [ -f "$CONFIG_FILE" ]; then
  # Check if enable_uart is already set
  if grep -q "^enable_uart=" "$CONFIG_FILE"; then
    sed -i 's/^enable_uart=.*/enable_uart=1/' "$CONFIG_FILE"
    print_success "Updated enable_uart in config.txt"
  else
    echo "" >>"$CONFIG_FILE"
    echo "# Enable UART" >>"$CONFIG_FILE"
    echo "enable_uart=1" >>"$CONFIG_FILE"
    print_success "Added enable_uart to config.txt"
  fi

  # Also disable Bluetooth to free up the hardware UART (optional but recommended)
  if ! grep -q "^dtoverlay=disable-bt" "$CONFIG_FILE"; then
    echo "dtoverlay=disable-bt" >>"$CONFIG_FILE"
    print_success "Disabled Bluetooth to free hardware UART"
  fi
else
  print_error "config.txt not found at $CONFIG_FILE"
fi

# ========================================
# 5. Additional Serial Console Configuration
# ========================================
print_info "Configuring serial console..."

CMDLINE_FILE="${PATH_TO_BOOTFS}/cmdline.txt"

if [ -f "$CMDLINE_FILE" ]; then
  # Backup original cmdline.txt
  cp "$CMDLINE_FILE" "${CMDLINE_FILE}.backup" 2>/dev/null || true

  # Check if console=serial0 is already present
  if ! grep -q "console=serial0" "$CMDLINE_FILE"; then
    sed -i '1s/^/console=serial0,115200 /' "$CMDLINE_FILE"
    print_success "Added serial console to cmdline.txt"
  else
    print_info "Serial console already configured in cmdline.txt"
  fi
else
  print_info "cmdline.txt not found (may not be needed for this OS version)"
fi

# ========================================
# Summary
# ========================================
echo ""
print_success "Raspberry Pi configuration complete!"
echo ""
echo "Configuration summary:"
echo "  • User '$USERNAME' created with sudo access"
echo "  • WiFi configured for network '$WIFI_SSID' (country: $WIFI_COUNTRY)"
echo "  • SSH enabled"
echo "  • UART enabled (hardware serial)"
echo "  • Serial console configured"
echo "  • WiFi unblock service enabled"
echo ""
echo "You can now unmount the SD card and boot your Raspberry Pi."
echo "Default login: $USERNAME / $PASSWORD"
echo ""
print_info "WiFi Troubleshooting:"
echo "If WiFi doesn't connect, try these steps on the Pi:"
echo "  1. Check WiFi status: iwconfig wlan0"
echo "  2. Check if interface is up: ip link show wlan0"
echo "  3. Scan for networks: sudo iwlist wlan0 scan | grep ESSID"
echo "  4. Check wpa_supplicant: sudo systemctl status wpa_supplicant"
echo "  5. Check NetworkManager: sudo systemctl status NetworkManager"
echo "  6. View logs: sudo journalctl -u wpa_supplicant -u NetworkManager"
echo "  7. Manual connect: sudo wpa_cli -i wlan0 reconfigure"
echo "  8. Check country code: sudo iw reg get"
echo ""
echo "To manually test WiFi configuration:"
echo "  sudo rfkill unblock wifi"
echo "  sudo ip link set wlan0 up"
echo "  sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf"
echo "  sudo dhcpcd wlan0"
