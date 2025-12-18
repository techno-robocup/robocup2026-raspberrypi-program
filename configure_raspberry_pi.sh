#!/bin/bash
set -e  # Exit on error

# Configuration variables
PATH_TO_ROOTFS="${PATH_TO_ROOTFS:-/media/rotarymars/rootfs}"
PATH_TO_BOOTFS="${PATH_TO_BOOTFS:-/media/rotarymars/bootfs}"
USERNAME="${USERNAME:-newuser}"
PASSWORD="${PASSWORD:-yourpassword}"
WIFI_SSID="${WIFI_SSID:-rotarymars}"
WIFI_PASSWORD="${WIFI_PASSWORD:-rotarymars}"
WIFI_COUNTRY="${WIFI_COUNTRY:-US}"

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
    echo "$USERNAME:x:1001:1001:New User,,,:/home/$USERNAME:/bin/bash" >> "${PATH_TO_ROOTFS}/etc/passwd"
    print_success "Added user to /etc/passwd"
fi

# Check if user already exists in shadow
if grep -q "^$USERNAME:" "${PATH_TO_ROOTFS}/etc/shadow" 2>/dev/null; then
    print_info "User $USERNAME already exists in shadow, skipping..."
else
    echo "$USERNAME:$ENCRYPTED:19000:0:99999:7:::" >> "${PATH_TO_ROOTFS}/etc/shadow"
    print_success "Added user to /etc/shadow"
fi

# Check if group already exists
if grep -q "^$USERNAME:" "${PATH_TO_ROOTFS}/etc/group" 2>/dev/null; then
    print_info "Group $USERNAME already exists, skipping..."
else
    echo "$USERNAME:x:1001:" >> "${PATH_TO_ROOTFS}/etc/group"
    print_success "Added group to /etc/group"
fi

# Add to sudo group if not already there
if grep "^sudo:.*$USERNAME" "${PATH_TO_ROOTFS}/etc/group" > /dev/null 2>&1; then
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

WPA_CONF="${PATH_TO_BOOTFS}/wpa_supplicant.conf"

# Create wpa_supplicant.conf (overwrite if exists)
cat > "$WPA_CONF" << EOF
country=$WIFI_COUNTRY
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

EOF

# Generate and append the network configuration
wpa_passphrase "$WIFI_SSID" "$WIFI_PASSWORD" >> "$WPA_CONF"
print_success "Created WiFi configuration"

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
        echo "" >> "$CONFIG_FILE"
        echo "# Enable UART" >> "$CONFIG_FILE"
        echo "enable_uart=1" >> "$CONFIG_FILE"
        print_success "Added enable_uart to config.txt"
    fi

    # Also disable Bluetooth to free up the hardware UART (optional but recommended)
    if ! grep -q "^dtoverlay=disable-bt" "$CONFIG_FILE"; then
        echo "dtoverlay=disable-bt" >> "$CONFIG_FILE"
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
echo "  • WiFi configured for network '$WIFI_SSID'"
echo "  • SSH enabled"
echo "  • UART enabled (hardware serial)"
echo "  • Serial console configured"
echo ""
echo "You can now unmount the SD card and boot your Raspberry Pi."
echo "Default login: $USERNAME / $PASSWORD"
