SSID="rotarymars"
PSWD="rotarymars"
PATH_TO_BOOTFS="/media/rotarymars/bootfs"

echo "country=US" >>${PATH_TO_BOOTFS}/wpa_supplicant.conf
echo "ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev" >>${PATH_TO_BOOTFS}/wpa_supplicant.conf
echo "update_config=1" >>${PATH_TO_BOOTFS}/wpa_supplicant.conf
wpa_passphrase ${SSID} ${PSWD} >>${PATH_TO_BOOTFS}/wpa_supplicant.conf
