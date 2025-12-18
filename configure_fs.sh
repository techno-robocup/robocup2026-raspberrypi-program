PATH_TO_ROOTFS="/media/rotarymars/rootfs"
USERNAME="newuser"
PASSWORD="yourpassword"
ENCRYPTED=$(openssl passwd -6 "$PASSWORD")
echo "$USERNAME:x:1001:1001:New User,,,:/home/$USERNAME:/bin/bash" >>${PATH_TO_ROOTFS}/etc/passwd
echo "$USERNAME:$ENCRYPTED:19000:0:99999:7:::" >>${PATH_TO_ROOTFS}/etc/shadow
echo "$USERNAME:x:1001:" >>${PATH_TO_ROOTFS}/etc/group
sed -i "s/sudo:x:27:.*/&,$USERNAME/" ${PATH_TO_ROOTFS}/etc/group # Add to sudo group
mkdir -p ${PATH_TO_ROOTFS}/home/$USERNAME
cp -r ${PATH_TO_ROOTFS}/etc/skel/. ${PATH_TO_ROOTFS}/home/$USERNAME/
chown -R 1001:1001 ${PATH_TO_ROOTFS}/home/$USERNAME
chmod 700 ${PATH_TO_ROOTFS}/home/$USERNAME
