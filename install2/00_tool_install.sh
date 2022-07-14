#!/bin/bash
# update 2022.02.07

# Check OS ver and Kernel ver
echo "# Check OS ver and Kernel ver"
cat /etc/os-release
uname -r

# rename English holder
echo "# rename English holder"
LANG=C xdg-user-dirs-gtk-update

# install chromium-browser
echo "# install chromium-browser"
sudo apt -y install chromium-browser

# install terminator
echo "# install terminator"
sudo apt -y install terminator

# install ssh
echo "# install ssh"
sudo apt -y install openssh-server

# install good tool gnome-tweaks
echo "# install good tool gnome-tweaks"
sudo apt -y install gnome-tweaks

# install VSCode
#https://qiita.com/yoshiyasu1111/items/e21a77ed68b52cb5f7c8
echo "# install VSCode"
sudo apt -y install curl
curl -L https://go.microsoft.com/fwlink/?LinkID=760868 -o vscode.deb
sudo apt -y install ./vscode.deb

# install imagemagic
sudo apt -y install imagemagick
