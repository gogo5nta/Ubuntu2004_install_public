■インストール画面でウインドが欠けて設定できない場合
　参考：Ubuntu 20.04 LTS Desktopをインストールする
　　　https://redj.hatenablog.com/entry/2020/04/26/220635

・[ctrl] + [alt] + [t]で、ターミナルを起動
・以下を入力し、フォントサイズを変更
$ gsettings set org.gnome.desktop.interface text-scaling-factor 0.75

・他にも、Alt+F7でインストール画面のウィンドウを移動
　　↓
　Ubuntu18.04でも可能？

・他にも、Ubuntuをインストールする前に以下をインストールし、フォントサイズ変更
$ sudo apt install gnome-tweaks

