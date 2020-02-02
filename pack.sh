#!/bin/sh
# 可执行程序名
appname="point_cloud_player"
dst="./app"
rm -rf $dst.tar.gz $dst
mkdir $dst
# 利用 ldd 提取依赖库的具体路径
liblist=$(ldd $appname | awk '{ if (match($3,"/")){ printf("%s "), $3 } }')
# 目标文件夹的检测
if [ ! -d $dst ];then
		mkdir $dst
fi
# 拷贝库文件和可执行程序到目标文件夹
cp $liblist $dst
cp $appname $dst
tar -zcvf $dst.tar.gz $dst
rm -rf $dst
