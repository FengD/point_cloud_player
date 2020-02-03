#!/bin/sh
# params
appname="point_cloud_player"
dst="./app"
plugins="plugins"
readme="readme.md"
rm -rf $dst.tar.gz $dst
mkdir $dst
# use ldd extract libs
liblist=$(ldd $appname | awk '{ if (match($3,"/")){ printf("%s "), $3 } }')

if [ ! -d $dst ];then
		mkdir $dst
fi
# cp files
cp $liblist $dst
cp $appname $dst
cp $readme $dst
cp -r $plugins $dst
# pack
tar -zcvf $dst.tar.gz $dst
#remove
rm -rf $dst
