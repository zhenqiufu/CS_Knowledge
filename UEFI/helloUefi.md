#### 创建虚拟镜像

###### 新建HDD_BOOT.img的空文件
qemu-img create -f raw HDD_BOOT.img 64M 
###### 格式化img文件
mkfs.vfat HDD_BOOT.img
###### 将文件加载到设备文件
losetup  /dev/loop8 HDD_BOOT.img
#新建挂载目录
mkdir -p /mnt/hello
#将设备挂载到目录
mount /dev/loop8 /mnt/hello


#### 把efi放进镜像

cp ./Build/OvmfX64/DEBUG_GCC48/X64/OvmfPkg/HelloWorld/HelloWorld/DEBUG/HelloWorld.efi /mnt/hello
###### 将efi文件回写到 HDD_BOOT.img
umount /dev/loop8
losetup -d /dev/loop8
————————————————
版权声明：本文为CSDN博主「xiaopangzi313」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/xiaopangzi313/article/details/89596652

#### 加载

qemu-system-x86_64 -bios ./Build/OvmfX64/DEBUG_GCC48/FV/OVMF.fd  \
-serial stdio -usb -drive if=none,format=raw,id=disk1,file=HDD_BOOT.img \
-device usb-storage,drive=disk1
————————————————
qemu-system-x86_64 -bios /root/edk2/Build/OvmfX64/DEBUG_GCC5/FV/OVMF.fd -hda hda.img  -serial stdio



-serial stdio 串口功能，将虚拟机内容显示在teminal中。



https://blog.csdn.net/xiaopangzi313/article/details/89596652