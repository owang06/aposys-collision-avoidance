https://manual.zlg.cn/web/#/55/2282 ����ͼ�ĵ������ĵ���������Ľ���һ�£�

USBCAN-II�°���������libusbʵ�֣���ȷ�����л�������libusb-1.0�Ŀ⡣
�����ubuntu�����������߰�װ���������£�
# apt-get install libusb-1.0-0

��libusbcan.so����/libĿ¼��
# sudo cp libusbcan.so /lib

����C�������̣�
# sudo ./test

����python���̣�
# sudo python3 usbcan.py



�豸���Գ������

1���鿴ϵͳ�Ƿ�����ö�ٵ�usb�豸����ӡ���ǵ�VID/PID��USBCANΪ0471:1200����
	# lsusb

2���鿴ϵͳ������USB�豸�ڵ㼰�����Ȩ�ޣ�
	# ls /dev/bus/usb/ -lR

3���޸�usb�豸�ķ���Ȩ��ʹ��ͨ�û����Բ���������xxx��Ӧlsusb�����Ϣ�е�bus��ţ�yyy��Ӧdevice��ţ�
	# chmod 666 /dev/bus/usb/xxx/yyy

4�����Ҫ���ø�����ͨ�û�����USBCAN�豸��Ȩ�ޣ���Ҫ�޸�udev���ã������ļ���/etc/udev/rules.d/50-usbcan.rules���������£�
	SUBSYSTEMS=="usb", ATTRS{idVendor}=="0471", ATTRS{idProduct}=="1200", GROUP="users", MODE="0666"

	���¼���udev��������豸����Ӧ����Ȩ�ޣ�
	# udevadm control --reload
