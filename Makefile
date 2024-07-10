obj-m += leds-epcu2217.o

PWD := $(CURDIR)
KDIR:= /root/rsgw-image/rsgw-kernel/obj_i

all:
	make -C $(KDIR) M=$(PWD) modules

testled:
	modprobe ledtrig_timer
	modprobe ledtrig_heartbeat
	echo '-------------- insmod led' > /dev/kmsg
	insmod leds-epcu2217.ko debug=false
	echo '-------------- default' > /dev/kmsg
	echo -n "LED1 = "
	cat /sys/class/leds/epcu\:1/brightness
	echo -n "LED2 = "
	cat /sys/class/leds/epcu\:2/brightness
	echo -n "LED3 = "
	cat /sys/class/leds/epcu\:3/brightness
	echo '-------------- value' > /dev/kmsg
	sleep 1
	echo 1 > /sys/class/leds/epcu\:1/brightness
	sleep 1
	echo 1 > /sys/class/leds/epcu\:2/brightness
	sleep 1
	echo 1 > /sys/class/leds/epcu\:3/brightness
	sleep 1
	echo 0 > /sys/class/leds/epcu\:3/brightness
	sleep 1
	echo 0 > /sys/class/leds/epcu\:2/brightness
	sleep 1
	echo 0 > /sys/class/leds/epcu\:1/brightness
	sleep 1
	echo '-------------- timer/heartbeat' > /dev/kmsg
	echo "heartbeat" > /sys/class/leds/epcu\:1/trigger
	echo "timer" > /sys/class/leds/epcu\:2/trigger
	echo "100" > /sys/class/leds/epcu\:2/delay_on
	echo "100" > /sys/class/leds/epcu\:2/delay_off
	echo "timer" > /sys/class/leds/epcu\:3/trigger
	echo "1000" > /sys/class/leds/epcu\:3/delay_on
	echo "1000" > /sys/class/leds/epcu\:3/delay_off
	sleep 30
	echo '-------------- rmmod' > /dev/kmsg
	rmmod leds-epcu2217

clean:
	make -C $(KDIR)  M=$(PWD) clean
