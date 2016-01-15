# ros-kondo-b3m-driver-beta

近藤シリアルサーボのB3MシリーズをROS使って動かすやつ
B3M-SC-1170-Aを使って動作を確認

## 準備

```bash
sudo modprobe ftdi_sio
sudo su
echo "165C 0009" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
exit
```

## 使い方

```bash
roslaunch b3m_description b3m.launch
roslaunch b3m_servo b3m_servo.launch
```
