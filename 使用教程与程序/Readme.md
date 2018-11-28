# 半穿戴式手指动作探测器Prototype使用说明

> 2018年11月28日

## 程序

1. Arduino程序

见文件夹`WIFI6050ReaderWithMux`。

原型机中已经烧录，理论上无需再次烧录。

1. Processing程序

见文件夹`Serial6050ShowOne_Angle`。

## 测试

安装Processing3，打开文件如下图

![image-20181128143902878](https://ws3.sinaimg.cn/large/006tNbRwly1fxnrqhxcr1j30z00u0q75.jpg)

点击△运行程序，点击□中止程序（不会生成数据文件）。

程序运行如下：

![image-20181128144044207](https://ws3.sinaimg.cn/large/006tNbRwly1fxnrs58g81j30go0hw3z0.jpg)

首先将鼠标移到此界面上面，点击一下界面使得键盘对程序起效。

然后通过键盘控制。控制方法如下：

```
 按S键Start
 按Q键Quit（这样才会存储数据。<u>如果点击□结束则不存储数据</u>）
 按+键下一张（即是看下一个关节传感器的状态）
 按-键上一张
 按>键开始轮播
 按<键停止轮播
```

界面中的`*`指示当前的传感器编号。

## Debug

### 1. 软件

注意！需要更改端口号。下方terminal中显示多个端口号。其排序从0开始。将设备端口号序号填入`Serial6050ShowOne_Angle.pde`的下方所示代码中。

```java
  println(Serial.list());
  String portName = Serial.list()[2];
  port = new Serial(this, portName, 115200);
```

在我的设备中，第三个是，所以我填2。

![](https://ws2.sinaimg.cn/large/006tNbRwly1fxns8oepekj30z00u00vh.jpg)

### 2. 硬件

由于原型机线材原因，会导致运行不稳定。

出错时现象如下：

1. 小飞机乱舞，方向不稳定。
2. 小飞机只能在一个平面上运动，另一个方向无法运动。
3. 上方数字变红。这样debug难度降低。当看到变红的则按+–号跳转到该传感器，然后看一下是否真的出错了。

解决方法如下：

1. 将装置断电，然后重启。
2. 如果没有解决，则是有线断了，找到断了的线，重新焊接。
3. 如果焊接无用，则是传感器坏了，更换传感器。{注意！更换的感器需校准后使用。}

