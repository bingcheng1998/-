# finger movement measurement (Prototype)

## Introduction

<img src="https://ws1.sinaimg.cn/large/006tNc79ly1fvnwve8je1j30tp0dd7i0.jpg" width="495">



## Test of prototype

![GIF 1](https://ws2.sinaimg.cn/large/006tNc79ly1fvnwpu32y4g307t0dw1kz.gif)

## Structure

<img src="https://ws3.sinaimg.cn/large/006tNc79ly1fvnx11tygsj31kw2424qs.jpg"  height="530">

## Usage

<img src="https://ws4.sinaimg.cn/large/006tNc79ly1fvnx0oydu7j31kw16ke85.jpg" width="495">

<img src="https://ws1.sinaimg.cn/large/006tNc79ly1fvnx17aqtlj31kw242npg.jpg"  height="530">

## Detailed construction

### MPU6050 (GY521)

We can get the [quaternion](https://en.wikipedia.org/wiki/Quaternion) from [MPU6050](http://playground.arduino.cc/Main/MPU-6050). The InvenSense MPU-6050 sensor contains a MEMS accelerometer and a MEMS gyro in a single chip. It is very accurate, as it contains 16-bits analog to digital conversion hardware for each channel. Therefor it captures the x, y, and z channel at the same time. The sensor uses the I2C-bus to interface with the Arduino.

The MPU-6050 is not expensive, especially given the fact that it combines both an accelerometer and a gyro.

<img src="http://fritzing.org/media/fritzing-repo/projects/m/mpu-6050-board-gy-521-acelerometro-y-giroscopio/images/GY-521.png" alt="MPU6050(GY521 chip)" height="200">

### Quaternion

In [mathematics](https://en.wikipedia.org/wiki/Mathematics), the **quaternions** are a [number system](https://en.wikipedia.org/wiki/Number_system) that extends the [complex numbers](https://en.wikipedia.org/wiki/Complex_number). They were first described by Irish mathematician [William Rowan Hamilton](https://en.wikipedia.org/wiki/William_Rowan_Hamilton) in 1843[[1\]](https://en.wikipedia.org/wiki/Quaternion#cite_note-1)[[2\]](https://en.wikipedia.org/wiki/Quaternion#cite_note-2) and applied to [mechanics](https://en.wikipedia.org/wiki/Mechanics) in [three-dimensional space](https://en.wikipedia.org/wiki/Three-dimensional_space). A feature of quaternions is that multiplication of two quaternions is [noncommutative](https://en.wikipedia.org/wiki/Noncommutative). Hamilton defined a quaternion as the [quotient](https://en.wikipedia.org/wiki/Quotient) of two directed lines in a three-dimensional space or equivalently as the quotient of two [vectors](https://en.wikipedia.org/wiki/Vector_(geometry)).

Quaternions are generally represented in the form:

<a href="https://www.codecogs.com/eqnedit.php?latex=a&space;&plus;&space;bi&space;&plus;&space;cj&space;&plus;&space;dk" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a&space;&plus;&space;bi&space;&plus;&space;cj&space;&plus;&space;dk" title="a + bi + cj + dk" /></a>





<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/b3/Quaternion_2.svg/600px-Quaternion_2.svg.png" alt="quaternion" height="400">

a, b, c, and d are real numbers, and ***i***, ***j***, and ***k*** are the fundamental quaternion units, where a, b, c and d can be get from MPU6050 chip.

