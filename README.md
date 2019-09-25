桌面级6轴机械臂安装使用说明书
=====

## 介绍

*   本说明书是针对桌面级6轴机械臂的使用说明。
*   在使用之前请仔细阅读本说明书内容。

<br>所有关节采用QDD Lite系列执行器搭建。其臂长活动范围最大430mm，末端负载500g。因QDD Lite系列执行器采用了复合材料，大大降低了高端机器人本机研发成本，主要应用于教育领域，学校、实验室、研究所、竞赛等。
                                                                          
<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0.png" style="width:720px">                                                                          

## 工程参数图
<br>[单位：毫米]

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0.md三视图.png" style="width:720px">


### 3D模型
[模型文件]( ../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0.step.zip )


## 基本参数

<table style="width:500px"><thead><tr><th colspan="2" style="background: PaleTurquoise; color: black;">桌面级6轴Lite版机械臂（6*NE30）参数</th></tr></thead><tbody></tr><tr><td>末端负载</td><td>500g</td></tr>
<tr><td>自重</td><td>3.7kg（带底座）；2.5kg（不带底座）</td></tr><tr><td>自由度</td><td>6</td></tr><tr><td>工作半径</td><td>421mm</td></tr><tr><td>关节范围</td><td>+/-170°</td></tr><tr><td>工具最大速度</td><td>2m/s</td></tr><tr><td>重复定位精度</td><td>+/-0.1mm</td></tr><tr><td>供电电压</td><td>42v</td></tr><tr><td>功耗</td><td>普通功耗约120w</td></tr><tr><td>结构件材料</td><td>铝合金/碳纤维管</td></tr><tr><td>工作环境温度</td><td>10-50°</td></tr><tr><td>工作环境湿度</td><td>5%~95%</td></tr><tr><td>防护等级</td><td>IP54</td></tr><tr><td>通信端口</td><td>CAN/以太网</td></tr><tr><td>示教器</td><td>电脑或者移动终端</td></tr></td></tbody></table>


## 产品实拍效果图

<img src="../img/Robot_DOF6/13.jpg" style="width:600px">

## 硬件需求与连接

**硬件需求**

<img src="../img/12.png" style="width:600px">

从前到后、从左到右依次为：六轴机械臂一台、插好终端电阻和回馈制动电容的ECB+HUB、急停开关+电源、电脑。


**连接ECB**

**连接电源**

*   连接电源与`ECB+HUB`

<img src="../img/cdy.jpg" style="width:600px">
<img src="../img/cwdy.jpg" style="width:600px">

**连接执行器及其配件**

*   连接`执行器综合线缆`

<img src="../img/cx.jpg" style="width:600px">
<img src="../img/cwx.jpg" style="width:600px">

**连接机械臂**

*   用执行器连接线连接`HUB`与执行器

<img src="../img/5.png" style="width:600px">

**连接电脑**

*   用网线连接`ECB`与电脑

<img src="../img/7.png" style="width:600px">

**连接后整体视图**

<img src="../img/12.png" style="width:600px">


**开启电源**

*   开启电源. 执行器的供电电压范围为直流24V-45V.

<img src="../img/poweron.png" style="width:600px">

*   上电以后，执行器LED状态灯会变成黄色闪烁，启动执行器后，LED会变成绿色闪烁，这时就可以与执行器进行通信了。如果执行器内部出现错误，LED灯会变为红色闪烁，请检查执行器错误代码。


## 软件安装与使用


**IAS软件的使用**

* `IAS`(INNFOS Actuator Studio)的为配置机械臂的上位机软件 , 请访问[INNFOS Actuator Studio(IAS)说明](#!pages/INNFOS_Actuator_Studio_IAS_instruction.md).

**运动功能使用**

* 示教-再现功能
  

### 下载安装

运行环境：linux-x86-64

访问该链接[download link](https://github.com/innfos/robot_controller-6-NE30-.git)下载机械臂软件或者直接执行以下命令
```sh
$ git clone https://github.com/innfos/robot_controller-6-NE30-.git
```
访问该链接[download link](https://github.com/innfos/ActuatorController_SDK.git)下载SDK相关文件或者直接执行以下命令
```sh
$ git clone https://github.com/innfos/ActuatorController_SDK.git
```

Note: 此两文件夹需放在同一目录下

### 运行模式
机械臂提供以下运行 mode0,mode1,mode2,mode3,mode4

进入到主目录
```sh
$ cd robot_controller-6-NE30-/
```
配置环境变量
```sh
$ . envirment
```
<br>可执行文件为robotserver
<br>根据运行模式在终端输入： 
```sh
$ ./robotserver xx
```
其中“xx”为mode0,mode1,mode2,mode3,mode4
<br>其中file文件夹用于存储路径文件

* mode0
此模式的作用是关掉机械臂使能状态
<br>执行命令
```sh
$ ./robotserver mode0
```

Note: 机械臂断电之前，需执行此命令，然后才能关掉电源


* mode1
此模式的作用是连续记录轨迹
<br>执行命令 
```sh
$ ./robotserver mode1
```


此时终端会显示：

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_10.png" style="width:600px">

<br>输入“start”开始记录。
<br>CTRL + C可停止记录，此时轨迹文件存储在file/trajectory.txt里。


* mode2
再现“mode1”生成的轨迹文件file/trajectory.txt。
该模式会循环运行
<br>执行命令 
```sh
$ ./robotserver mode2
```
此时终端会显示：

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_20.png" style="width:600px">

<br>该值为再现速度比例，取值范围为0.1-1，即10%-100%，输入正确的值后按Enter键进行下一步。建议第一次运行采用低的再现速度比例，确认路径无误后，可采用较高的再现速度比例。

CTRL + C可停止运行循环运行，此时终端会提示：

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_21.png" style="width:600px">

<br>输入“end”会使机械臂进入电流模式，请用手扶着机械臂。



* mode3
此模式下会逐步记录路径点位，并存储在文件file/data.txt中。
<br>执行命令 
```sh
$ ./robotserver mode3
```
此时终端会提示：

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_30.png" style="width:600px">

<br>该选项为选择插补类型，可选择输入“MOVJ”、“MOVL”、“MOVC”，输入其它值则退出程序。

MOVJ:
关节插补，末端运动轨迹具有不确定性，仅在关节空间进行规划，终端输入“MOVJ”

MOVL:
直线插补，末端运动轨迹为直线，终端输入“MOVL”

MOVC:
圆弧插补，末端运动轨迹为圆弧，圆弧插补需要记录两个点，一个为辅助点，输入“MOVC”
<br>后记录下辅助点，终端如下提示，此时输入“YES”记录圆弧终点，输入其他则退出程序。

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_31.png" style="width:600px">


* mode4
按模式“mode3”生成的路径文件file/data.txt进行再现运行。
<br>执行命令
```sh
$ ./robotserver mode4
```
此时终端会显示：

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_40.png" style="width:600px">

<br>该值为再现速度比例，取值范围为0.1-1，即10%-100%，输入正确的值后按Enter键进行下一步。建议第一次运行采用低的再现速度比例，确认路径无误后，可采用较高的再现速度比例。
<br>确定再现速度比例后终端会显示：

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_41.png" style="width:600px">

<br>该选项为选择再现运行模式，可选择输入“STEP”、“CYCLE”、“CONTINUOUS”，输入其它值则退出程序。

STEP:
单步运行模式，该模式下可输入“FORWARD”运行到下一个点，或者输入“BACKWARD”运行到上一个点，输入其它值则退出程序。

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_42.png" style="width:600px">

CYCLE:
单循环运行模式，即只运行程序一遍，此模式下终端会出现以下界面，输入“YES”以平滑的方式再现路径（注意路径会变形），输入其他则按正常运动方式运行（即点到点运动速度降低为零）。

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_43.png" style="width:600px">

CONTINUOUS:
连续循环运行模式，连续再现“mode3”示教的路径，此模式下终端会出现以下界面，输入“YES”以平滑的方式再现路径（注意路径会变形），输入其他则按正常运动方式运行（即点到点运动速度降低为零）

<img src="../img/桌面级6轴机械臂（QDD Lite-NE30-36版）_v1_0_44.png" style="width:600px">

<br>以上模式均可按下CTRL + C终止程序运行


### 注意事项
在机械臂停止的状态下，在断电之前，请用手托住机械臂并执行mode0关闭电机使能。



## 版本变更记录
**下表简单描述了版本变更记录**

<table style="width:400px"><thead><tr style="background:PaleTurquoise"><th style="width:100px">版本号</th><th style="width:150px">更新时间</th><th style="width:150px">更新内容</th></tr></thead><tbody><tr><td>v1.0.0</td><td>2019.09.05</td><td>全文添加</td></tbody></table>
