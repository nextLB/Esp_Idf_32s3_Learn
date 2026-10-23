

# ESP IDF LEARN

## Window11

### 安装配置烧录工具

在windows11上安装配置烧录工具的话，简单的就是去esp idf的官网下载windows版本的安装包，一路跟着指引去安装配置知道最终可以使用esp idf的cmd或者是powershell的界面即可


### SG90舵机测试Demo    (可见servo_test_demo文件夹下)

第一步  基本的连线

    sg90舵机有红线、黄线、棕线，红线与5V外部电源的正极相连；棕线与外部5V电源的GND相连，5V外部电源的GND再与esp32s3的GND相连；黄线直接与esp32s3的某个spio引脚相连这种即可，这里的demo程序是直接与4号引脚相连的。

第二步  编译烧录测试步骤

    首先，当时项目的初步创建于初始化是使用的 idf.py create-project servo_test_demo 这个指令在idf powershell工具的终端

    切换路径    cd ./servo_test_demo/servo_test_demo

    接下来编译烧录前要设置目标芯片  idf.py set-target esp32s3

    开始进行正式的编译并烧录    idf.py -p COM3 flash monitor

    进行最终5V电源的上电观察, 给外部 5V 电源通电

    舵机应该开始 0° → 180° → 0° 来回扫动，每步 5°，每步停留 50ms，一个来回约 4 秒


