

# ESP IDF LEARN

    注意若要更新项目，请先用 idf.py fullclean   这个清空build后再上传git

    
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



### 简易呼吸灯的Demo    （可见breath_led_demo文件夹下）

    同上，正常的编译烧录后，运行成功的终端输出字样与效果说明如下：

    终端输出：I (266) main_task: Started on CPU0
            I (286) main_task: Calling app_main()
            I (286) ws2812_breath: Color-cycling breath on GPIO48, 7 colors
            I (286) ws2812_breath: Color: Red
            I (1206) ws2812_breath: Color: Green
            I (2126) ws2812_breath: Color: Blue
            I (3046) ws2812_breath: Color: Yellow
            I (3966) ws2812_breath: Color: Cyan
            I (4886) ws2812_breath: Color: Magenta
            I (5806) ws2812_breath: Color: White
            I (6726) ws2812_breath: Color: Red
            I (7646) ws2812_breath: Color: Green
            I (8566) ws2812_breath: Color: Blue
            ......
    效果就是灯一闪一闪的还换颜色


### 简易多路舵机测试Demo    (可见mullti_servo_demo文件夹下)

    第一步  基本连线

        PCA9685与esp32s3

            GND --> GND
            SCL --> GPIO9
            SDA --> GPIO8
            VCC --> 3.3V
        
        PCA9685与外部5V电源连接，通过那个绿色的引脚

        PCA9685与多路舵机，随意插就行，主要是测试能不能动的Demo

    
    第二步按照上述编译烧录并观察现象即可


### sg90舵机控制     （可见sg90_servo_control文件夹下）



### sf006舵机控制       （可见sf006_servo_control文件夹下）


### pca9685多路舵机具体控制     （可见pca9685_servo_control文件夹下）





### 常见的故障与解决办法

    关于esp32s3编译烧录上的问题，常用按着Boot键，按一次Ret键后，松开Boot这样清空解决
