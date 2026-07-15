***image_upload_analyzer图生文，origincar_competition巡线+避障，qr_decoder二维码识别***
***话题发布者和接收者需保持统一***
***上位机画面卡顿，可以更换一个带宽较高的路由器或者增加一个网卡（压缩画质降低码率），切换信道等，方法有很多***
***运行大模型图像识别时，将代码里的API_KEY替换为自己的，不清楚在哪里申请请看视频***
***编译之前请将库文件中的libalog.so，libhbmem.so，libion.so放到/usr/hobot/lib/目录下在进行编译***
sudo nmcli device wifi rescan        # 扫描wifi网络
sudo nmcli device wifi list          # 列出找到的wifi网络
sudo wifi_connect "************"  "*********"   # 连接某指定的wifi网络

X5 SD卡扩容：
sudo srpi-config        (6,A1)

X3 SD卡扩容：
df -h
growpart /dev/mmcblk2 2
resize2fs /dev/mmcblk2p2
df -h

深度相机驱动与图像可视化
启动相机
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py
启动rqt_image_view
ros2 run rqt_image_view rqt_image_view


查看芯片工作频率、温度等状态，可通过sudo hrut_somstatus命令查询   **超频请加强散热**
超频：sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
降频：sudo bash -c 'echo 0 > /sys/devices/system/cpu/cpufreq/boost'

将板载天线转化为外置天线 sed -i 's/trace/cable/g' /etc/init.d/hobot-wifi ，重启后生效。 
使用以下命令 sed -i 's/cable/trace/g' /etc/init.d/hobot-wifi 重启后进行复原。

colcon build --packages-select <包名>    //单独编译某一功能包
colcon build							//全部编译
source install/setup.bash				//重新加载环境变量

***car工作空间文件***
//一键换源
wget http://fishros.com/install -O fishros && . fishros

巡线，避障，二维码识别均使用start.launch.py启动

底盘驱动：
ros2 launch origincar_base origincar_bringup.launch.py			//启动底盘

运动控制
ros2 launch origincar_competition start.launch.py			//二维码识别均已包含在当前文件里，无需额外启动

上位机：
ros2 launch rosbridge_server rosbridge_websocket_launch.xml     		//开启上位机端口

图像识别节点：
ros2 run image_upload_analyzer image_upload_analyzer		//启动图像分析节点
安装依赖**pip install tos**//**pip install -U volcengine-python-sdk[ark]**

