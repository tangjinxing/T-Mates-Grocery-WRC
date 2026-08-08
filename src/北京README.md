1、导航点位需要全部重新设置：
（1）货架四个放置点位，一面左右放置各一个
（2）两个后退点位，用于放置导航移动发生碰撞
（3）识别小票位置和放置点位需要重新设置，放置的逻辑是获取拍照姿态和放置姿态，只使用放置姿态的x,y和z，保持拍照姿态的rx、ry和rz不变，中间过渡点位代码直接解算
2、将抓取流程连贯，货物种类需要总结和重新多次抓取，速度均调快一倍
3、询问升降机能否降低到五百以下，这将决定能否抓第三层，目前进度是一和二两层
4、机械臂左右臂奇异点规避问题需要确认，右臂移动过程中会突然掉落，左臂未发生这种现象
5、整体逻辑还需要改，需要跟识别小票接口和位姿估计接口重新对齐抓取逻辑


5、执行新指令：
开机重启后，两个夹爪都要重新使能：
左臂夹爪使能：
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python - <<'PY'
import sys
import time

sys.path.insert(0, "/home/lh/robot_api/arm_api_new")
from realman_arm_api_api2 import RealmanArmClient

client = RealmanArmClient(
    ip="169.254.128.18",
    model="left",
    auto_connect=False,
)

try:
    client.connect()

    print("配置夹爪行程 0～1000……")
    client.configure_gripper_range(0, 1000)
    time.sleep(1)

    print("发送夹爪打开命令……")
    client.gripper_release(speed=100, block=True, timeout=10)
    time.sleep(1)

    state = client.get_gripper_state()
    print("夹爪状态:", state)

    if state.enable_state == 1 and state.status == 1 and state.error == 0:
        print("夹爪已成功使能并在线")
    else:
        print("夹爪仍未使能，请检查工具端24V、通信线及控制器夹爪配置")
finally:
    client.disconnect()
PY



右臂夹爪使能：
conda activate hand_eye_calib
cd "/home/lh/WRC/src/control/grasp_pipeline"

python - <<'PY'
import sys
import time

sys.path.insert(0, "/home/lh/robot_api/arm_api_new")
from realman_arm_api_api2 import RealmanArmClient

client = RealmanArmClient(
    ip="169.254.128.19",
    model="right",
    auto_connect=False,
)

try:
    client.connect()

    print("右臂连接成功")
    print("配置右臂夹爪行程 0～1000……")
    client.configure_gripper_range(0, 1000)
    time.sleep(1)

    print("发送右臂夹爪打开命令……")
    client.gripper_release(speed=100, block=True, timeout=10)
    time.sleep(1)

    state = client.get_gripper_state()
    print("右臂夹爪状态:", state)

    if state.enable_state == 1 and state.status == 1 and state.error == 0:
        print("右臂夹爪已成功使能并在线")
    else:
        print(
            "右臂夹爪仍未使能，请检查工具端24V、"
            "通信线及控制器夹爪配置"
        )
finally:
    client.disconnect()
PY


姿态记录页面启动命令：
conda activate hand_eye_calib
cd /home/lh/WRC/src/control/initial_pose_console
python app.py
如果需要新增位姿记录点，就让codex在页面中新增一个记录参数为XXX即可，使用完记录页面需要关闭并释放相机以免影响后续拍照执行。


位姿估计网页：http://192.168.130.59:18086/


右臂第一层：
拍照：
python acquire_sample.py   --mode eye_in_hand_right   --shelf-level 1

拍照位姿包含躯干高度和右臂位姿，躯干会先上升然后才会右臂移动到目标位姿

抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm right \
  --shelf-level 1 \
  --preset level_1_right \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50


右臂第二层：
拍照：
python acquire_sample.py   --mode eye_in_hand_right   --shelf-level 2
抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm right \
  --shelf-level 2 \
  --preset level_2_right \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50

左臂第一层：
拍照：
python acquire_sample.py   --mode eye_in_hand_left   --shelf-level 1
抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm left \
  --shelf-level 1 \
  --preset level_1_left \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50

左臂第二层：
拍照：
python acquire_sample.py   --mode eye_in_hand_left   --shelf-level 2
抓取：
python run_pose_target_6d.py \
  --mode eye_in_hand \
  --arm left \
  --shelf-level 2 \
  --preset level_2_left \
  --up-axis-policy measured \
  --standoff 0.20 \
  --return-lift-mm 50



夹爪松开：
python test_native_gripper.py   --arm left   --open-only   --speed 100

