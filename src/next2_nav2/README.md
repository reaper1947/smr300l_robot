# next2_nav2

## Overview
แพ็กเกจนี้ใช้สำหรับการนำทางและสร้างแผนที่ด้วย ROS 2 (Nav2 + SLAM Toolbox)

## Launch Files

### 1. สร้างแผนที่ใหม่ด้วย SLAM Toolbox
```bash
ros2 launch next2_nav2 slam_toolbox.launch.py
```
- ใช้ SLAM Toolbox เพื่อสร้างแผนที่ใหม่
- พารามิเตอร์จะถูกอ่านจาก `config/mapper_params_online_async.yaml`

### 2. บันทึกแผนที่ (Save Map)
```bash
ros2 launch next2_nav2 save_map.launch.py
```
- จะบันทึกแผนที่เป็นไฟล์ `config/map_01.yaml` และ `config/map_01.pgm`

### 3. เปิดแผนที่ที่บันทึกไว้ (Map Server)
```bash
ros2 launch next2_nav2 map.launch.py
```
- เปิด map server โดยใช้ไฟล์ `config/map_01.yaml`

## การปรับแต่ง
- สามารถแก้ไขค่าพารามิเตอร์ SLAM ได้ที่ `config/mapper_params_online_async.yaml`
- หากต้องการเปลี่ยนชื่อไฟล์แผนที่ที่บันทึก ให้แก้ไขใน `save_map.launch.py`

## โครงสร้างที่เกี่ยวข้อง
- `launch/` : ไฟล์ launch ทั้งหมด
- `config/` : ไฟล์พารามิเตอร์และแผนที่