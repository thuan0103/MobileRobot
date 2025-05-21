# MobileRobot

Dự án này mô phỏng một xe robot di chuyển tự động trong môi trường ảo (sử dụng Gazebo + ROS 2), với mục tiêu đến tọa độ đích **(4, -10)**. Robot được điều khiển thông qua **mạng nơ-ron nhân tạo (ANN)** và được huấn luyện/bối ưu thông qua **thuật toán di truyền (Genetic Algorithm - GA)**.

## 🚀 Mục tiêu dự án
- Mô phỏng một robot di chuyển trong môi trường Gazebo.
- Áp dụng mạng neural để học cách điều khiển robot đến vị trí mục tiêu.
- Sử dụng thuật toán di truyền để tối ưu các tham số của mạng neural.
- Đánh giá hiệu quả điều khiển dựa trên quãng đường, thời gian và sai số vị trí.

## 🛠 Công nghệ sử dụng
- **ROS 2 (Foxy/Humble)** – Framework cho robot.
- **Gazebo** – Môi trường mô phỏng 3D.
- **Python** – Ngôn ngữ lập trình chính.
- **NumPy** – Xử lý tính toán số.
- **Neural Network** – Tự xây dựng hoặc dùng PyTorch (tùy chọn).
- **Genetic Algorithm** – Tự cài đặt để huấn luyện mạng.

## 🧠 Cấu trúc điều khiển
- **Input**: 16 đầu vào bao gồm:
  - 11 giá trị từ cảm biến khoảng cách (LiDAR).
  - 2 tọa độ hiện tại (x, y).
  - Góc định hướng hiện tại (yaw).
  - 2 tọa độ mục tiêu (x_goal, y_goal).
- **Output**: 
  - Vận tốc tuyến tính (`linear.x`).
  - Vận tốc góc (`angular.z`).

# MobileRobot
This project simulates autonomous robot navigating in a virtual enviroment (using Gazebo + ROS 2),aiming reach a target coordinate at **(4, -10)**. The robot is controlled by an **Artificial Neural Network (ANN)**, which is trianed and optimized using a **Genetic Algorithm (GA)**.

## 🚀 Project Objective
- Simulate a robot navigating in the Gazebo environment.
- Apply a Neural Network to learn how to control the robot toward the target location.
- Use a genetic algorithm to optimize the neural network parameters.
- Evaluate the control performance based on distance traveled, time taken, and final position error.

## 🛠 Technologies Used
- ROS 2 (Foxy/Humble) – Robotics framework.
- Gazebo – 3D simulation environment.
- Python – Main programming language.
- NumPy – For numerical computation.
- Neural Network – Either custom-built or using PyTorch (optional).
- Genetic Algorithm – Custom implementation for training the network.

## 🧠 Control Architecture
- **Input**: 16 inputs including:
  - 11 distance sensor (LiDAR) readings.
  - 2 current coordinates (x, y).
  - Current orientation angle (yaw).
  - 2 target coordinates (x_goal, y_goal).
- **Output**:
  - Linear velocity (linear.x).
  - Angular velocity (angular.z).
