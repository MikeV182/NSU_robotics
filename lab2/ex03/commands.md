``` bash
bugwriter@DESKTOP-6RGE64I:~/Documents/NSU_robotics/lab2/ex03$ ros2 doctor --report > full_doctor.txt
bugwriter@DESKTOP-6RGE64I:~/Documents/NSU_robotics/lab2/ex03$ grep -A 10 "PLATFORM INFORMATION\|RWM MIDDLEWARE\|ROS 2 INFORMATION\|TOPIC LIST" full_doctor.txt > doctor.txt
```