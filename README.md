# myCobot on ROS 2 (Masutani Lab version)

## はじめに

- フォーク元 https://github.com/tylerjw/mycobot
- ROS Foxyで使えるようにすることが目的．

## インストール

ワークスペースを`$W`とする．

```
mkdir -p $W/src
cd $W/src
git clone https://github.com/MasutaniLab/mycobot
vcs import < mycobot/upstream.repos 
rosdep install -r -y -i --from-paths .
cd ..
colcon build
```
