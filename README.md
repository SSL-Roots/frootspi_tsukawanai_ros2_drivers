# FrootsPi Drivers

FrootsPiのハードウェアを動かすROS 2パッケージです。

## Requirements

- FrootsPi
- Raspberry Pi (Raspberry Pi 4推奨)
- Ubuntu 20.04
- ROS 2 Foxy

## Installation

```sh
$ MAKEFLAGS=-j1 colcon build --executor sequential --symlink-install
```
