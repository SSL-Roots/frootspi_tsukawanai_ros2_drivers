# FrootsPi Drivers

FrootsPiのハードウェアを動かすROS 2パッケージです。

## Requirements

- FrootsPi
- Raspberry Pi (Raspberry Pi 4推奨)
- Ubuntu 20.04
- ROS 2 Foxy

## Installation

```sh
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/SSL-Roots/frootspi_ros2_drivers.git

# Install dependencies
$ rosdep install -r -y --from-paths . --ignore-src

$ cd ~/ros2_ws
$ colcon build --symlink-install

# RasPi3ではメモリ不足によりビルドがフリーズすることがあります
# その場合はこちらのコマンドを実行する
$ MAKEFLAGS=-j1 colcon build --executor sequential --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## LICENSE

Apache 2.0

## Examples

### LED & Switch

GPIO19にスイッチ（プルダウン）、GPIO25にLEDを接続。

下記コマンドを実行。

```sh
$ ros2 launch frootspi_examples led_switch.launch.py
```

スイッチを押すとLEDが点灯する。

#### Configure

[led_switch.launch.py](./frootspi_examples/launch/led_switch.launch.py)
のコンテナノードを`component_container`から`component_container_mt`に変更すると、
LEDドライバノードとスイッチドライバノードがマルチスレッドで実行されます。

（処理が重くなる）

```python
container = ComposableNodeContainer(
        name='led_switch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',  # component_container_mtはmulti threads
```
