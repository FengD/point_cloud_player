#### Point Cloud Player

## Build and Execute
```
Generate <Makefile file>. `qmake build.pro`
Build. `Make`
Execute. `./point_cloud_player`
```

## How to use
1. Initial page.
![Image 1](https://github.com/FengD/point_cloud_player/blob/master/images/1_init.png)
2. Click the "Add" menu bar.
![Image 2](https://github.com/FengD/point_cloud_player/blob/master/images/2_add.png)
3. Add a lidar(online) or a pcap file(offline).
![Image 3](https://github.com/FengD/point_cloud_player/blob/master/images/3_actions.png)

### Version 1.0
```
 * Support the model "VLP16", "P40P", "RS16", "RS32" and "VLP32MR".
 * Display lidar point clouds.
 * Save one frame as PCD file(binary mode).
 * The cloud cloud be display forward or backward.

 BUG:
 * Each time needs to reopen for open a new file.
```

### Version 2.0 (On going)
```
 * Play and pause with multipule speed.
 * Online Mode.(not sure if it is necessary)
 * Bug fixing.
```
