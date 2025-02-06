<div align="center">
    <h1>Simple ROS2 pointcloud vertical filter in python</h1>
</div>

## :open_file_folder: What's in this repo

* Python filter file

## :package: Prerequisite packages
> What we need are ros2 humble and numpy.

```commandline
sudo apt-get install python3-numpy -y
```
## :gear: How to build & Run
```commandline
git clone https://github.com/leonardonels/PCL2_filter_project.git
```
```commandline
python3 filter.py
```
> To change how the filter works modify the **vertical_zones** list inside params.yaml
```commandline
vertical_zones: [
          {'start': 0.0, 'end': 0.25, 'downsample': 4},  # Upper 25% of rows, keep 1/4
          {'start': 0.25, 'end': 0.75, 'downsample': 1},  # Middle 50%, keep all
          {'start': 0.75, 'end': 1.0, 'downsample': 4},  # Lower 25%, keep 1/4
]
```
