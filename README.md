# This package modified based laser_filters/box_filter.cpp

## 1. Usage
- .yaml 의 box에 filtering을 원하는 좌표를 string 형식으로 입력
- e.g.) [top_left_x, top_left_y],[top_right_x, top_right_y],[bottom_right_x, bottom_right_y],[bottom_left_x, bottom_left_y]
```
box:"[
        [[0.6980, 0.3730],[0.6980, 0.3430],[0.5980, 0.3430],[0.5980, 0.3730]],
        ...
    ]"
```