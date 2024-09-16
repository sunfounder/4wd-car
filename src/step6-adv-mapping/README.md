
## Table of Contents

- [[#Advanced Mapping Script (Step 6 of Car Lab)|Advanced Mapping Script (Step 6 of Car Lab)]]
	- [[#Advanced Mapping Script (Step 6 of Car Lab)#Mapping|Mapping]]
		- [[#Mapping#`main()` → `map_obj()`|`main()` → `map_obj()`]]
		- [[#Mapping#`map_obj()`|`map_obj()`]]
		- [[#Mapping#`map_obj()` → `interpolate()`|`map_obj()` → `interpolate()`]]
		- [[#Mapping#`interpolate()` → `map_obj()` → `main()`|`interpolate()` → `map_obj()` → `main()`]]
- [[#Footnotes|Footnotes]]

## Advanced Mapping Script (Step 6 of Car Lab)

This script below implements an **advanced mapping algorithm** that detects obstacles using an ultrasonic sensor and maps them relative to the car's current position. While the mapping is performed, it **does not account for the car's movement** or track the car's position dynamically. It therefore does not implement **SLAM**, or (Simultaneous Localization and Mapping).

In order for this script to meet the definition of SLAM, it would have to also update the car's position along the grid as the car moves, as well as the grid itself. However, this script assumes the car is in a **fixed position** and does not update the car's position as it moves through spaces.

### Mapping

#### `main()` → `map_obj()`

```python
	mapping = np.zeros((MAX_MAPPING_DIST * 2, MAX_MAPPING_DIST * 2), dtype=int)
	    
	# Car at bottom center, facing up
	car_pos = [MAX_MAPPING_DIST, 0]
	
	dist = np.array(scan_dist(direct=0))

	mapping = map_obj(mapping, car_pos, dist)
```

`main()` begins with creating a `numpy` matrix of zeros using constants defined at the top. `MAX_MAPPING_DIST` is set to 50cm as a constant and both the X and Y axes were set to **two times** `MAX_MAPPING_DIST` to create a 100x100 NumPy grid array.

We then retrieve an array of raw distance values from `scan_dist` in centimeters, and then we use those values, the position of the car, and the array of zeros to call `map_obj()`.

#### `map_obj()`

```python
def map_obj(mapping, car_pos, dist):
    dist_clip = np.clip(dist, a_min=0, a_max=MAX_MAPPING_DIST)

    angles_in_rad = np.deg2rad(np.array(ANGLE_LIST))
    
    obj_pos = dist_clip * np.array([np.sin(angles_in_rad), np.cos(angles_in_rad)])
    obj_xy = np.int32(np.round(obj_pos, 0)).T

    car_x, car_y = car_pos
    
    obstacle_points = []
    
    for point in obj_xy:
        x = point[0] + car_x
        y = car_y + point[1] # changed to + to place obstacles in front of the car
        if 0 <= x < mapping.shape[1] and 0 <= y < mapping.shape[0]:
            mapping[y][x] = 1  # row x column
            obstacle_points.append((x, y))

    for i in range(1, len(obstacle_points)):
        point1 = obstacle_points[i - 1]
        point2 = obstacle_points[i]
        interpolate(mapping, point1, point2)
        
    visualize_map(mapping, car_pos, obj_xy)
    
    return mapping
```

The function begins by **clipping** the distance readings to ensure all values are within a valid range using:

```python
dist_clip = np.clip(dist, a_min=0, a_max=MAX_MAPPING_DIST)
```

We then convert the degrees from `ANGLE_LIST` to radians.

```python
angles_in_rad = np.deg2rad(np.array(ANGLE_LIST))

obj_pos = dist_clip * np.array([np.sin(angles_in_rad), np.cos(angles_in_rad)])
```

The lines above convert the angles to radians and uses radians to place the obstacles on `XY` coordinates in the Cartesian plane:

$$
x = r \cdot \sin(\theta)
$$

$$
y = r \cdot \cos(\theta)
$$

```python
obj_xy = np.int32(np.round(obj_pos, 0)).T
```

We then transpose these values such that they're easier to process later in the script as coordinate pairs:

```python
# Before transposing:
[[x1, x2, x3, …],
 [y1, y2, y3, …]]

# After transposing:
[[x1, y1],
 [x2, y2],
 [x3, y3],
 …]
```

```python
height, width = mapping.shape
        
    for point in obj_xy:
        x = point[0] + car_x
        y = car_y + point[1]
        if 0 <= x < width and 0 <= y < height:
            mapping[y][x] = 1  # row x column
            obstacle_points.append((x, y))
```

Let's go through one iteration of the above for-loop together. Here are all of the variables for the first `point` in `obj_xy`:

```python
point = array([-1,  0], dtype=int32)
car_x = 20
car_y = 20
x = 19
y = 20
obj_xy = array([[ -1,   0],
       [-35,   3],
       [-36,   6],
       [-56,  15],
       [-53,  19],
		...
       [ 32,   9],
       [ 34,   6],
       [ 35,   3],
       [ 37,   0]], dtype=int32)
```

This for-loop maps each obstacle point. So `x` and `y` come to define the obstacle's position relative to the car.

The if-statement determines if this obstacle is between 0 and the max boundary of the grid (represented by `width` and `height`). If true, that point on the grid `mapping` is marked as an obstacle `mapping[y][x]`, and the coordinates of that obstacle are appended as a tuple to `obstacle_points`, which is an array of obstacle point coordinate tuples.

```python
for i in range(1, len(obstacle_points)):
        point1 = obstacle_points[i - 1]
        point2 = obstacle_points[i]
        interpolate(mapping, point1, point2)
```

After the obstacle-mapping for-loop is over, we now come upon the interpolation loop. For each value in `obstacle_points` starting from 1,[^1] we call the `interpolate` function.

The reason that the output of `interpolate` isn't being assigned to anything (and, later, the reason why `interpolate` doesn't return anything) is because lists and arrays are **mutable** objects in Python, meaning that when passed to functions, the function gets a **reference** to the original object, not a copy. Changes to mutable objects inside of functions/call stacks persist after the function exits. These are known as **in-place** modifications.

#### `map_obj()` → `interpolate()`

```python
def interpolate(mapping, point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    dx = x2 - x1
    dy = y2 - y1
    
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return  

    x_increment = dx / steps
    y_increment = dy / steps

    for step in range(1, steps):
        x = int(round(x1 + step * x_increment))
        y = int(round(y1 + step * y_increment))
        if 0 <= x < mapping.shape[0] and 0 <= y < mapping.shape[1]:
            mapping[y][x] = 1
```

Let's break down this function:

```python
	x1, y1 = point1
    x2, y2 = point2

    dx = x2 - x1
    dy = y2 - y1
    
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return
```

`point1` and `point2` (the two points being interpolated) and broken down into their component XY values and the difference between the horizontal and vertical points is calculated and stored as `dx` and `dy`.

$$
\text{steps} = \max(\left|x_2 - x_1\right|, \left|y_2 - y_1\right|)
$$

We calculate the number of "steps" or intermediate points between the two points by taking the **maximum** of the absolute values of `dx` (the horizontal distance) and `dy` (the vertical distance). This ensures that we interpolate along the longer direction, guaranteeing that the gaps between points are filled with sufficient granularity, whether the two points are farther apart horizontally or vertically. By using the maximum, we make sure that no gaps are left between the points, regardless of the direction of the line connecting them.

As a quick check, we also just return the function and make no modifications if `steps == 0` i.e. `point1` and `point2` are the same and there is nothing to interpolate.

```python
x_increment = dx / steps
y_increment = dy / steps
```

Here we are setting the value for how much we are incrementing along the two axes. If earlier, we found the *number* of steps, now we are determining the *size* of those steps.

```python
	for step in range(1, steps):
        x = int(round(x1 + step * x_i))
        y = int(round(y1 + step * y_i))
        if 0 <= x < mapping.shape[1] and 0 <= y < mapping.shape[0]:
            mapping[y][x] = 1  # row x column
```

First, I want to point out that the for-loop starts from 1 and not from 0. This is because we are *interpolating* so the value of the two points, and the obstacle value of the first point (`point1`) and the last point (`point2`) are already known; they're the two points whose intermediate values we're interpolating.

`x` and `y` values are derived from `x1` or `y1` (the initial obstacle point) *plus* $increments * step$ for each step. This is to find the location of the interpolated obstacle along the XY grid. We are iteratively going through the total number of steps, checking to see if they pass the boundary conditions with the if-statement, and marking that point on the grid with a 1 if true.

Once the for-loop ends, `mapping` is finished being directly modified, and `map_obj()` then calls `visualize_map`. For the sake of brevity, I am not going to provide an in-depth explanation of this function, but I will note one important thing: you will need to install `matplotlib` via `sudo apt-get install python3-matplotlib` on your RPI and then set `VISUALIZE_MAP` to `True` in order to generate images to `OUTPUT_FOLDER`, which is a relative path and will produce files in `${pwd}/maps`.
`map_obj()` then returns back to `main()`.

#### `interpolate()` → `map_obj()` → `visualize_map()` → `main()`

```python
def main():
    mapping = np.zeros((MAX_MAPPING_DIST * 2, MAX_MAPPING_DIST * 2), dtype=int)
    
    # Car at bottom center, facing up
    car_pos = [MAX_MAPPING_DIST, 0]
    
    dist = np.array(scan_dist(direct=0))
    
    mapping = map_obj(mapping, car_pos, dist)

    print("Distance Readings:", list(dist))
    
    for row in mapping:
        print(' '.join(map(str, row)))

if __name__ == "__main__":
    main()
```

Once `map_obj` exits, the script simply prints the distance readings and the binary mapping, and we are done with the mapping and interpolation step, and are ready to get started on localization.

---

## Footnotes

[^1]: We are starting from 1 because the way this loop works is by comparing the *current* value of the list of obstacles with the *previous* value of the list of obstacles; we are only interested in interpolating in-between values, so this way, we are starting with the in-between value for `obstacle_point[0]` and `obstacle_point[1]`.