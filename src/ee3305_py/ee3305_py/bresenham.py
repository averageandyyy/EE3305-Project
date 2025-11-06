def get_bresenham_line(x0, y0, x1, y1):
    # Bresenham's Line Algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    # Will be used to get the grid cells that a line between two points passes through for obstacle checking
    delta_x = abs(x1 - x0)
    delta_y = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = delta_x + delta_y

    points = []
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        err2 = 2 * err
        if err2 >= delta_y:
            err += delta_y
            x0 += sx
        if err2 <= delta_x:
            err += delta_x
            y0 += sy
    return points

def main():
    # Test Cases
    print("Test Case 1: (0,0) to (7,5)")
    points = get_bresenham_line(0, 0, 7, 5)
    print(points)
    print("Test Case 2: (0,0) to (5,7)")
    points = get_bresenham_line(0, 0, 5, 7)
    print(points)
    print("Test Case 3: (-5,-5) to (5,7)")
    points = get_bresenham_line(-5, -5, 5, 7)
    print(points)
    print("Test Case 4: (5,7) to (-5,-5)")
    points = get_bresenham_line(5, 7, -5, -5)
    print(points)

    # Visualize the points using matplotlib
    import matplotlib.pyplot as plt
    for (x0, y0, x1, y1) in [(0, 0, 7, 5), (0, 0, 5, 7), (-5, -5, 5, 7), (5, 7, -5, -5)]:
        points = get_bresenham_line(x0, y0, x1, y1)
        xs, ys = zip(*points)
        plt.figure()
        plt.plot(xs, ys, marker='o')
        plt.title(f"Bresenham Line from ({x0},{y0}) to ({x1},{y1})")
        plt.grid()
        plt.axis('equal')
        plt.show()

if __name__ == "__main__":
    main()

