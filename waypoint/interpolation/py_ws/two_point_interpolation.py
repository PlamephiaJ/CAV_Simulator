import csv
import numpy as np
x1, y1 = 0.0, 0.0
x2, y2 = 2.210431, 117.104736

n_points = 2000

x = np.linspace(x1, x2, n_points)
y = np.linspace(y1, y2, n_points)

speed = [10] * n_points

with open('straight.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['x', 'y', 'speed'])
    for i in range(n_points):
        writer.writerow([x[i], y[i], speed[i]])

print('CSV file generated successfully!')