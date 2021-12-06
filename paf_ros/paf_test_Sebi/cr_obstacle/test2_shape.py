import matplotlib.pyplot as plt
import commonroad_dc.pycrcc as pycrcc
from commonroad.visualization.mp_renderer import MPRenderer

# Axis-aligned rectangle with width/2, height/2, x-position , y-position
aabb = pycrcc.RectAABB(2.0, 3.0, 3.0, 2.0)

# Oriented rectangle with width/2, height/2, orientation, x-position , y-position
obb = pycrcc.RectOBB(1.0, 2.0, 0.3, 8.0, 10.0)

# Circle with radius, x-position , y-position
circ = pycrcc.Circle(2.5, 6.0, 7.0)

# Triangle with vertices (x1, y1), (x2, y2), and (x3, y3)
tri = pycrcc.Triangle(0.0, 0.0, 4.0, 0.0, 2.0, 2.0)

rnd = MPRenderer(figsize=(10, 10))
aabb.draw(rnd, draw_params={'facecolor': 'green'})
obb.draw(rnd, draw_params={'facecolor': 'red'})
circ.draw(rnd, draw_params={'facecolor': 'yellow'})
tri.draw(rnd, draw_params={'facecolor': 'blue'})
rnd.render()
plt.show()