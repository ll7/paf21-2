import triangle
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

# define the vertices of the outer boundary, we assume that we have no holes
vertices = [[2.0, 0.0], [3.0, 0.0], [3.5, 1.5], [5.0, 2.0], [4.5, 2.5], [1.5, 1.5]]
# triangulate the polygon
number_of_vertices = len(vertices)
segments = list(zip(range(0, number_of_vertices - 1), range(1, number_of_vertices)))
segments.append((0, number_of_vertices - 1))
triangles = triangle.triangulate({"vertices": vertices, "segments": segments}, opts="pqS2.4")
# convert all triangles to pycrcc.Triangle
mesh = list()
for t in triangles["triangles"]:
    v0 = triangles["vertices"][t[0]]
    v1 = triangles["vertices"][t[1]]
    v2 = triangles["vertices"][t[2]]
    mesh.append(pycrcc.Triangle(v0[0], v0[1], v1[0], v1[1], v2[0], v2[1]))
# create the polygon with the vertices of the outer boundary, the holes, and the triangle mesh
polygon = pycrcc.Polygon(vertices, list(), mesh)

# draw the polygon and its triangle mesh
plt.figure(figsize=(10, 10))
plt.subplot(211)
rnd = MPRenderer()
aabb.draw(rnd, draw_params={"facecolor": "green"})
obb.draw(rnd, draw_params={"facecolor": "red"})
circ.draw(rnd, draw_params={"facecolor": "yellow"})
tri.draw(rnd, draw_params={"facecolor": "blue"})
polygon.draw(rnd, draw_params={"facecolor": "orange", "draw_mesh": False})
rnd.render()


plt.subplot(212)
rnd2 = MPRenderer()
aabb.draw(rnd2, draw_params={"facecolor": "green"})
obb.draw(rnd2, draw_params={"facecolor": "red"})
circ.draw(rnd2, draw_params={"facecolor": "yellow"})
tri.draw(rnd2, draw_params={"facecolor": "blue"})
rnd2.draw_list(mesh, draw_params={"facecolor": "orange", "draw_mesh": False})
rnd2.render()
print("Collision between OBB and AABB: ", obb.collide(aabb))
print("Collision between AABB and Circle: ", aabb.collide(circ))
print("Collision between Circle and OBB:  ", circ.collide(obb))
print("Collision between Triangle and AABB:  ", tri.collide(aabb))
print("Collision between Polygon and Triangle: ", polygon.collide(tri))
print("Collision between Polygon and Circle: ", polygon.collide(circ))
plt.show()
