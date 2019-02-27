import math
from panda3d.core import GeomVertexFormat, GeomVertexData, Geom, GeomTriangles, GeomVertexWriter, Texture, GeomNode, PerspectiveLens, Light, Spotlight, LVector3


def normalized(*args):
    myVec = LVector3(*args)
    myVec.normalize()
    return myVec


def make_circle(x1, y1, z1, x2, y2, z2, inner_radius, end_inner_radius, arc, new_z1, new_z2):
    format = GeomVertexFormat.getV3n3cpt2()
    vdata = GeomVertexData('circle', format, Geom.UHDynamic)

    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    texcoord = GeomVertexWriter(vdata, 'texcoord')

    ccw = arc < 0
    arc = -arc if ccw else arc

    num_parts = 2 * (arc / 360)**-1
    num_draw = round(num_parts * (arc / 360))

    x3 = x1-x2
    y3 = y1-y2
    z3 = z1-z2
    x3, y3, z3 = normalized(x3, y3, z3)

    print(x3, y3)

    theta_offset = math.atan2(y1 - y2, x1 - x2)
    orig_radius = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

    cur_x1 = 0
    cur_y1 = 0
    cur_z1 = 0
    cur_x2 = 0
    cur_y2 = 0
    cur_z2 = 0

    z1_diff = new_z1 - z1
    z2_diff = new_z2 - z2

    inner_radius_diff = end_inner_radius - inner_radius

    x1_interp = x1 + (inner_radius * x3)
    y1_interp = y1 + (inner_radius * y3)
    z1_interp = z1 + (inner_radius * z3)
    z2_interp = z2 + (inner_radius * z3)

    print(x1_interp, y1_interp)
    print(theta_offset)

    for i in range(num_draw+1):
        theta = ((2 * -math.pi * i / num_parts) + math.pi) % (2 * math.pi)
        interp = (i/num_draw)

        inner_radius_interp = inner_radius + (inner_radius_diff * interp)
        # inner_radius_interp = inner_radius

        radius = orig_radius + inner_radius_interp

        x_outer = math.cos((-theta if ccw else theta) + theta_offset) * radius
        y_outer = math.sin((-theta if ccw else theta) + theta_offset) * radius
        x_inner = math.cos((-theta if ccw else theta) + theta_offset) * inner_radius_interp
        y_inner = math.sin((-theta if ccw else theta) + theta_offset) * inner_radius_interp

        print(theta, x_inner + x1_interp, y_inner + y1_interp, inner_radius_interp, theta + theta_offset)

        vertex.addData3(x1_interp + x_inner, y1_interp + y_inner, z1_interp + (z1_diff * interp))
        cur_x1, cur_y1, cur_z1 = x1_interp + x_inner, y1_interp + y_inner, z1_interp + (z1_diff * interp)
        normal.addData3(normalized(2 * (x1_interp + x_inner) - 1, 2 * (y1_interp + y_inner) - 1,
                                   2 * (z1_interp + (z1_diff * interp)) - 1))
        texcoord.addData2f(((i/num_parts)*2*math.pi*inner_radius_interp)/13, 1)

        vertex.addData3(x1_interp + x_outer, y1_interp + y_outer, z2_interp + (z2_diff * interp))
        cur_x2, cur_y2, cur_z2 = x1_interp + x_outer, y1_interp + y_outer, z2_interp + (z2_diff * interp)
        normal.addData3(normalized(2 * (x1_interp + x_outer) - 1, 2 * (y1_interp + y_outer) - 1
                                   , 2 * (z2_interp + (z2_diff * interp)) - 1))
        texcoord.addData2f(((i/num_parts)*2*math.pi*radius)/13, 0)

    tris = GeomTriangles(Geom.UHDynamic)
    for i in range(num_draw*2):
        tris.addVertices(0+i, 1+i, 2+i)

    circle = Geom(vdata)
    circle.addPrimitive(tris)

    return circle, cur_x1, cur_y1, cur_z1, cur_x2, cur_y2, cur_z2


make_circle(6, 0, 0, 0, 6, 0, 5, 3, 90, 0, 0)
