from panda3d.core import GeomVertexFormat, GeomVertexData, Geom, GeomTriangles, GeomVertexWriter, LVector3, Vec3
import math


def normalized(*args):
    myVec = LVector3(*args)
    myVec.normalize()
    return myVec


def make_square(x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, tex_len=None, tex_width=None):
    format = GeomVertexFormat.getV3n3cpt2()
    vdata = GeomVertexData('square', format, Geom.UHDynamic)

    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    texcoord = GeomVertexWriter(vdata, 'texcoord')

    vertex.addData3(x1, y1, z1)
    vertex.addData3(x2, y2, z2)
    vertex.addData3(x3, y3, z3)
    vertex.addData3(x4, y4, z4)

    D = Vec3(x1, y1, z1)
    C = Vec3(x2, y2, z2)
    B = Vec3(x3, y3, z3)
    A = Vec3(x4, y4, z4)
    normal_vec = (C - A).cross(D - B).normalize()

    normal.addData3(normal_vec)
    normal.addData3(normal_vec)
    normal.addData3(normal_vec)
    normal.addData3(normal_vec)

    side_len = math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)
    start_width = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    end_width = math.sqrt((x4 - x3) ** 2 + (y4 - y3) ** 2)

    texcoord.addData2f(0.0, (start_width / tex_width) if tex_width is not None else 1.0)
    texcoord.addData2f(0.0, 0.0)
    texcoord.addData2f((side_len / tex_len) if tex_len is not None else 1.0, 0.0)
    texcoord.addData2f((side_len / tex_len) if tex_len is not None else 1.0,
                       (end_width / tex_width) if tex_width is not None else 1.0)

    # Quads aren't directly supported by the Geom interface
    # you might be interested in the CardMaker class if you are
    # interested in rectangle though
    tris = GeomTriangles(Geom.UHDynamic)
    tris.addVertices(0, 1, 2)
    tris.addVertices(0, 2, 3)

    square = Geom(vdata)
    square.addPrimitive(tris)
    return square


def make_circle(x1, y1, z1, x2, y2, z2, inner_radius, end_inner_radius, arc, new_z1, new_z2, tex_len=None,
                tex_width=None, end_radius=None, rev=False):
    format = GeomVertexFormat.getV3n3cpt2()
    vdata = GeomVertexData('circle', format, Geom.UHDynamic)

    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    texcoord = GeomVertexWriter(vdata, 'texcoord')

    ccw = arc < 0
    arc = -arc if ccw else arc

    num_parts = 50 * (arc / 360) ** -1
    num_draw = round(num_parts * (arc / 360))

    x3 = x1 - x2
    y3 = y1 - y2
    z3 = z1 - z2
    x3, y3, z3 = normalized(x3, y3, z3)

    theta_offset = math.atan2(y1 - y2, x1 - x2)
    orig_radius = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    cur_x1 = x1
    cur_y1 = y1
    cur_z1 = z1
    cur_x2 = x2
    cur_y2 = y2
    cur_z2 = z2

    z1_diff = new_z1 - z1
    z2_diff = new_z2 - z2

    if end_radius is None:
        end_radius = orig_radius

    inner_radius_diff = end_inner_radius - inner_radius
    radius_diff = end_radius - orig_radius

    for i in range(num_draw + 1):
        theta = (math.pi - 2 * math.pi * (i / num_parts)) % (2 * math.pi)
        interp = (i / num_draw)

        inner_radius_interp = inner_radius + (inner_radius_diff * (i / num_parts))

        radius = (orig_radius + (radius_diff * interp)) + inner_radius_interp

        radius_offset = 0
        if rev:
            radius_offset = radius_diff * interp

        x1_interp = x1 + ((inner_radius + radius_offset) * x3)
        y1_interp = y1 + ((inner_radius + radius_offset) * y3)
        z1_interp = z1 + ((inner_radius + radius_offset) * z3)

        x_outer = (math.cos((-theta if ccw else theta) + theta_offset) * radius) + x1_interp
        y_outer = (math.sin((-theta if ccw else theta) + theta_offset) * radius) + y1_interp
        x_inner = (math.cos((-theta if ccw else theta) + theta_offset) * inner_radius_interp) + x1_interp
        y_inner = (math.sin((-theta if ccw else theta) + theta_offset) * inner_radius_interp) + y1_interp

        D = Vec3(cur_x1, cur_y1, cur_z1)
        C = Vec3(cur_x2, cur_y2, cur_z2)
        B = Vec3(x_inner, y_inner, z1_interp + (z1_diff * interp))
        A = Vec3(x_outer, y_outer, z1_interp + (z2_diff * interp))
        normal_vec = (C - A).cross(D - B).normalize()

        vertex.addData3(x_inner, y_inner, z1_interp + (z1_diff * interp))
        cur_x1, cur_y1, cur_z1 = x_inner, y_inner, z1_interp + (z1_diff * interp)
        normal.addData3(normal_vec)

        texcoord.addData2f(
            (((i / num_parts) * 2 * math.pi * inner_radius_interp) / tex_len) if tex_len is not None else 1.0,
            (orig_radius / tex_width) if tex_width is not None else 1.0)

        vertex.addData3(x_outer, y_outer, z1_interp + (z2_diff * interp))
        cur_x2, cur_y2, cur_z2 = x_outer, y_outer, z1_interp + (z2_diff * interp)
        normal.addData3(normal_vec)
        texcoord.addData2f((((i / num_parts) * 2 * math.pi * radius) / tex_len) if tex_len is not None else 1.0, 0)

    tris = GeomTriangles(Geom.UHDynamic)
    for i in range(num_draw * 2):
        tris.addVertices(0 + i, 1 + i, 2 + i)

    circle = Geom(vdata)
    circle.addPrimitive(tris)

    return circle, cur_x1, cur_y1, cur_z1, cur_x2, cur_y2, cur_z2
