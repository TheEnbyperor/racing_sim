from panda3d.core import TextureAttrib, RenderState, GeomNode, BitMask32
from direct.showbase.DirectObject import DirectObject
from panda3d.bullet import BulletTriangleMesh, BulletTriangleMeshShape, BulletRigidBodyNode, BulletGhostNode
import geom
import math


class Track(DirectObject):
    def __init__(self, track_data, world):
        super().__init__()

        self.track_data = track_data
        self.world = world

        self.snode = GeomNode('track')

        self.width = track_data["width"]

        self.cur_x1 = 0
        self.cur_y1 = 0
        self.cur_z1 = 0
        self.cur_x2 = self.width
        self.cur_y2 = 0
        self.cur_z2 = 0

        self.mesh = BulletTriangleMesh()

        self.surfaces = {}
        self.load_surfaces()

        self.segments = []
        self.gen_segments()
        shape = BulletTriangleMeshShape(self.mesh, dynamic=False)

        self.track = render.attachNewNode(BulletRigidBodyNode('Track'))
        self.track.node().addShape(shape)
        self.world.attachRigidBody(self.track.node())
        self.track.attachNewNode(self.snode)
        self.track.setTwoSided(True)
        self.track.setCollideMask(BitMask32.allOn())

    @property
    def tight_bounds(self):
        return self.track.getTightBounds()

    def load_surfaces(self):
        for name, surface in self.track_data["surfaces"].items():
            texture = loader.loadTexture(f"tex/{surface['tex']}")

            self.surfaces[name] = {
                "len": surface.get("len"),
                "width": surface.get("width"),
                "tex": texture
            }

    def gen_segments(self):
        for segment in self.track_data["segments"]:
            seg_type = segment["type"]
            surface = self.surfaces[segment["surface"]]
            gradient = segment.get("grade")
            gradient = gradient if gradient is not None else 0

            segment_mesh = BulletTriangleMesh()

            if seg_type == "str":
                section = self.gen_str_segment(segment, surface, gradient, segment_mesh)
            elif seg_type == "arc":
                section = self.gen_arc_segment(segment, surface, gradient, segment_mesh)
            else:
                continue

            tex_attr = TextureAttrib.make(surface["tex"])
            state = RenderState.make(tex_attr)
            self.snode.addGeom(section, state)
            self.mesh.addGeom(section)
            segment_mesh.addGeom(section)

            segment_shape = BulletTriangleMeshShape(segment_mesh, dynamic=False)
            segment_ghost = render.attachNewNode(BulletGhostNode('track_segment'))
            segment_ghost.node().addShape(segment_shape)

            self.segments.append(segment_ghost)

    def gen_str_segment(self, segment, surface, gradient, segment_shape):
        length = segment["length"]
        rise = (gradient / 100) * length
        new_z1 = self.cur_z1 + rise
        new_z2 = self.cur_z2 + rise

        x3 = self.cur_x1 - self.cur_x2
        y3 = self.cur_y1 - self.cur_y2
        z3 = self.cur_z1 - self.cur_z2
        x3, y3, z3 = geom.normalized(x3, y3, z3)
        x4, y4, z4 = x3, y3, z3
        x3, y3, z3 = y3, -x3, z3

        section = geom.make_square(self.cur_x1, self.cur_y1, self.cur_z1, self.cur_x2, self.cur_y2, self.cur_z2, self.cur_x2 + (x3 * segment["length"]),
                                   self.cur_y2 + (y3 * segment["length"]), new_z2, self.cur_x1 + (x3 * segment["length"]),
                                   self.cur_y1 + (y3 * segment["length"]), new_z1, surface["len"], surface["width"])

        left_sides = segment.get("left_side", [])
        for left_side in left_sides:
            self.gen_str_left(segment, left_side, x3, y3, x4, y4, z4, new_z1, segment_shape)

        right_sides = segment.get("right_side", [])
        for right_side in right_sides:
            self.gen_str_right(segment, right_side, x3, y3, x4, y4, new_z2, segment_shape)

        self.cur_y1 += y3 * segment["length"]
        self.cur_y2 += y3 * segment["length"]
        self.cur_x1 += x3 * segment["length"]
        self.cur_x2 += x3 * segment["length"]
        self.cur_z1 = new_z1
        self.cur_z2 = new_z2

        return section

    def gen_str_left(self, segment, left_side, x3, y3, x4, y4, z4, new_z1, segment_shape):
        left_surface = self.surfaces[left_side["surface"]]
        start_width = left_side["start_width"]
        end_width = left_side["end_width"]
        left_section = geom.make_square(self.cur_x1 + (x4 * start_width), self.cur_y1 + (y4 * start_width),
                                        self.cur_z1 + (z4 * start_width),
                                        self.cur_x1, self.cur_y1, self.cur_z1,
                                        self.cur_x1 + (x3 * segment["length"]), self.cur_y1 + (y3 * segment["length"]),
                                        new_z1,
                                        self.cur_x1 + (x3 * segment["length"]) + (x4 * end_width),
                                        self.cur_y1 + (y3 * segment["length"]) + (y4 * end_width),
                                        new_z1 + (z4 * end_width),
                                        left_surface["len"], left_surface["width"])
        tex_attr = TextureAttrib.make(left_surface["tex"])
        state = RenderState.make(tex_attr)
        self.snode.addGeom(left_section, state)
        self.mesh.addGeom(left_section)
        segment_shape.addGeom(left_section)

    def gen_str_right(self, segment, right_side, x3, y3, x4, y4, new_z2, segment_shape):
        right_surface = self.surfaces[right_side["surface"]]
        start_width = right_side["start_width"]
        end_width = right_side["end_width"]
        right_section = geom.make_square(self.cur_x2, self.cur_y2, self.cur_z2,
                                         self.cur_x2 - (x4 * start_width), self.cur_y2 - (y4 * start_width), self.cur_z2,
                                         self.cur_x2 + (x3 * segment["length"]) - (x4 * end_width),
                                         self.cur_y2 + (y3 * segment["length"]) - (y4 * end_width), new_z2,
                                         self.cur_x2 + (x3 * segment["length"]), self.cur_y2 + (y3 * segment["length"]),
                                         new_z2,
                                         right_surface["len"], right_surface["width"])
        tex_attr = TextureAttrib.make(right_surface["tex"])
        state = RenderState.make(tex_attr)
        self.snode.addGeom(right_section, state)
        self.mesh.addGeom(right_section)
        segment_shape.addGeom(right_section)

    def gen_arc_segment(self, segment, surface, gradient, segment_shape):
        arc = segment["arc"]
        radius = segment["radius"]
        end_radius = segment.get("end_radius")
        end_radius = end_radius if end_radius is not None else radius

        length = (arc / 360) * math.pi * 2 * (radius if radius >= 0 else -radius)
        rise = (gradient / 100) * length
        new_z1 = self.cur_z1 + rise
        new_z2 = self.cur_z2 + rise

        x3 = self.cur_x1 - self.cur_x2
        y3 = self.cur_y1 - self.cur_y2
        z3 = self.cur_z1 - self.cur_z2
        x3, y3, z3 = geom.normalized(x3, y3, z3)
        orig_radius = math.sqrt(
            (self.cur_x2 - self.cur_x1) ** 2 + (self.cur_y2 - self.cur_y1) ** 2 + (self.cur_z2 - self.cur_z1) ** 2)

        if radius < 0:
            arc = -arc

        left_sides = segment.get("left_side", [])
        for left_side in left_sides:
            self.gen_arc_left(left_side, radius, end_radius, orig_radius, arc, x3, y3, z3, new_z1, new_z2, segment_shape)

        right_sides = segment.get("right_side", [])
        for right_side in right_sides:
            self.gen_arc_right(right_side, radius, end_radius, orig_radius, arc, x3, y3, z3, new_z1, new_z2, segment_shape)

        section, self.cur_x2, self.cur_y2, self.cur_z2, self.cur_x1, self.cur_y1, self.cur_z1 = \
            geom.make_circle(self.cur_x2, self.cur_y2, self.cur_z2, self.cur_x1, self.cur_y1, self.cur_z1, radius,
                             end_radius, arc, new_z1, new_z2, surface["len"], surface["width"])

        return section

    def gen_arc_left(self, left_side, radius, end_radius, orig_radius, arc, x3, y3, z3, new_z1, new_z2, segment_shape):
        left_surface = self.surfaces[left_side["surface"]]
        start_width = left_side["start_width"]
        end_width = left_side["end_width"]
        left_section = geom.make_circle(self.cur_x1, self.cur_y1, self.cur_z1,
                                        self.cur_x1 + (x3 * start_width), self.cur_y1 + (y3 * start_width),
                                        self.cur_z1 + (z3 * start_width),
                                        radius + orig_radius, end_radius + orig_radius, arc, new_z1, new_z2,
                                        left_surface["len"], left_surface["width"], end_width, False)[0]
        tex_attr = TextureAttrib.make(left_surface["tex"])
        state = RenderState.make(tex_attr)
        self.snode.addGeom(left_section, state)
        self.mesh.addGeom(left_section)
        segment_shape.addGeom(left_section)

    def gen_arc_right(self, right_side, radius, end_radius, orig_radius, arc, x3, y3, z3, new_z1, new_z2, segment_shape):
        right_surface = self.surfaces[right_side["surface"]]
        start_width = right_side["start_width"]
        end_width = right_side["end_width"]
        right_section = \
            geom.make_circle(self.cur_x2 - (x3 * start_width), self.cur_y2 - (y3 * start_width),
                             self.cur_z1 - (z3 * start_width),
                             self.cur_x2, self.cur_y2, self.cur_z2,
                             radius - orig_radius,
                             end_radius - orig_radius,
                             arc, new_z1, new_z2,
                             right_surface["len"], right_surface["width"], end_width, True)[0]
        tex_attr = TextureAttrib.make(right_surface["tex"])
        state = RenderState.make(tex_attr)
        self.snode.addGeom(right_section, state)
        self.mesh.addGeom(right_section)
        segment_shape.addGeom(right_section)

