from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from panda3d.core import GeomVertexFormat, GeomVertexData, Geom, GeomTriangles, GeomVertexWriter, TextureAttrib, \
    RenderState, GeomNode, LVector3, Vec3, Point3, DirectionalLight, AmbientLight, Vec4, BitMask32, Fog, \
    TransparencyAttrib
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.OnscreenText import OnscreenText
from panda3d.bullet import BulletWorld, BulletTriangleMesh, BulletDebugNode, BulletTriangleMeshShape, \
    BulletRigidBodyNode, BulletVehicle, ZUp, BulletBoxShape
import inputs
import math
import threading
import json

camera_start = [0, 5, 0.5]

base = ShowBase()
base.setBackgroundColor(0.1, 0.1, 0.8, 1)
base.setFrameRateMeter(True)
base.disableMouse()
base.camera.setPos(*camera_start)
base.camera.setHpr(0, 0, 0)

world = BulletWorld()
world.setGravity(Vec3(0, 0, -9.81))


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

    num_parts = 10 * (arc / 360) ** -1
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


class MyTapper(DirectObject):
    def __init__(self):
        super().__init__()

        self.cam_pos = camera_start

        self.controller = inputs.devices.gamepads[0]
        self.steering = 0
        self.accelerator = 0
        self.brake = 0

        self.gear_ratios = [-4, 3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]
        self.gear = 1
        self.differential_ratio = 4.5
        self.transmission_efficiency = 0.95
        self.wheel_radius = 0.4
        self.drag_coefficient = 0.48
        self.engine_torque_curve = [
            (0, 100),
            (1000, 100),
            (2000, 120),
            (3000, 140),
            (4000, 160),
            (5000, 180),
            (6000, 220),
            (7000, 260),
            (8000, 300),
            (9000, 340),
            (10000, 340),
            (11000, 340),
            (12000, 340),
            (13000, 345),
            (14000, 350),
            (15000, 339),
            (16000, 355),
            (17000, 360),
            (18000, 360),
            (19000, 330),
            (20000, 30)
        ]
        self.fw_wingspan = 0.64
        self.fw_cord = 1.01
        self.fw_clift = 0.7
        self.rw_wingspan = 0.64
        self.rw_cord = 0.51
        self.rw_clift = 0.2

        self.controller_t = threading.Thread(target=self.controller_tf, daemon=True)
        self.controller_t.start()

        track_f = open("track.json")
        track_data = json.load(track_f)
        track_f.close()

        snode = GeomNode('track')
        mesh = BulletTriangleMesh()

        width = track_data["width"]

        cur_x1 = 0
        cur_y1 = 0
        cur_z1 = 0
        cur_x2 = width
        cur_y2 = 0
        cur_z2 = 0

        surfaces = {}
        for name, surface in track_data["surfaces"].items():
            texture = loader.loadTexture(surface["tex"])

            surfaces[name] = {
                "len": surface.get("len"),
                "width": surface.get("width"),
                "tex": texture
            }

        for segment in track_data["segments"]:

            seg_type = segment["type"]
            surface = surfaces[segment["surface"]]
            gradient = segment.get("grade")
            gradient = gradient if gradient is not None else 0

            if seg_type == "str":
                length = segment["length"]
                rise = (gradient / 100) * length
                new_z1 = cur_z1 + rise
                new_z2 = cur_z2 + rise

                x3 = cur_x1 - cur_x2
                y3 = cur_y1 - cur_y2
                z3 = cur_z1 - cur_z2
                x3, y3, z3 = normalized(x3, y3, z3)
                x4, y4, z4 = x3, y3, z3
                x3, y3, z3 = y3, -x3, z3

                section = make_square(cur_x1, cur_y1, cur_z1, cur_x2, cur_y2, cur_z2, cur_x2 + (x3 * segment["length"]),
                                      cur_y2 + (y3 * segment["length"]), new_z2, cur_x1 + (x3 * segment["length"]),
                                      cur_y1 + (y3 * segment["length"]), new_z1, surface["len"], surface["width"])

                left_sides = segment.get("left_side", [])
                for left_side in left_sides:
                    left_surface = surfaces[left_side["surface"]]
                    start_width = left_side["start_width"]
                    end_width = left_side["end_width"]
                    left_section = make_square(cur_x1 + (x4 * start_width), cur_y1 + (y4 * start_width),
                                               cur_z1 + (z4 * start_width),
                                               cur_x1, cur_y1, cur_z1,
                                               cur_x1 + (x3 * segment["length"]), cur_y1 + (y3 * segment["length"]),
                                               new_z1,
                                               cur_x1 + (x3 * segment["length"]) + (x4 * end_width),
                                               cur_y1 + (y3 * segment["length"]) + (y4 * end_width),
                                               new_z1 + (z4 * end_width),
                                               surface["len"], surface["width"])
                    tex_attr = TextureAttrib.make(left_surface["tex"])
                    state = RenderState.make(tex_attr)
                    snode.addGeom(left_section, state)
                    mesh.addGeom(left_section)

                right_sides = segment.get("right_side", [])
                for right_side in right_sides:
                    right_surface = surfaces[right_side["surface"]]
                    start_width = right_side["start_width"]
                    end_width = right_side["end_width"]
                    right_section = make_square(cur_x2, cur_y2, cur_z2,
                                                cur_x2 - (x4 * start_width), cur_y2 - (y4 * start_width), cur_z2,
                                                cur_x2 + (x3 * segment["length"]) - (x4 * end_width),
                                                cur_y2 + (y3 * segment["length"]) - (y4 * end_width), new_z2,
                                                cur_x2 + (x3 * segment["length"]), cur_y2 + (y3 * segment["length"]),
                                                new_z2,
                                                surface["len"], surface["width"])
                    tex_attr = TextureAttrib.make(right_surface["tex"])
                    state = RenderState.make(tex_attr)
                    snode.addGeom(right_section, state)
                    mesh.addGeom(right_section)

                cur_y1 += y3 * segment["length"]
                cur_y2 += y3 * segment["length"]
                cur_x1 += x3 * segment["length"]
                cur_x2 += x3 * segment["length"]
                cur_z1 = new_z1
                cur_z2 = new_z2

            elif seg_type == "arc":
                arc = segment["arc"]
                radius = segment["radius"]
                end_radius = segment.get("end_radius")
                end_radius = end_radius if end_radius is not None else radius

                length = (arc / 360) * math.pi * 2 * (radius if radius >= 0 else -radius)
                rise = (gradient / 100) * length
                new_z1 = cur_z1 + rise
                new_z2 = cur_z2 + rise

                x3 = cur_x1 - cur_x2
                y3 = cur_y1 - cur_y2
                z3 = cur_z1 - cur_z2
                x3, y3, z3 = normalized(x3, y3, z3)
                orig_radius = math.sqrt((cur_x2 - cur_x1) ** 2 + (cur_y2 - cur_y1) ** 2 + (cur_z2 - cur_z1) ** 2)

                if radius < 0:
                    arc = -arc

                left_sides = segment.get("left_side", [])
                for left_side in left_sides:
                    left_surface = surfaces[left_side["surface"]]
                    start_width = left_side["start_width"]
                    end_width = left_side["end_width"]
                    left_section = make_circle(cur_x1, cur_y1, cur_z1,
                                               cur_x1 + (x3 * start_width), cur_y1 + (y3 * start_width),
                                               cur_z1 + (z3 * start_width),
                                               radius + orig_radius, end_radius + orig_radius, arc, new_z1, new_z2,
                                               surface["len"], surface["width"], end_width, False)[0]
                    tex_attr = TextureAttrib.make(left_surface["tex"])
                    state = RenderState.make(tex_attr)
                    snode.addGeom(left_section, state)
                    mesh.addGeom(left_section)

                right_sides = segment.get("right_side", [])
                for right_side in right_sides:
                    right_surface = surfaces[right_side["surface"]]
                    start_width = right_side["start_width"]
                    end_width = right_side["end_width"]
                    right_section = \
                        make_circle(cur_x2 - (x3 * start_width), cur_y2 - (y3 * start_width),
                                    cur_z1 - (z3 * start_width),
                                    cur_x2, cur_y2, cur_z2,
                                    radius - orig_radius,
                                    end_radius - orig_radius,
                                    arc, new_z1, new_z2,
                                    surface["len"], surface["width"], end_width, True)[0]
                    tex_attr = TextureAttrib.make(right_surface["tex"])
                    state = RenderState.make(tex_attr)
                    snode.addGeom(right_section, state)
                    mesh.addGeom(right_section)

                section, cur_x2, cur_y2, cur_z2, cur_x1, cur_y1, cur_z1 = \
                    make_circle(cur_x2, cur_y2, cur_z2, cur_x1, cur_y1, cur_z1, radius, end_radius, arc, new_z1, new_z2,
                                surface["len"], surface["width"])
            else:
                continue

            tex_attr = TextureAttrib.make(surface["tex"])
            state = RenderState.make(tex_attr)
            snode.addGeom(section, state)
            mesh.addGeom(section)

        shape = BulletTriangleMeshShape(mesh, dynamic=False)
        self.track = render.attachNewNode(BulletRigidBodyNode('Track'))
        self.track.node().addShape(shape)
        world.attachRigidBody(self.track.node())
        self.track.attachNewNode(snode)
        self.track.setTwoSided(True)
        self.track.setCollideMask(BitMask32.allOn())

        car = loader.loadModel("car.egg")
        car.flattenLight()
        car_bounds = car.getTightBounds()
        car_shape = BulletBoxShape(Vec3((car_bounds[1].x - car_bounds[0].x) / 2,
                                        (car_bounds[1].y - car_bounds[0].y) / 2,
                                        (car_bounds[1].z - car_bounds[0].z) / 2))

        self.car_node = render.attachNewNode(BulletRigidBodyNode('Car'))
        self.car_node.node().setDeactivationEnabled(False)
        self.car_node.node().setMass(1200)
        self.car_node.node().addShape(car_shape)
        world.attachRigidBody(self.car_node.node())
        car.reparentTo(self.car_node)
        self.car_node.setPos(0, 6, 1)

        self.car = BulletVehicle(world, self.car_node.node())
        self.car.setCoordinateSystem(ZUp)
        world.attachVehicle(self.car)

        speed_dial = OnscreenImage(image='speed360.rgb', pos=(-0.15, 0, 0.15), scale=(0.15, 1, 0.15),
                                   parent=base.a2dBottomCenter)
        speed_dial.setTransparency(TransparencyAttrib.MAlpha)
        self.speed_dial = OnscreenImage(image='dial.rgb', parent=speed_dial)

        rpm_dial = OnscreenImage(image='rpm20000.rgb', pos=(0.15, 0, 0.15), scale=(0.15, 1, 0.15),
                                 parent=base.a2dBottomCenter)
        rpm_dial.setTransparency(TransparencyAttrib.MAlpha)
        self.rpm_dial = OnscreenImage(image='dial.rgb', parent=rpm_dial)
        self.gear_text = OnscreenText(text='1', pos=(0, -0.65), scale=0.4, parent=rpm_dial, fg=(255, 0, 0, 1))

        wheel_fl = loader.loadModel("wheelL")
        wheel_fl.reparentTo(render)
        self.make_wheel(Point3(-0.4, 1.28, 0), True, wheel_fl)

        wheel_fr = loader.loadModel("wheelR")
        wheel_fr.reparentTo(render)
        self.make_wheel(Point3(0.4, 1.28, 0), True, wheel_fr)

        wheel_rl = loader.loadModel("wheelL")
        wheel_rl.reparentTo(render)
        self.make_wheel(Point3(-0.4, -1.35, 0), False, wheel_rl)

        wheel_rr = loader.loadModel("wheelR")
        wheel_rr.reparentTo(render)
        self.make_wheel(Point3(0.4, -1.35, 0), False, wheel_rr)

        dlight = DirectionalLight('dlight')
        dlnp = render.attachNewNode(dlight)
        dlnp.setHpr(0, -80, 0)
        render.setLight(dlnp)
        alight = AmbientLight('alight')
        alight.setColor(Vec4(0.2, 0.2, 0.2, 1))
        alnp = render.attachNewNode(alight)
        render.setLight(alnp)

        myFog = Fog("Fog")
        myFog.setColor(0.2, 0.2, 0.2)
        myFog.setExpDensity(0.005)
        render.setFog(myFog)

        taskMgr.add(self.update, 'update')

    def make_wheel(self, pos, front, np):
        wheel = self.car.createWheel()

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(pos)
        wheel.setFrontWheel(front)
        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(self.wheel_radius)
        wheel.setSuspensionStiffness(40)
        wheel.setWheelsDampingRelaxation(25)
        wheel.setWheelsDampingCompression(52)
        wheel.setRollInfluence(0)
        wheel.setMaxSuspensionTravelCm(20)
        wheel.setMaxSuspensionForce(8000)
        wheel.setFrictionSlip(1.2)

    def update(self, task):
        car_pos = self.car_node.getPos()
        base.camera.lookAt(car_pos)
        car_vec = self.car.getForwardVector()
        camera_distance = -15
        camera_pos = self.car_node.getPos() + Vec3(camera_distance * car_vec.x,
                                                   camera_distance * car_vec.y,
                                                   camera_distance * car_vec.z + 2)
        base.camera.setPos(camera_pos)

        car_speed = self.car.getCurrentSpeedKmHour()
        car_speed_ms = car_speed * 1000 / 3600
        car_speed_ms_abs = abs(car_speed_ms)
        car_speed_abs = abs(car_speed)
        angular_velocity = car_speed_ms / self.wheel_radius

        drag_coefficient = 0.5 * self.drag_coefficient * 1.29 * 5
        drag_force = drag_coefficient * car_speed_ms_abs ** 2
        rr_force = drag_coefficient * 30 * car_speed_ms_abs

        fw_downforce = 0.5 * self.fw_cord * self.fw_wingspan * self.fw_clift * 1.29 * car_speed_ms_abs ** 2
        rw_downforce = 0.5 * self.rw_cord * self.rw_wingspan * self.rw_clift * 1.29 * car_speed_ms_abs ** 2

        self.car_node.node().clearForces()
        self.car_node.node().applyCentralForce(car_vec * -drag_force)
        self.car_node.node().applyCentralForce(car_vec * -rr_force)

        self.car_node.node().applyForce(Vec3(0, 0, -fw_downforce), Point3(-0.61, 2.32, 0.23))
        self.car_node.node().applyForce(Vec3(0, 0, -fw_downforce), Point3(0.61, 2.32, 0.23))
        self.car_node.node().applyForce(Vec3(0, 0, -rw_downforce), Point3(-0.61, -2.47, 1.2))
        self.car_node.node().applyForce(Vec3(0, 0, -rw_downforce), Point3(0.61, -2.47, 1.2))

        rpm = abs(angular_velocity) * self.gear_ratios[self.gear] * self.differential_ratio * 60 / (2 * math.pi)
        engine_torque = self.get_engine_torque(rpm)

        max_engine_force = engine_torque * self.gear_ratios[self.gear] * self.differential_ratio * \
                           self.transmission_efficiency / self.wheel_radius
        engine_force = max_engine_force * (self.accelerator / 128)

        self.car.applyEngineForce(engine_force, 2)
        self.car.applyEngineForce(engine_force, 3)

        self.car.setBrake(self.brake, 0)
        self.car.setBrake(self.brake, 1)
        self.car.setBrake(self.brake, 2)
        self.car.setBrake(self.brake, 3)

        steering_ratio = max((car_speed_abs / 5), 10)
        self.car.setSteeringValue(self.steering / steering_ratio, 0)
        self.car.setSteeringValue(self.steering / steering_ratio, 1)

        dt = globalClock.getDt()
        world.doPhysics(dt, 10, 1 / 100)

        min_rpm = min(self.engine_torque_curve, key=lambda p: p[0])[0]

        self.speed_dial.setHpr(0, 0, car_speed_abs * (270/360))
        self.rpm_dial.setHpr(0, 0, max(rpm, min_rpm) * (270/20000) * (self.accelerator / 128))

        return task.cont

    def get_engine_torque(self, rpm):
        min_rpm = min(self.engine_torque_curve, key=lambda p: p[0])
        max_rpm = max(self.engine_torque_curve, key=lambda p: p[0])

        if rpm <= min_rpm[0]:
            return min_rpm[1]
        elif rpm >= max_rpm[0]:
            return 0

        less_rpm = filter(lambda p: p[0] <= rpm, self.engine_torque_curve)
        more_rpm = filter(lambda p: p[0] >= rpm, self.engine_torque_curve)
        max_less_rpm = max(less_rpm, key=lambda p: p[0])
        min_more_rpm = min(more_rpm, key=lambda p: p[0])

        rpm_diff = min_more_rpm[0] - max_less_rpm[0]
        torque_diff = min_more_rpm[1] - max_less_rpm[1]
        slope = torque_diff / rpm_diff
        diff = rpm - max_less_rpm[0]

        return max_less_rpm[1] + (slope * diff)

    def down_gear(self):
        self.gear -= 1
        if self.gear < 0:
            self.gear = 0
        self.update_gear_text()

    def up_gear(self):
        self.gear += 1
        if self.gear >= len(self.gear_ratios):
            self.gear = len(self.gear_ratios) - 1
        self.update_gear_text()

    def update_gear_text(self):
        if self.gear >= 1:
            text = str(self.gear)
        elif self.gear == 0:
            text = "R"
        else:
            text = ""
        self.gear_text.setText(text)

    def controller_tf(self):
        while True:
            events = self.controller.read()
            for event in events:
                if event.code == "ABS_X":
                    self.steering = -(event.state - 128)
                elif event.code == "ABS_Y":
                    if event.state < 128:
                        self.accelerator = -event.state + 128
                    else:
                        self.brake = event.state - 128

                elif (event.code == "BTN_BASE" or event.code == "BTN_TOP2") and event.state == 1:
                    self.down_gear()
                elif (event.code == "BTN_BASE2" or event.code == "BTN_PINKIE") and event.state == 1:
                    self.up_gear()


debugNode = BulletDebugNode('Debug')
debugNode.showWireframe(True)
debugNode.showConstraints(True)
debugNode.showBoundingBoxes(True)
debugNode.showNormals(True)
debugNP = render.attachNewNode(debugNode)
# debugNP.show()

world.setDebugNode(debugNP.node())

t = MyTapper()
base.run()
