from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import Vec3, Point3, TransparencyAttrib, TransformState
from panda3d.bullet import BulletRigidBodyNode, BulletVehicle, ZUp, BulletBoxShape
import math


class Car(DirectObject):
    def __init__(self, base, world, track):
        super().__init__()

        self.world = world
        self.track = track

        self.steering = 0
        self.accelerator = 0
        self.brake = 0

        self.gear_ratios = [-4, 0, 3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]
        self.gear = 1
        self.differential_ratio = 4.5
        self.transmission_efficiency = 0.95
        self.wheel_radius = 0.4
        self.drag_coefficient = 0.48
        self.engine_torque_curve = [
            (0, 466),
            (564, 469),
            (1612, 469),
            (2822, 518),
            (3870, 517),
            (4516, 597),
            (5000, 613),
            (5564, 600),
            (6048, 655),
            (6693, 681),
            (7177, 716),
            (7822, 696),
            (8306, 696),
            (11048, 569),
            (13951, 391),
            (15000, 339),
            (15483, 301),
            (16612, 247),
            (17177, 65),
            (18306, 55)
        ]
        self.fw_wingspan = 0.64
        self.fw_cord = 1.01
        self.fw_clift = 0.7
        self.rw_wingspan = 0.64
        self.rw_cord = 0.51
        self.rw_clift = 0.2

        self.car_node = None
        self.car = None

        dial_scale = 0.2

        speed_dial = OnscreenImage(image='tex/speed360.rgb', pos=(-dial_scale, 0, dial_scale),
                                   scale=(dial_scale, 1, dial_scale),
                                   parent=base.a2dBottomCenter)
        speed_dial.setTransparency(TransparencyAttrib.MAlpha)
        self.speed_dial = OnscreenImage(image='tex/dial.rgb', parent=speed_dial)

        rpm_dial = OnscreenImage(image='tex/rpm20000.rgb', pos=(dial_scale, 0, dial_scale),
                                 scale=(dial_scale, 1, dial_scale),
                                 parent=base.a2dBottomCenter)
        rpm_dial.setTransparency(TransparencyAttrib.MAlpha)
        self.rpm_dial = OnscreenImage(image='tex/dial.rgb', parent=rpm_dial)
        self.gear_text = OnscreenText(text='N', pos=(-0.02, -0.67), scale=0.4, parent=rpm_dial, fg=(255, 0, 0, 1))

        self.reset()
        taskMgr.add(self.update, 'update')

    def make_wheel(self, pos, front, np):
        wheel = self.car.createWheel(0.02)

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(pos)
        wheel.setFrontWheel(front)
        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(self.wheel_radius)
        wheel.setSuspensionStiffness(200)
        wheel.setWheelsDampingRelaxation(23)
        wheel.setWheelsDampingCompression(44)
        wheel.setRollInfluence(0.1)
        wheel.setMaxSuspensionTravelCm(10)
        wheel.setFrictionSlip(1.2)
        # wheel.setMaxSuspensionForce(1000)

    def reset(self):
        car = loader.loadModel("models/car.egg")
        car.flattenLight()
        car_bounds = car.getTightBounds()
        car_shape = BulletBoxShape(Vec3((car_bounds[1].x - car_bounds[0].x) / 2,
                                        (car_bounds[1].y - car_bounds[0].y) / 2,
                                        (car_bounds[1].z - car_bounds[0].z) / 2))
        car_ts = TransformState.makePos(Point3(0, 0, 0.5))

        if self.car_node is not None:
            self.car_node.removeNode()

        self.car_node = render.attachNewNode(BulletRigidBodyNode('Car'))
        self.car_node.node().setDeactivationEnabled(False)
        self.car_node.node().setMass(600)
        self.car_node.node().addShape(car_shape, car_ts)
        self.world.attachRigidBody(self.car_node.node())
        car.reparentTo(self.car_node)
        self.car_node.setPos(0, 6, 1)

        self.car = BulletVehicle(self.world, self.car_node.node())
        self.car.setCoordinateSystem(ZUp)
        self.world.attachVehicle(self.car)

        self.car_node.setPos(0, 6, 1)
        self.car_node.setHpr(0, 0, 0)
        self.car.resetSuspension()
        self.car_node.node().clearForces()

        wheel_fl = loader.loadModel("models/wheelL")
        wheel_fl.reparentTo(render)
        self.make_wheel(Point3(-0.4, 1.28, 0), True, wheel_fl)

        wheel_fr = loader.loadModel("models/wheelR")
        wheel_fr.reparentTo(render)
        self.make_wheel(Point3(0.4, 1.28, 0), True, wheel_fr)

        wheel_rl = loader.loadModel("models/wheelL")
        wheel_rl.reparentTo(render)
        self.make_wheel(Point3(-0.4, -1.35, 0), False, wheel_rl)

        wheel_rr = loader.loadModel("models/wheelR")
        wheel_rr.reparentTo(render)
        self.make_wheel(Point3(0.4, -1.35, 0), False, wheel_rr)

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

    @property
    def pos(self):
        return self.car_node.getPos()

    @property
    def forward_vector(self):
        return self.car.getForwardVector()

    def update(self, task):
        car_pos = self.pos
        car_vec = self.forward_vector
        track_bounds = self.track.tight_bounds

        if car_pos.x < track_bounds[0].x or \
                car_pos.x > track_bounds[1].x or \
                car_pos.y < track_bounds[0].y or \
                car_pos.y > track_bounds[1].y or \
                car_pos.z < track_bounds[0].z:
            self.reset()
            return task.cont

        car_speed = self.car.getCurrentSpeedKmHour()
        car_speed_ms = car_speed * 1000 / 3600
        car_speed_abs = abs(car_speed)

        self.car_node.node().clearForces()

        self.apply_wing_force(car_speed_ms)
        self.apply_drag_force(car_speed_ms, car_vec)
        rpm = self.apply_engine_force(car_speed_ms)
        self.apply_brake_force()
        self.apply_steering_moment(car_speed_abs)
        self.update_dials(rpm, car_speed_abs)

        return task.cont

    def apply_wing_force(self, car_speed_ms):
        fw_downforce = 0.5 * self.fw_cord * self.fw_wingspan * self.fw_clift * 1.29 * car_speed_ms ** 2
        rw_downforce = 0.5 * self.rw_cord * self.rw_wingspan * self.rw_clift * 1.29 * car_speed_ms ** 2

        self.car_node.node().applyForce(Vec3(0, 0, -fw_downforce), Point3(-0.61, 2.32, 0.23))
        self.car_node.node().applyForce(Vec3(0, 0, -fw_downforce), Point3(0.61, 2.32, 0.23))
        self.car_node.node().applyForce(Vec3(0, 0, -rw_downforce), Point3(-0.61, -2.47, 1.2))
        self.car_node.node().applyForce(Vec3(0, 0, -rw_downforce), Point3(0.61, -2.47, 1.2))

    def apply_drag_force(self, car_speed_ms, car_vec):
        drag_coefficient = 0.5 * self.drag_coefficient * 1.29 * 5
        drag_force = drag_coefficient * car_speed_ms ** 2
        rr_force = drag_coefficient * 30 * car_speed_ms
        if car_speed_ms < 0:
            drag_force = -drag_force

        self.car_node.node().applyCentralForce(car_vec * -drag_force)
        self.car_node.node().applyCentralForce(car_vec * -rr_force)

    def apply_engine_force(self, car_speed_ms):
        angular_velocity = car_speed_ms / self.wheel_radius
        rpm = angular_velocity * self.gear_ratios[self.gear] * self.differential_ratio * 60 / (2 * math.pi)
        rpm = max(0, rpm)
        engine_torque = self.get_engine_torque(rpm)

        max_engine_force = engine_torque * self.gear_ratios[self.gear] * self.differential_ratio * \
                           self.transmission_efficiency / self.wheel_radius
        engine_force = max_engine_force * (self.accelerator / 128)

        self.car.applyEngineForce(engine_force, 2)
        self.car.applyEngineForce(engine_force, 3)
        return rpm

    def apply_brake_force(self):
        brake_adj = 5
        self.car.setBrake(self.brake * brake_adj, 0)
        self.car.setBrake(self.brake * brake_adj, 1)
        self.car.setBrake(self.brake * brake_adj, 2)
        self.car.setBrake(self.brake * brake_adj, 3)

    def apply_steering_moment(self, car_speed_abs):
        steering_ratio = max((car_speed_abs / 3), 30)
        self.car.setSteeringValue(self.steering / steering_ratio, 0)
        self.car.setSteeringValue(self.steering / steering_ratio, 1)

    def update_dials(self, rpm, car_speed_abs):
        min_rpm = min(self.engine_torque_curve, key=lambda p: p[0])[0]
        if self.gear == 1:
            min_rpm = max(self.engine_torque_curve, key=lambda p: p[0])[0]

        self.speed_dial.setHpr(0, 0, car_speed_abs * (270 / 360))
        self.rpm_dial.setHpr(0, 0, max(rpm, min_rpm) * (270 / 20000) * (self.accelerator / 128))

    def update_gear_text(self):
        if self.gear >= 2:
            text = str(self.gear - 1)
        elif self.gear == 0:
            text = "R"
        elif self.gear == 1:
            text = "N"
        else:
            text = ""
        self.gear_text.setText(text)

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
