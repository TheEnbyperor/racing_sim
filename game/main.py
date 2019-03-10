from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from panda3d.core import Vec3, DirectionalLight, AmbientLight, Vec4, Fog
from panda3d.bullet import BulletWorld, BulletDebugNode
import inputs
import threading
import json
import track
import car

base = ShowBase()
base.setBackgroundColor(0.1, 0.1, 0.1, 1)
base.setFrameRateMeter(True)
base.disableMouse()

world = BulletWorld()
world.setGravity(Vec3(0, 0, -9.81))


class RacingSim(DirectObject):
    def __init__(self):
        super().__init__()

        self.controller = inputs.devices.gamepads[0]

        self.cur_track_segment = 0

        track_f = open("track.json")
        track_data = json.load(track_f)
        track_f.close()

        self.track = track.Track(track_data, world)
        self.car = car.Car(base, world)

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
        self.controller_t = threading.Thread(target=self.controller_tf, daemon=True)
        self.controller_t.start()

    def update(self, task):
        car_vec = self.car.forward_vector
        car_pos = self.car.pos
        camera_distance = -15
        camera_pos = car_pos + Vec3(camera_distance * car_vec.x,
                                    camera_distance * car_vec.y,
                                    camera_distance * car_vec.z + 2)
        base.camera.setPos(camera_pos)
        base.camera.lookAt(car_pos)

        contact_segments = [self.car.car_node.node() in y for y in
                            (x.node().getOverlappingNodes() for x in self.track.segments) if len(y) > 0]
        if contact_segments[self.cur_track_segment]:
            self.cur_track_segment += 1
            if self.cur_track_segment == len(self.track.segments):
                self.cur_track_segment = 0
                self.lap()
            print(self.cur_track_segment)


        dt = globalClock.getDt()
        world.doPhysics(dt, 10, 1 / 100)
        return task.cont

    def lap(self):
        print("Lap!")

    def controller_tf(self):
        while True:
            events = self.controller.read()
            for event in events:
                if event.code == "ABS_X":
                    self.car.steering = -(event.state - 128)
                elif event.code == "ABS_Y":
                    if event.state < 128:
                        self.car.accelerator = -event.state + 128
                    else:
                        self.car.brake = event.state - 128

                elif (event.code == "BTN_BASE" or event.code == "BTN_TOP2") and event.state == 1:
                    self.car.down_gear()
                elif (event.code == "BTN_BASE2" or event.code == "BTN_PINKIE") and event.state == 1:
                    self.car.up_gear()


debugNode = BulletDebugNode('Debug')
debugNode.showWireframe(True)
debugNode.showConstraints(True)
debugNode.showBoundingBoxes(True)
debugNode.showNormals(True)
debugNP = render.attachNewNode(debugNode)
debugNP.show()

world.setDebugNode(debugNP.node())

t = RacingSim()
base.run()
