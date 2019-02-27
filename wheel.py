from inputs import devices

stick = devices.gamepads[0]

while True:
    events = stick.read()
    for event in events:
        print(event.ev_type, event.code, event.state)
