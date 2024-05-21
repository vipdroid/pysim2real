import lcm
from droid import lcmCtrl


def my_handler(channel, data):
    msg = lcmCtrl.joint_state_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp_ms))
    print("   position    = %s" % str(msg.pc[0]))
    print("   velocity = %s" % str(msg.vc[0]))
    print("   mode        = '%s'" % msg.mode[0])
    print("")

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
subscription = lc.subscribe("joint_state", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass