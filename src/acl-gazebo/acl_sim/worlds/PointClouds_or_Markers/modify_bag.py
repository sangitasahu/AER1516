import rosbag

with rosbag.Bag('density01seed0_buena.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('density01seed0.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/forest":
            #msg.header.frame_id='world'
            outbag.write("/forest", msg)
            break
