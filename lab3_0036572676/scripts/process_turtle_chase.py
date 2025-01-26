#!/usr/bin/env python3
import rosbag
import sys
import math
from lab3_0036572676.msg import ChaserStatus # Import the custom message

def calculate_dist(x_0, y_0, x_f, y_f):
    if (x_0 == None or y_0 == None):
        return 0
    else:
        x = x_0 - x_f
        y = y_0 - y_f
        return math.sqrt(x * x + y * y)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f'Usage: {sys.argv[0]} input.bag topic Name')
        sys.exit()

    # ""../bags" is added to be able to open the bag in the right file
    rosbag_filename = sys.argv[1]
    target_topic = sys.argv[2]

    print(f'Processing input bag: {rosbag_filename}')
    print(f'Target pose topic: {target_topic}')
    message_counter = 0
    new_message_counter = 0

    initial_time = None
    final_time = None
    avr_vel = None

    # Initializate the variable of position
    x_ini = None
    y_ini = None

    # Initializate the position the target
    x_trg = None
    y_trg = None

    dist_tot = 0
    dist_2_trg = -1

    # Open to read & write the rosbag
    rosbag_filename =rosbag_filename
    output_bag_filename = rosbag_filename.replace('.bag', '_processed.bag')
    bag = rosbag.Bag('../bags/' + rosbag_filename, 'r')

    with rosbag.Bag('../bags/' + output_bag_filename, 'w') as output_bag:
        for topic, msg, serialization_time in bag.read_messages():
            if topic == "/turtle1/pose":
                dist_tot = dist_tot + calculate_dist(x_ini, y_ini, msg.x, msg.y)

                if initial_time == None:
                    initial_time = serialization_time
                final_time = (serialization_time - initial_time).to_sec()

                # Write the first topic
                output_bag.write('/chaser/pose', msg, serialization_time)
                
                # Create the ChaserStatus topic
                chaser_sts_topic = ChaserStatus()
                chaser_sts_topic.distance_travelled = dist_tot

                if x_trg == None or y_trg == None:
                        dist_2_trg = -1
                else:
                        dist_2_trg = calculate_dist(msg.x, msg.y, x_trg, y_trg)

                chaser_sts_topic.distance_to_target = calculate_dist(x_ini, y_ini, msg.x, msg.y)
                output_bag.write('/chaser/status', chaser_sts_topic, serialization_time)

                x_ini = msg.x
                y_ini = msg.y
                message_counter += 1

            elif topic == target_topic:
                x_trg = msg.x
                y_trg = msg.y

    avr_vel = dist_tot / final_time
    print(f'Chaser turtle statistics:\n\tCovered distance: {dist_tot}\n\tAverage velocity {avr_vel}')
    print(f'Chase session duration: {final_time} s')
    print(f'Wrote {message_counter} to {output_bag_filename}')
