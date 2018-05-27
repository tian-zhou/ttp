#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class ITEM_CONTROL:
    def __init__(self):
        rospy.init_node('ITEM_CONTROL')
        self.item_id = 0
        self.rail_id = 1
        self.thumbtack_id = 1

        self.item_count_pub = rospy.Publisher('item_count', String, queue_size=10)
        rospy.Subscriber("cmd_pred", String, self.cmd_msgcb)
        
    def cmd_msgcb(self, msg):
        msg_split = msg.data.split('|')
        # cmd = {}
        # cmd['item_id'] = int(msg_split[0])
        # cmd['item_pred'] = msg_split[1].strip(" ")
        # cmd['intent_pred'] = float(msg_split[2])
        # cmd['timestamp'] = msg_split[3].strip(" ")
        # cmd['comment'] = msg_split[4].strip(" ")
        
        item_pred = msg_split[1].strip(" ")
        if item_pred[:9] == 'thumbtack':
            self.thumbtack_id += 1
        if item_pred[:4] == 'rail':
            self.rail_id += 1
        self.item_id += 1

        new_msg = '%i|%i|%i' % (self.item_id, self.rail_id, self.thumbtack_id)
        print "new_msg", new_msg

        self.item_count_pub.publish(new_msg)

    def run(self):
        rospy.spin()

def main():
    ic = ITEM_CONTROL()
    ic.run()

if __name__ == '__main__':
    main()
