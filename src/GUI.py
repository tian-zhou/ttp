#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pandas as pd 
from tkinter import Tk, Label, Button, Entry
from datetime import datetime


class GUI:
    def __init__(self):
        rospy.init_node('GUI_node')
        rospy.Subscriber("cmd_pred", String, self.cmd_pred_msgcb)
        self.cmd_queue = pd.DataFrame(columns = ['item_id', 'item_pred', 
                    'intent_pred', 'timestamp', 'comment', 'status'])
        rospy.Subscriber("motion_plan_pub", String, self.motion_plan_msgcb)
        # self.motion_queue = pd.DataFrame(columns = ['item_id', 'item_pred', 'step', 
                    # 'motion', 'status', 'timestamp'])
        self.motion_msg_queue = []
        self.abort_signal_pub = rospy.Publisher('abort_command_pub', String, queue_size=10)
        self.cmd_pub = rospy.Publisher('cmd_pred', String, queue_size=10)
        
        self.cmd_but_count = 0
        self.cmd_but_dict = {}

        status = {}
        status['left stile'] =  0
        for i in range(1, 7):
            status['rail'+str(i)] = 0
        status['seat'] = 0
        status['right stile'] = 0
        status['scissors'] = 0
        status['marker'] = 0
        status['tape'] = 0
        status['sticky notes'] = 0
        status['thumbtack'] = 16
        self.status = status

    def init_window(self):
        self.window = Tk()
        self.window.title("Turn-taking")
        self.window.geometry('500x1000') # width x height
        
    def cmd_pred_msgcb(self, msg):
        msg_split = msg.data.split('|')
        cmd = {}
        cmd['item_id'] = int(msg_split[0])
        cmd['item_pred'] = msg_split[1].strip(" ")
        cmd['intent_pred'] = float(msg_split[2])
        cmd['timestamp'] = msg_split[3].strip(" ")
        cmd['comment'] = msg_split[4].strip(" ")
        cmd['status'] = 'received'
        self.cmd_queue = self.cmd_queue.append(cmd, ignore_index=True)

        display_msg = '| %i | %.2f | %s | %s | %s | %s | ' % (cmd['item_id'], 
            cmd['intent_pred'], cmd['timestamp'], cmd['status'], cmd['comment'], cmd['item_pred'])
        
        self.but = Button(self.window, text=display_msg, bg='green', command=lambda:self.abort(cmd['item_id'], cmd['item_pred']))
        self.but.grid(column=0, row=11 + self.cmd_but_count, columnspan=2)
        self.cmd_but_count += 1

        self.cmd_but_dict[cmd['item_id']] = self.but
        
    def motion_plan_msgcb(self, msg):
        # self.motion_msg_queue.append(msg.data)
        # self.lab.configure(text=self.format_msg())
        msg_split = msg.data.split('|')
        motion = {}
        motion['item_id'] = int(msg_split[0])
        motion['item_pred'] = msg_split[1].strip(" ")
        motion['step'] = int(msg_split[2])
        motion['motion'] = msg_split[3].strip(" ")
        motion['status'] = msg_split[4].strip(" ")
        motion['timestamp'] = msg_split[5].strip(" ")

        # now do something about the motion
        # print '--motion:--\n', motion
        # print "--self.cmd_queue:--\n", self.cmd_queue

        # based on motion['item_id'] and motion['item_pred'], find the cmd_but, change it to red
        # once the status is "done_last", change to grey
        while True:
            cmd = self.cmd_queue.loc[(self.cmd_queue['item_id'] == motion['item_id']) & 
                                     (self.cmd_queue['item_pred'] == motion['item_pred'])]
            if len(cmd) == 0:
                continue
            cmd_but_id = cmd['item_id'].values[0]
            if cmd_but_id not in self.cmd_but_dict:
                continue
            break

        if motion['status'] == 'done_last':
            self.cmd_but_dict[cmd_but_id].config(bg='grey')
        elif motion['status'] in ['to_do', 'to_do_last']:
            self.cmd_but_dict[cmd_but_id].config(bg='green')
        else:
            self.cmd_but_dict[cmd_but_id].config(bg='red')
            
        # self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

    def abort(self, cmd_id, cmd_name):
        button = self.cmd_but_dict[cmd_id]
        button.config(bg='orange')
        print "abort number %i: %s" % (cmd_id, cmd_name)
        print "publish this message!!!"
        self.abort_signal_pub.publish("abort|%i|%s" % (cmd_id, cmd_name))

    def pub_cmd(self, part):
        if len(self.cmd_queue.tail(1)):
            last_item_id = self.cmd_queue.tail(1)['item_id'].values[0]
        else:
            last_item_id = -1
        msg = '%i|%s|1.00|' % (last_item_id+1, part)
        msg += '%s|' % datetime.now().strftime('%H:%M:%S:%f')[:-4] # %Y_%m_%d_%H_%M_%S_%f
        msg += '%s' % 'GUI_button'
        self.cmd_pub.publish(msg)

    def update_status_left(self, part, newstate):
        self.status[part] = newstate
        print "new state [%s] for part [%s]" % (newstate, part)
        self.but_left.config(bg='grey')
        self.pub_cmd(part)

    def update_status_rail6(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_rail6.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_rail5(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_rail5.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_rail4(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_rail4.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_rail3(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_rail3.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_rail2(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_rail2.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_rail1(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_rail1.config(bg='grey')
        self.pub_cmd(part)

    def update_status_seat(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_seat.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_right(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_right.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_scissors(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_scissors.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_marker(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_marker.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_tape(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_tape.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_notes(self, part, newstate):
        self.status[part] = newstate
        # print "new state [%s] for part [%s]" % (newstate, part)
        self.but_notes.config(bg='grey')
        self.pub_cmd(part)
        
    def update_status_thumbtack(self, part):
        self.pub_cmd(part + '%i' % (17-self.status[part]))
        self.status[part] -= 1
        if self.status[part] == 0:
            self.but_thumbtack.config(bg='grey')
        self.but_thumbtack.configure(text="%i thumbtacks" % self.status[part])
        # print "[%i] thumbtacks available..." % self.status[part]

    def refresh_cmd_queue(self):
        print "refresh cmd queue..."
        for key in self.cmd_but_dict:
            self.cmd_but_dict[key].grid_forget()
        self.cmd_but_count = 0    
        # destroy is too much work (will see errors for color updating), just hide it

    def run(self):
        self.header = Label(self.window, text="Button control", font=("Arial", 20))
        self.header.grid(column=0, row=0, columnspan=2)

        self.but_left = Button(self.window, text="left stile", font=(15), width=10, bg='blue',
            command=lambda:self.update_status_left('left stile', 'used'))
        self.but_left.grid(column=0, row=1)

        self.but_rail6 = Button(self.window, text="rail6", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_rail6('rail6', 'used'))
        self.but_rail6.grid(column=0, row=2)

        self.but_rail5 = Button(self.window, text="rail5", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_rail5('rail5', 'used'))
        self.but_rail5.grid(column=0, row=3)

        self.but_rail4 = Button(self.window, text="rail4", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_rail4('rail4', 'used'))
        self.but_rail4.grid(column=0, row=4)

        self.but_rail3 = Button(self.window, text="rail3", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_rail3('rail3', 'used'))
        self.but_rail3.grid(column=0, row=5)

        self.but_rail2 = Button(self.window, text="rail2", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_rail2('rail2', 'used'))
        self.but_rail2.grid(column=0, row=6)

        self.but_rail1 = Button(self.window, text="rail1", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_rail1('rail1', 'used'))
        self.but_rail1.grid(column=0, row=7)

        self.but_seat = Button(self.window, text="seat", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_seat('seat', 'used'))
        self.but_seat.grid(column=0, row=8)

        self.but_right = Button(self.window, text="right stile", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_right('right stile', 'used'))
        self.but_right.grid(column=1, row=1)

        self.but_scissors = Button(self.window, text="scissors", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_scissors('scissors', 'used'))
        self.but_scissors.grid(column=1, row=2)

        self.but_marker = Button(self.window, text="marker", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_marker('marker', 'used'))
        self.but_marker.grid(column=1, row=3)

        self.but_tape = Button(self.window, text="tape", font=(15), width=10, bg='blue',
            command=lambda:self.update_status_tape('tape', 'used'))
        self.but_tape.grid(column=1, row=4)
        
        self.but_notes = Button(self.window, text="sticky notes", font=(15), width=10, bg='blue', 
            command=lambda:self.update_status_notes('sticky notes', 'used'))
        self.but_notes.grid(column=1, row=5)
        

        self.but_thumbtack = Button(self.window, text="%i thumbtacks" % self.status['thumbtack'], bg='blue',
            font=(15), width=10, command=lambda:self.update_status_thumbtack('thumbtack'))
        self.but_thumbtack.grid(column=1, row=8)
        
        self.header = Label(self.window, text="Command queue", font=("Arial", 20))
        self.header.grid(column=0, row=9, columnspan=1)

        self.but_refresh = Button(self.window, text="refresh", font=(15), width=10, bg = 'cyan',
            command=lambda:self.refresh_cmd_queue())
        self.but_refresh.grid(column=1, row=9)

        self.header = Label(self.window, text="| id  | intent|  timestamp  |   status  | comment | item     |")
        self.header.grid(column=0, row=10, columnspan=2)

        # Label
        # self.lab = Label(self.window, text=self.format_msg(), justify='left') # , font=("Arial", 14), anchor='w')
        # self.lab.grid(column=0, row=13, columnspan=2)

        # text Entry
        # self.txt = Entry(self.window,width=10)
        # self.txt.grid(column=0, row=1)
        # self.txt.focus() # set focus so that we can directly enter

        self.window.mainloop()

def main():
    gui = GUI()
    gui.init_window()
    gui.run()

if __name__ == '__main__':
    main()
