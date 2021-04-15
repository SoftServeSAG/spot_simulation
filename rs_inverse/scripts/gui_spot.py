#!/usr/bin/env python

""" GUI for Spot` controller"""

import rospy
from rs_msgs.msg import GaitInput
from Tkinter import Tk, Label, Button, Entry, END


class RsGui:
    def __init__(self, win, pub):
        self.label_x = Label(win, text='x')
        self.label_y = Label(win, text='y')
        self.Label_z = Label(win, text='z')
        self.label_roll = Label(win, text='roll')
        self.label_pitch = Label(win, text='pitch')
        self.label_yaw = Label(win, text='yaw')
        self.label_steplength = Label(win, text='StepLength')
        self.label_latfrac = Label(win, text='LateralFraction')
        self.label_yawrate = Label(win, text='YawRate')
        self.label_stepvel = Label(win, text='StepVelocity')
        self.label_clheight = Label(win, text='ClearanceHeight')
        self.label_pentdepth = Label(win, text='PenetrationDepth')
        self.label_swing = Label(win, text='SwingPeriod')
        self.label_yawcontrol = Label(win, text='YawControl')
        self.label_yawcontrolon = Label(win, text='YawControlOn')

        self.entry_x = Entry()
        self.entry_y = Entry()
        self.entry_z = Entry()
        self.entry_roll = Entry()
        self.entry_pitch = Entry()
        self.entry_yaw = Entry()
        self.entry_steplength = Entry()
        self.entry_latfrac = Entry()
        self.entry_yawrate = Entry()
        self.entry_stepvel = Entry()
        self.entry_clheight = Entry()
        self.entry_pentdepth = Entry()
        self.entry_swing = Entry()
        self.entry_yawcontrol = Entry()
        self.entry_yawcontrolon = Entry()

        self.entry_x.insert(END, 0)
        self.entry_y.insert(END, 0)
        self.entry_z.insert(END, 0.1)
        self.entry_roll.insert(END, 0)
        self.entry_pitch.insert(END, 0)
        self.entry_yaw.insert(END, 0)
        self.entry_steplength.insert(END, 0.1)
        self.entry_latfrac.insert(END, 0)
        self.entry_yawrate.insert(END, 0)
        self.entry_stepvel.insert(END, 0.8)
        self.entry_clheight.insert(END, 0.15)
        self.entry_pentdepth.insert(END, 0.00003)
        self.entry_swing.insert(END, 0.3)
        self.entry_yawcontrol.insert(END, 0)
        self.entry_yawcontrolon.insert(END, 1)

        self.label_x.place(x=100, y=50)
        self.label_y.place(x=100, y=100)
        self.Label_z.place(x=100, y=150)
        self.label_roll.place(x=100, y=200)
        self.label_pitch.place(x=100, y=250)
        self.label_yaw.place(x=100, y=300)
        self.label_steplength.place(x=100, y=350)
        self.label_latfrac.place(x=100, y=400)
        self.label_yawrate.place(x=100, y=450)
        self.label_stepvel.place(x=100, y=500)
        self.label_clheight.place(x=100, y=550)
        self.label_pentdepth.place(x=100, y=600)
        self.label_swing.place(x=100, y=650)
        self.label_yawcontrol.place(x=100, y=700)
        self.label_yawcontrolon.place(x=100, y=750)


        self.entry_x.place(x=220, y=50)
        self.entry_y.place(x=220, y=100)
        self.entry_z.place(x=220, y=150)
        self.entry_roll.place(x=220, y=200)
        self.entry_pitch.place(x=220, y=250)
        self.entry_yaw.place(x=220, y=300)
        self.entry_steplength.place(x=220, y=350)
        self.entry_latfrac.place(x=220, y=400)
        self.entry_yawrate.place(x=220, y=450)
        self.entry_stepvel.place(x=220, y=500)
        self.entry_clheight.place(x=220, y=550)
        self.entry_pentdepth.place(x=220, y=600)
        self.entry_swing.place(x=220, y=650)
        self.entry_yawcontrol.place(x=220, y=700)
        self.entry_yawcontrolon.place(x=220, y=750)

        self.b1 = Button(win, text='Send', command=self.command_pub)
        self.b1.place(x=100, y=800)
        self.pub = pub

    def command_pub(self):
        """ Publish commands on controller"""
        msg = GaitInput()
        msg.x = float(self.entry_x.get())
        msg.y = float(self.entry_y.get())
        msg.z = float(self.entry_z.get())
        msg.roll = float(self.entry_roll.get())
        msg.pitch = float(self.entry_pitch.get())
        msg.yaw = float(self.entry_yaw.get())
        msg.StepLength = float(self.entry_steplength.get())
        msg.LateralFraction = float(self.entry_latfrac.get())
        msg.YawRate = float(self.entry_yawrate.get())
        msg.StepVelocity = float(self.entry_stepvel.get())
        msg.ClearanceHeight = float(self.entry_clheight.get())
        msg.PenetrationDepth = float(self.entry_pentdepth.get())
        msg.SwingPeriod = float(self.entry_swing.get())
        msg.YawControl = float(self.entry_yawcontrol.get())
        msg.YawControlOn = float(self.entry_yawcontrolon.get())
        self.pub.publish(msg)


def main():
    spot_name = rospy.get_param('~/spot_name')
    rospy.init_node(spot_name + '_inverse_gui')
    pub = rospy.Publisher('/' + spot_name + '/inverse_gait_input', GaitInput, queue_size=10)

    root = Tk()
    my_gui = RsGui(root, pub)
    root.title(spot_name + 'Control Input')
    root.geometry("400x900+10+10")
    root.mainloop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
