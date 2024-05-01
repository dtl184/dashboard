import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import Float64, Float64MultiArray, String, Int16
from sensor_msgs.msg import NavSatFix

os.environ["QT_API"] = "pyqt5"

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also, if you open multiple instances of your
        # plugin at once, these lines add a number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Data Subscribers, one for each value we wish to output
        self._speed_subscriber = rospy.Subscriber('car/state/vel_x', Float64, self.callback_speed)
        self._steering_subscriber = rospy.Subscriber('car/state/steering', Float64MultiArray, self.callback_steering)
        self._accstate_subscriber = rospy.Subscriber('acc/cruise_state', String, self.callback_accstate)
        self._accsetdist_subscriber = rospy.Subscriber('acc/set_distance', Int16, self.callback_accsetdist)
        self._gps_subscriber = rospy.Subscriber('gps_fix', NavSatFix, self.callback_gps)
        self._minicar_subscriber = rospy.Subscriber('/acc/mini_car', Int16, self.callback_minicar)

    def callback_speed(self, subscribed_data):
        speed = subscribed_data.data
        self._widget.Velocity_Display_Number.display(speed)
        
    def callback_steering(self, subscribed_data):
        steering = subscribed_data.data[0]
        self._widget.Steering_Display_Number.display(steering)

    def callback_accstate(self, subscribed_data):
        accstate = subscribed_data.data
        self._widget.ACCState_OutputLabel.setText(accstate)

    def callback_accsetdist(self, subscribed_data):
        accsetdist = subscribed_data.data
        self._widget.ACC_SetDist_Display_Number.display(accsetdist)

    def callback_gps(self, subscribed_data):
        latitude = subscribed_data.latitude
        longitude = subscribed_data.longitude
        self._widget.Latitude_Display_Number.display(latitude)
        self._widget.Longitude_Display_Number.display(longitude)

    def callback_minicar(self, subscribed_data):
        minicar = subscribed_data.data
        tf = ""
        if minicar == 0:
        	tf = "False"
        else:
        	tf = "True"
        self._widget.Minicar_OutputLabel.setText(tf)

    def shutdown_plugin(self):
        # Shutdown the subscriber when the plugin is closed
        if self._speed_subscriber:
            self._speed_subscriber.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # Save intrinsic configuration
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore intrinsic configuration
        pass

