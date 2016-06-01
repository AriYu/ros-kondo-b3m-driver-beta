import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class B3mPlugin(Plugin):

    def __init__(self, context):
        super(B3mPlugin, self).__init__(context)
        self.setObjectName('B3mPlugin')

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_b3m_servo'), 'resource', 'B3mPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('B3mPluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' %d ' % context.serial_number()))
        context.add_widget(self._widget)
        self._widget.cancelButton.clicked[bool].connect(self._handle_cancel_clicked)
        self._widget.sendButton.clicked[bool].connect(self._handle_send_clicked)

        # Setting for Slider
        self._widget.targetPositionSlider.setMinimum(-320)
        self._widget.targetPositionSlider.setMaximum(320)
        self._widget.targetPositionSlider.setValue(0)
        self._widget.targetPositionSlider.valueChanged.connect(self._widget.lcdInputNumber.display)
        
    def shutdown_plugin(self):
        # unregister all publisher here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # save intrinsic configuration. usually using:
        # instance_settings.get_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # restore instrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def _handle_cancel_clicked(self):
        print "cancelButton is clicked"

    def _handle_send_clicked(self):
        print "sendButton is clicked"
