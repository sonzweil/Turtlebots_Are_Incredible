#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from turtlebots_are_incredible.incredible_widget import IncredibleWidget

from rqt_gui_py.plugin import Plugin


class Incredible(Plugin):

    def __init__(self, context):
        super(Incredible, self).__init__(context)
        self.setObjectName('Turtlebots are incredible')
        self.widget = IncredibleWidget(context.node)
        serial_number = context.serial_number()
        if serial_number > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + ' ({0})'.format(serial_number))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print('Shutdown the Turtlebots are incredible.')
        self.widget.shutdown_widget()
