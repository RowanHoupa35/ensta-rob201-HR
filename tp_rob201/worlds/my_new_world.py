import math
import random
from typing import Type, Union

from place_bot.simulation.robot.robot_abstract import RobotAbstract
from place_bot.simulation.gui_map.closed_playground import ClosedPlayground
from place_bot.simulation.gui_map.world_abstract import WorldAbstract

from worlds import walls_new_world


class MyNewWorld(WorldAbstract):

    def __init__(self, robot: RobotAbstract, use_shaders: bool = True):
        super().__init__(robot=robot)

        # PARAMETERS MAP
        self._size_area = (1113, 750)

        # PLAYGROUND
        self._playground = ClosedPlayground(size=self._size_area, use_shaders=use_shaders)
        walls_new_world.add_walls(self._playground)
        walls_new_world.add_boxes(self._playground)

        # POSITION OF THE ROBOT: start in the top-left room (TL)
        angle = 0  # random.uniform(-math.pi, math.pi)
        self._robot_pos = ((-300.0, 200.0), angle)   # x<100, y>0 → TL room
        self._playground.add(robot, self._robot_pos)
