"""
New simulation environment: 4-room maze.

Map size : (1113, 750), centered at (0, 0)
  x range : -556  to  556
  y range : -375  to  375

Layout (schematic):
  +---------+  +----------+
  |   TL    |  |    TR    |
  |  (box)  |  |          |
  +----  ---+  +----  ----+
       gap(y)       gap(y)
  +---------+  +----------+
  |   BL    |  |    BR    |
  |         |  |  (box)   |
  +---------+  +----------+
       gap(x) at y=0

Two interior walls divide the map into 4 rooms:
  - Vertical wall at x=100, with a 300 px gap centred on y=0
  - Horizontal wall at y=0, with a 500 px gap centred on x=50

Connections (all passages ≥ 200 px wide):
  TL ↔ TR  :  cross x=100 at  y : 0   → 150  (in vertical gap)
  BL ↔ BR  :  cross x=100 at  y : -150 → 0   (in vertical gap)
  TL ↔ BL  :  cross y=0   at  x : -200 → 100  (in horizontal gap)
  TR ↔ BR  :  cross y=0   at  x : 100  → 300  (in horizontal gap)
"""

from place_bot.simulation.elements.normal_wall import NormalWall, NormalBox


def add_walls(playground):
    # ── Vertical wall at x=100 (left/right divider) ───────────────────────
    # Gap at y: -150 to 150  (300 px)
    wall = NormalWall(pos_start=(100.0, 375.0), pos_end=(100.0, 150.0))
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(100.0, -150.0), pos_end=(100.0, -375.0))
    playground.add(wall, wall.wall_coordinates)

    # ── Horizontal wall at y=0 (top/bottom divider) ───────────────────────
    # Gap at x: -200 to 300  (500 px)
    wall = NormalWall(pos_start=(-556.0, 0.0), pos_end=(-200.0, 0.0))
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(300.0, 0.0), pos_end=(556.0, 0.0))
    playground.add(wall, wall.wall_coordinates)


def add_boxes(playground):
    # Interior obstacle in TL room (x<100, y>0)
    box = NormalBox(up_left_point=(-380.0, 300.0), width=120, height=120)
    playground.add(box, box.wall_coordinates)

    # Interior obstacle in BR room (x>100, y<0)
    box = NormalBox(up_left_point=(220.0, -130.0), width=120, height=120)
    playground.add(box, box.wall_coordinates)
