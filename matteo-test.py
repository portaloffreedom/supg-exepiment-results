import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
print(sys.path)

import trollius
from trollius import From

from pygazebo.pygazebo import DisconnectError
from trollius.py33_exceptions import ConnectionResetError

from tol.config import parser
from tol.manage import World

from sdfbuilder import Pose
from sdfbuilder.math import Vector3

from revolve.convert.yaml import yaml_to_robot
from revolve.angle import Tree
from revolve.util import wait_for


# Some sample YAML to use later
spider9_yaml = '''
---
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg11Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg11
                  type        : FixedBrick
    2:
      id          : Leg20Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg20
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg21Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg21
                  type        : FixedBrick
    3:
      id          : Leg30Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg30
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg31Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg31
                  type        : FixedBrick


brain:
  type:
  # extra input neuron (bias)
  neurons:
    Core-hidden-0:
      id: Core-hidden-0
      layer: hidden
      part_id: Core
      type: Oscillator

  # Here you specify the connections between neurons, as
  # {"src": "src-id", "dst": "dst-id", "weight": float}
  connections:
  - src: Core-hidden-0
    dst: Leg00Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg01Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg10Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg11Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg20Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg21Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg30Joint-out-0
    weight: 1.0

  - src: Core-hidden-0
    dst: Leg31Joint-out-0
    weight: 1.0


  params:
    Core-hidden-0:
      period: 1.0
      phase_offset: 0
      amplitude: 1.8
'''

spider13_yaml = '''
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg02Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg02
                          type        : FixedBrick
                          orientation : -90
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg11Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg11
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg12Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg12
                          type        : FixedBrick
                          orientation : -90
    2:
      id          : Leg20Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg20
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg21Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg21
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg22Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg22
                          type        : FixedBrick
                          orientation : -90
    3:
      id          : Leg30Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg30
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg31Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg31
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg32Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg32
                          type        : FixedBrick
                          orientation : -90
'''

spider17_yaml = '''
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg02Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg02
                          type        : FixedBrick
                          orientation : -90
                          children    :
                           1:
                             id          : Leg03Joint
                             type        : ActiveHinge
                             orientation : 0
                             children    :
                               1:
                                 id          : Leg03
                                 type        : FixedBrick
                                 orientation : 0
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg11Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg11
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg12Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg12
                          type        : FixedBrick
                          orientation : -90
                          children    :
                           1:
                             id          : Leg13Joint
                             type        : ActiveHinge
                             orientation : 0
                             children    :
                               1:
                                 id          : Leg13
                                 type        : FixedBrick
                                 orientation : 0
    2:
      id          : Leg20Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg20
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg21Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg21
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg22Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg22
                          type        : FixedBrick
                          orientation : -90
                          children    :
                           1:
                             id          : Leg23Joint
                             type        : ActiveHinge
                             orientation : 0
                             children    :
                               1:
                                 id          : Leg23
                                 type        : FixedBrick
                                 orientation : 0
    3:
      id          : Leg30Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg30
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg31Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg31
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg32Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg32
                          type        : FixedBrick
                          orientation : -90
                          children    :
                           1:
                             id          : Leg33Joint
                             type        : ActiveHinge
                             orientation : 0
                             children    :
                               1:
                                 id          : Leg33
                                 type        : FixedBrick
                                 orientation : 0
'''

gecko7_yaml = '''
---
body:
  id          : Core
  type        : Core
  children    :
    2:
      slot        : 0
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg00
          type        : FixedBrick
          orientation : 0
    3:
      slot        : 0
      id          : Leg01Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg01
          type        : FixedBrick
          orientation : 0
    1:
      slot        : 0
      id          : BodyJoint0
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          slot        : 0
          id          : Body0
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              slot        : 0
              id          : BodyJoint1
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  slot        : 0
                  id          : Body1
                  type        : FixedBrick
                  orientation : -90
                  children:
                    2:
                      slot        : 0
                      id          : Leg10Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          slot        : 0
                          id          : Leg10
                          type        : FixedBrick
                          orientation : 0
                    3:
                      slot        : 0
                      id          : Leg11Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          slot        : 0
                          id          : Leg11
                          type        : FixedBrick
                          orientation : 0
'''

gecko12_yaml = '''
body:
  id          : Core
  type        : Core
  children    :
    2:
      slot        : 0
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg00
          type        : FixedBrick
          orientation : 0
          children    :
            1:
              id          : Leg001Joint
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  id          : Leg001
                  type        : FixedBrick
                  orientation : -90
    3:
      slot        : 0
      id          : Leg01Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg01
          type        : FixedBrick
          orientation : 0
          children    :
            1:
              id          : Leg011Joint
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  id          : Leg011
                  type        : FixedBrick
                  orientation : -90
    1:
      slot        : 0
      id          : BodyJoint0
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          slot        : 0
          id          : Body0
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              slot        : 0
              id          : BodyJoint1
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  slot        : 0
                  id          : Body1
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : BodyJoint2
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          slot        : 0
                          id          : Body2
                          type        : FixedBrick
                          orientation : -90
                          children:
                            2:
                              slot        : 0
                              id          : Leg10Joint
                              type        : ActiveHinge
                              orientation : 0
                              children    :
                                1:
                                  slot        : 0
                                  id          : Leg10
                                  type        : FixedBrick
                                  orientation : 0
                                  children    :
                                    1:
                                      id          : Leg101Joint
                                      type        : ActiveHinge
                                      orientation : 90
                                      children    :
                                        1:
                                          id          : Leg101
                                          type        : FixedBrick
                                          orientation : -90
                            3:
                              slot        : 0
                              id          : Leg11Joint
                              type        : ActiveHinge
                              orientation : 0
                              children    :
                                1:
                                  slot        : 0
                                  id          : Leg11
                                  type        : FixedBrick
                                  orientation : 0
                                  children    :
                                    1:
                                      id          : Leg111Joint
                                      type        : ActiveHinge
                                      orientation : 90
                                      children    :
                                        1:
                                          id          : Leg111
                                          type        : FixedBrick
                                          orientation : -90
'''

gecko17_yaml = '''
---
body:
  id          : Core
  type        : Core
  children    :
    2:
      slot        : 0
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg00
          type        : FixedBrick
          orientation : 0
          children    :
            1:
              id          : Leg001Joint
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  id          : Leg001
                  type        : FixedBrick
                  orientation : -90
                  children    :
                    1:
                      id          : Leg002Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          id          : Leg002
                          type        : FixedBrick
                          orientation : 0
    3:
      slot        : 0
      id          : Leg01Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg01
          type        : FixedBrick
          orientation : 0
          children    :
            1:
              id          : Leg011Joint
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  id          : Leg011
                  type        : FixedBrick
                  orientation : -90
                  children    :
                    1:
                      id          : Leg012Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          id          : Leg012
                          type        : FixedBrick
                          orientation : 0
    1:
      slot        : 0
      id          : BodyJoint0
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          slot        : 0
          id          : Body0
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              slot        : 0
              id          : BodyJoint1
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  slot        : 0
                  id          : Body1
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      slot        : 0
                      id          : BodyJoint2
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          slot        : 0
                          id          : Body2
                          type        : FixedBrick
                          orientation : -90
                          children    :
                            1:
                              id          : BodyJoint3
                              type        : ActiveHinge
                              orientation : 0
                              children    :
                                1:
                                  slot        : 0
                                  id          : Body3
                                  type        : FixedBrick
                                  orientation : 0
                                  children:
                                    2:
                                      slot        : 0
                                      id          : Leg10Joint
                                      type        : ActiveHinge
                                      orientation : 0
                                      children    :
                                        1:
                                          slot        : 0
                                          id          : Leg10
                                          type        : FixedBrick
                                          orientation : 0
                                          children    :
                                            1:
                                              id          : Leg101Joint
                                              type        : ActiveHinge
                                              orientation : 90
                                              children    :
                                                1:
                                                  id          : Leg101
                                                  type        : FixedBrick
                                                  orientation : -90
                                                  children    :
                                                    1:
                                                      id          : Leg102Joint
                                                      type        : ActiveHinge
                                                      orientation : 0
                                                      children    :
                                                        1:
                                                          id          : Leg102
                                                          type        : FixedBrick
                                                          orientation : 0
                                    3:
                                      slot        : 0
                                      id          : Leg11Joint
                                      type        : ActiveHinge
                                      orientation : 0
                                      children    :
                                        1:
                                          slot        : 0
                                          id          : Leg11
                                          type        : FixedBrick
                                          orientation : 0
                                          children    :
                                            1:
                                              id          : Leg111Joint
                                              type        : ActiveHinge
                                              orientation : 90
                                              children    :
                                                1:
                                                  id          : Leg111
                                                  type        : FixedBrick
                                                  orientation : -90
                                                  children    :
                                                    1:
                                                      id          : Leg112Joint
                                                      type        : ActiveHinge
                                                      orientation : 0
                                                      children    :
                                                        1:
                                                          id          : Leg112
                                                          type        : FixedBrick
                                                          orientation : 0
'''

snake5_yaml = '''
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
                  orientation : 0
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg11Joint
              type        : ActiveHinge
              children    :
                1:
                  id          : Leg11
                  type        : FixedBrick
                  orientation : 0
'''

snake7_yaml = '''
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg02Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg02
                          type        : FixedBrick
                          orientation : -90
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg11Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg11
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg12Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg12
                          type        : FixedBrick
                          orientation : -90
'''

snake9_yaml = '''
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg02Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg02
                          type        : FixedBrick
                          orientation : -90
                          children    :
                           1:
                             id          : Leg03Joint
                             type        : ActiveHinge
                             orientation : 0
                             children    :
                               1:
                                 id          : Leg03
                                 type        : FixedBrick
                                 orientation : 0
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg11Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg11
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg12Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg12
                          type        : FixedBrick
                          orientation : -90
                          children    :
                           1:
                             id          : Leg13Joint
                             type        : ActiveHinge
                             orientation : 0
                             children    :
                               1:
                                 id          : Leg13
                                 type        : FixedBrick
                                 orientation : 0
'''

baby1_yaml = '''
---
body:
  id          : Core
  type        : Core
  children    :
    2:
      slot        : 0
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg00
          type        : FixedBrick
          orientation : 0
    3:
      slot        : 0
      id          : Leg01Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          id          : Leg011Joint
          type        : ActiveHinge
          orientation : 90
          children    :
            1:
              slot        : 0
              id          : Leg01
              type        : FixedBrick
              orientation : -90
              children    :
                1:
                  id          : Leg021Joint
                  type        : ActiveHinge
                  orientation : 0
                  children    :
                    1:
                      slot        : 0
                      id          : Leg02
                      type        : FixedBrick
                      orientation : 0
    1:
      slot        : 0
      id          : BodyJoint0
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          slot        : 0
          id          : Body0
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              slot        : 0
              id          : BodyJoint1
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  slot        : 0
                  id          : Body1
                  type        : FixedBrick
                  orientation : -90
                  children:
                    2:
                      slot        : 0
                      id          : Leg10Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          slot        : 0
                          id          : Leg10
                          type        : FixedBrick
                          orientation : 0
                    3:
                      slot        : 0
                      id          : Leg11Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          slot        : 0
                          id          : Leg11
                          type        : FixedBrick
                          orientation : 0
'''
baby2_yaml = '''
body:
  id          : Core
  type        : Core
  children    :
    0:
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg00
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg01Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg01
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg02Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg02
                          type        : FixedBrick
                          orientation : -90
    1:
      id          : Leg10Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg10
          type        : FixedBrick
          orientation : -90
    2:
      id          : Leg20Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg20
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg21Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg21
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg22Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg22
                          type        : FixedBrick
                          orientation : -90
    3:
      id          : Leg30Joint
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          id          : Leg30
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              id          : Leg31Joint
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  id          : Leg31
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      id          : Leg32Joint
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          id          : Leg32
                          type        : FixedBrick
                          orientation : -90
'''
baby3_yaml = '''
---
body:
  id          : Core
  type        : Core
  children    :
    2:
      slot        : 0
      id          : Leg00Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg00
          type        : FixedBrick
          orientation : 0
          children    :
            1:
              id          : Leg001Joint
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  id          : Leg001
                  type        : FixedBrick
                  orientation : -90
                  children    :
                    1:
                      id          : Leg002Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          id          : Leg002
                          type        : FixedBrick
                          orientation : 0
    3:
      slot        : 0
      id          : Leg01Joint
      type        : ActiveHinge
      orientation : 0
      children    :
        1:
          slot        : 0
          id          : Leg01
          type        : FixedBrick
          orientation : 0
          children    :
            1:
              id          : Leg011Joint
              type        : ActiveHinge
              orientation : 90
              children    :
                1:
                  id          : Leg011
                  type        : FixedBrick
                  orientation : -90
                  children    :
                    1:
                      id          : Leg012Joint
                      type        : ActiveHinge
                      orientation : 0
                      children    :
                        1:
                          id          : Leg012
                          type        : FixedBrick
                          orientation : 0
    1:
      slot        : 0
      id          : BodyJoint0
      type        : ActiveHinge
      orientation : 90
      children    :
        1:
          slot        : 0
          id          : Body0
          type        : FixedBrick
          orientation : -90
          children    :
            1:
              slot        : 0
              id          : BodyJoint1
              type        : ActiveHinge
              orientation : 0
              children    :
                1:
                  slot        : 0
                  id          : Body1
                  type        : FixedBrick
                  orientation : 0
                  children    :
                    1:
                      slot        : 0
                      id          : BodyJoint2
                      type        : ActiveHinge
                      orientation : 90
                      children    :
                        1:
                          slot        : 0
                          id          : Body2
                          type        : FixedBrick
                          orientation : -90
                          children    :
                            1:
                              id          : BodyJoint3
                              type        : ActiveHinge
                              orientation : 0
                              children    :
                                1:
                                  slot        : 0
                                  id          : Body3
                                  type        : FixedBrick
                                  orientation : 0
                                  children:
                                    2:
                                      slot        : 0
                                      id          : Leg10Joint
                                      type        : ActiveHinge
                                      orientation : 0
                                      children    :
                                        1:
                                          slot        : 0
                                          id          : Leg10
                                          type        : FixedBrick
                                          orientation : 0
                                          children    :
                                            1:
                                              id          : Leg101Joint
                                              type        : ActiveHinge
                                              orientation : 90
                                              children    :
                                                1:
                                                  id          : Leg101
                                                  type        : FixedBrick
                                                  orientation : -90
                                                  children    :
                                                    1:
                                                      id          : Leg102Joint
                                                      type        : ActiveHinge
                                                      orientation : 0
                                                      children    :
                                                        1:
                                                          id          : Leg102
                                                          type        : FixedBrick
                                                          orientation : 0
                                    3:
                                      slot        : 0
                                      id          : Leg11Joint
                                      type        : ActiveHinge
                                      orientation : 0
                                      children    :
                                        1:
                                          slot        : 0
                                          id          : Leg11
                                          type        : FixedBrick
                                          orientation : 0
                                          children    :
                                            1:
                                              id          : Leg111Joint
                                              type        : ActiveHinge
                                              orientation : 90
                                              children    :
                                                1:
                                                  id          : Leg112Joint
                                                  type        : ActiveHinge
                                                  orientation : -90
                                                  children    :
                                                    1:
                                                      id          : Leg112
                                                      type        : FixedBrick
                                                      orientation : 0
'''

#bot_yaml = spider9_yaml
#bot_yaml = spider13_yaml
#bot_yaml = spider17_yaml
#bot_yaml = gecko7_yaml
#bot_yaml = gecko12_yaml
#bot_yaml = gecko17_yaml
#bot_yaml = snake5_yaml
#bot_yaml = snake7_yaml
bot_yaml = snake9_yaml
#bot_yaml = baby1_yaml
#bot_yaml = baby2_yaml
#bot_yaml = baby3_yaml


@trollius.coroutine
def run():
    """
    The main coroutine, which is started below.
    """
    # Parse command line / file input arguments
    conf = parser.parse_args()

    # This disables the analyzer; enable it if you want to generate valid robots
    # Can also do this using arguments of course, just pass an empty string
    # conf.analyzer_address = None

    conf.output_directory = "output"

    # Create the world, this connects to the Gazebo world
    world = yield From(World.create(conf))

    # These are useful when working with YAML
    body_spec = world.builder.body_builder.spec
    brain_spec = world.builder.brain_builder.spec

    # Create a robot from YAML
    robot = yaml_to_robot(body_spec, brain_spec, bot_yaml)

    # Create a revolve.angle `Tree` representation from the robot, which
    # is what is used in the world manager.
    robot_tree = Tree.from_body_brain(robot.body, robot.brain, body_spec)

    # Insert the robot into the world. `insert_robot` resolves when the insert
    # request is sent, the future it returns resolves when the robot insert
    # is actually confirmed and a robot manager object has been created
    pose = Pose(position=Vector3(0, 0, 0.05))
    future = yield From(world.insert_robot(robot_tree, pose))
    robot_manager = yield From(future)

    # I usually start the world paused, unpause it here. Note that
    # pause again returns a future for when the request is sent,
    # that future in turn resolves when a response has been received.
    # This is the general convention for all message actions in the
    # world manager. `wait_for` saves the hassle of grabbing the
    # intermediary future in this case.
    yield From(wait_for(world.pause(False)))

    # Start a run loop to do some stuff
    while True:
        # Print robot fitness every second
        print("Robot fitness: %f" % robot_manager.fitness())
        yield From(trollius.sleep(1.0))


def main():
    def handler(loop, context):
        exc = context['exception']
        if isinstance(exc, DisconnectError) or isinstance(exc, ConnectionResetError):
            print("Got disconnect / connection reset - shutting down.")
            sys.exit(0)
            raise context['exception']
    try:
        loop = trollius.get_event_loop()
        loop.set_exception_handler(handler)
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")


if __name__ == '__main__':
    main()
