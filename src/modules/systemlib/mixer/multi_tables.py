#!/usr/bin/env python
############################################################################
#
#   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# Generate multirotor mixer scale tables compatible with the ArduCopter layout
#

# for python2.7 compatibility
from __future__ import  print_function

import math

print("/*")
print("* This file is automatically generated by multi_tables - do not edit.")
print("*/")
print("")
print("#ifndef _MIXER_MULTI_TABLES")
print("#define _MIXER_MULTI_TABLES")
print("")

def rcos(angleInRadians):
  return math.cos(math.radians(angleInRadians))

CCW = 1.0
CW = -CCW

quad_x = [
    [  45, CCW],
    [-135, CCW],
    [-45,  CW],
    [135,  CW],
]

quad_plus = [
    [  90, CCW],
    [ -90, CCW],
    [   0, CW],
    [ 180, CW],
]

quad_deadcat = [
    [  63, CCW, 1.0],
    [-135, CCW, 0.964],
    [ -63, CW, 1.0],
    [ 135, CW, 0.964],
]

quad_v = [
    [  18.8, 0.4242],
    [ -18.8, 1.0],
    [ -18.8, -0.4242],
    [  18.8, -1.0],
]

quad_wide = [
    [   68, CCW],
    [ -129, CCW],
    [  -68, CW],
    [  129, CW],
]

quad_iris = [
    [   61.23, CCW],
    [ -124.25, CCW],
    [  -61.23, CW],
    [  124.25, CW],
]

hex_x = [
    [  90, CW],
    [ -90, CCW],
    [ -30, CW],
    [ 150, CCW],
    [  30, CCW],
    [-150, CW],
]

hex_plus = [
    [   0, CW],
    [ 180, CCW],
    [-120, CW],
    [  60, CCW],
    [ -60, CCW],
    [ 120, CW],
]

hex_cox = [
    [  60, CW],
    [  60, CCW],
    [ 180, CW],
    [ 180, CCW],
    [ -60, CW],
    [ -60, CCW],
]

octa_x = [
    [  22.5, CW],
    [-157.5, CW],
    [  67.5, CCW],
    [ 157.5, CCW],
    [ -22.5, CCW],
    [-112.5, CCW],
    [ -67.5, CW],
    [ 112.5, CW],
]

octa_plus = [
    [   0, CW],
    [ 180, CW],
    [  45, CCW],
    [ 135, CCW],
    [ -45, CCW],
    [-135, CCW],
    [ -90, CW],
    [  90, CW],
]

octa_cox = [
    [  45, CCW],
    [ -45, CW],
    [-135, CCW],
    [ 135, CW],
    [ -45, CCW],
    [  45, CW],
    [ 135, CCW],
    [-135, CW],
]

twin_engine = [
    [ 90, 0.0],
    [-90, 0.0],
]


tables = [quad_x, quad_plus, quad_v, quad_wide, quad_iris, quad_deadcat, hex_x, hex_plus, hex_cox, octa_x, octa_plus, octa_cox, twin_engine]

def variableName(variable):
    for variableName, value in list(globals().items()):
        if value is variable:
            return variableName

def unpackScales(scalesList):
    if len(scalesList) == 2:
        scalesList += [1.0] #Add thrust scale
    return scalesList

def printEnum():
    print("enum class MultirotorGeometry : MultirotorGeometryUnderlyingType {")
    for table in tables:
        print("\t{},".format(variableName(table).upper()))

    print("\n\tMAX_GEOMETRY")
    print("}; // enum class MultirotorGeometry\n")

def printScaleTables():
    for table in tables:
        print("const MultirotorMixer::Rotor _config_{}[] = {{".format(variableName(table)))
        for row in table:
            angle, yawScale, thrustScale = unpackScales(row)
            rollScale = rcos(angle + 90)
            pitchScale = rcos(angle)
            print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(rollScale, pitchScale, yawScale, thrustScale))
        print("};\n")

def printScaleTablesIndex():
    print("const MultirotorMixer::Rotor *_config_index[] = {")
    for table in tables:
        print("\t&_config_{}[0],".format(variableName(table)))
    print("};\n")


def printScaleTablesCounts():
    print("const unsigned _config_rotor_count[] = {")
    for table in tables:
        print("\t{}, /* {} */".format(len(table), variableName(table)))
    print("};\n")



printEnum()

print("namespace {")
printScaleTables()
printScaleTablesIndex()
printScaleTablesCounts()

print("} // anonymous namespace\n")
print("#endif /* _MIXER_MULTI_TABLES */")
print("")
