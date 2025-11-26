import math

import pcbnew

pcb = pcbnew.LoadBoard("pebble.kicad_pcb")

for i in range(1, 73):
    c = pcb.FindFootprintByReference("D{0}".format(i))
    t = i * 1 / 72 * math.pi * 2
    c.SetPosition(pcbnew.VECTOR2I_MM(150 + math.cos(t) * 60, 100 + math.sin(t) * 60))
    c.SetOrientationDegrees(-t / (math.pi * 2) * 360)
pcbnew.Refresh()
