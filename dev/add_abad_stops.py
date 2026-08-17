"""Give the ABAD hinges physical joint stops matching the real +/-70 deg range.

WHY minStop/maxStop AND NOT minPosition/maxPosition.

    minPosition / maxPosition   soft limits enforced by Webots' POSITION
                                controller. corgi_driver drives the ABAD with
                                setTorque(), which bypasses that controller
                                entirely, so these are never consulted.
    minStop / maxStop           ODE joint stops on the HingeJointParameters.
                                Physical, and enforced whatever the motor is
                                doing.

Found the hard way on 2026-08-14: with only minPosition/maxPosition set, a
commanded 80 deg (already clamped to 70 in the driver) still let the measured
ABAD angles reach 192 / 184 / 295 / 285 deg once the robot tumbled and external
forces drove the joints. Nothing in the simulation stopped them.

The hip motors are deliberately left unlimited -- the leg-wheel rolls by
spinning beta continuously, 6.6 revolutions in a 20 s wheeled-mode run.

Idempotent: re-running will not double up the stops.

Run from the corgi_sim package root, then rebuild:
    python3 dev/add_abad_stops.py && colcon build --packages-select corgi_sim
"""
import math
import os
import re
import sys

PROTO = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "protos", "CorgiRobotABAD.proto")
LIMIT_DEG = 70.0


def main():
    rad = math.radians(LIMIT_DEG)
    with open(PROTO) as fh:
        lines = fh.readlines()

    targets = [i for i, l in enumerate(lines)
               if re.match(r'^\s*name "[ABCD]_ABAD"\s*$', l)]
    if len(targets) != 4:
        sys.exit(f"expected 4 ABAD motors, found {len(targets)}")

    inserts = []
    for i in targets:
        j = i
        while j > 0 and "jointParameters HingeJointParameters {" not in lines[j]:
            j -= 1
        if i - j >= 40:
            sys.exit(f"no HingeJointParameters within 40 lines above line {i}")
        block = "".join(lines[j:i])
        if "minStop" in block:
            print(f"  line {j+1}: stops already present, skipping")
            continue
        indent = re.match(r"^(\s*)", lines[j]).group(1) + "  "
        inserts.append((j + 1, indent))

    if not inserts:
        print("nothing to do")
        return

    for pos, indent in sorted(inserts, reverse=True):
        lines.insert(pos, indent + "minStop %.5f\n" % -rad)
        lines.insert(pos + 1, indent + "maxStop %.5f\n" % rad)

    with open(PROTO, "w") as fh:
        fh.writelines(lines)
    print("added minStop/maxStop +/-%.0f deg to %d ABAD hinges"
          % (LIMIT_DEG, len(inserts)))


if __name__ == "__main__":
    main()
