import argparse
import math


PINS = [681, 270, 100, 12 * 2]
LOWER_PART = 47


def print_conditional(boundaries, indent="", pos=0):
    indent = indent + "    "
    if not boundaries:
        print("{}return {};".format(indent, pos))
        return
    L = len(boundaries)
    b = boundaries[math.floor(L / 2)]
    if(b < round(b)):
        c = "<"
    else:
        c = "<="
    print("{}if(v {} {}) {}".format(indent, c, round(b), "{"))
    print_conditional(boundaries[0:math.floor(L/2)], indent, pos << 1)
    print("{}{} else {}".format(indent, "}", "{"))
    print_conditional(boundaries[math.floor(L/2)+1:], indent, (pos << 1) + 1)
    print("{}{}".format(indent, "}"))


def print_code(boundaries):
    print("uint8_t interpret_dip_adc(int16_t v) {")
    print_conditional(boundaries)
    print("}")


def main():
    parser = argparse.ArgumentParser("dip switch calculator")
    parser.add_argument("-d", dest="debug", action="store_true")
    parser.add_argument("-H", dest="hex", action="store_true")
    parser.add_argument("-v", dest="voltage", type=int, default=1024)
    args = parser.parse_args()

    values = []
    for i in range(len(PINS) ** 2):
        L = 0
        for j, r in enumerate(PINS):
            if (1 << j) & i:
                L += 1./r
        if L == 0:
            values.append(0)
        else:
            R = 1 / L
            values.append(args.voltage * LOWER_PART / (R + LOWER_PART))
    boundaries = []
    for i in range(len(values) - 1):
        boundaries.append((values[i] + values[i + 1]) / 2)

    # boundaries = [round(i) for i in boundaries]
    if not args.debug:
        print_code(boundaries)
    else:
        for i, v in enumerate(values):
            if args.hex:
                print(i, hex(round(v/4)))
            else:
                print(i, v)


if __name__ == "__main__":
    main()
