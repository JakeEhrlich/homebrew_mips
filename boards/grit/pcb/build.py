#!/usr/bin/env python3
"""Build the grit PCB project for flatland (`pcb`) from the board file.

    python3 boards/grit/pcb/build.py            # library + netlist + placement
    python3 boards/grit/pcb/build.py --route    # ... then autoroute, check, outputs

Reads ../netlist.json (the board file exported by `mips32 export grit`),
writes the component library in lib/ (footprints for the packages the
flatland libraries do not have yet), and drives `pcb` to create pcb.json:
every chip becomes an instance, every net a net, and the floorplan below
places it all.  Re-running starts from scratch (pcb.json is regenerated).
"""
import json, os, subprocess, sys, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD = json.load(open(os.path.join(HERE, "..", "netlist.json")))
PCB = os.environ.get("PCB", os.path.expanduser("~/flatland/target/release/pcb"))
JLC = os.path.expanduser("~/flatland/library-jlcpcb")
LIB = os.path.join(HERE, "lib")
ROUTE = "--route" in sys.argv
FOUR = "--4layer" in sys.argv
LAYERS = "F.Cu,In1.Cu,In2.Cu,B.Cu" if FOUR else "F.Cu,B.Cu"


def run(*args, check=True, quiet=False):
    cmd = [PCB, "-p", os.path.join(HERE, "pcb.json")] + [str(a) for a in args]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 and check:
        print("FAILED:", " ".join(cmd))
        print(r.stdout)
        print(r.stderr)
        sys.exit(1)
    if not quiet and r.stdout.strip():
        print(r.stdout.strip())
    return r


# ---------------------------------------------------------------- footprints
def th(name, x, y, drill=1.0, size=1.7, square=False):
    return {"name": name, "type": "through_hole", "shape": "rect" if square else "oval", "at": [round(x, 3), round(y, 3)], "size": [size, size], "drill": drill}


def smd(name, x, y, w, h, r=0.15):
    return {"name": name, "type": "smd", "shape": "round_rect", "at": [round(x, 3), round(y, 3)], "size": [w, h], "corner_radius": r}


def court(w, h):
    return [{"polyline": {"points": [[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]]}}]


def fp(name, desc, pads, silk, cw, ch, label_y):
    return {"schema": "pcb-footprint/1", "name": name, "description": desc, "pads": pads, "silkscreen": silk, "courtyard": court(cw, ch), "label_at": [0, label_y]}


def dip(n, row_mm, name, desc):
    """DIP with the long axis vertical, pin 1 top-left, counter-clockwise."""
    half = n // 2
    x = row_mm / 2
    pads = []
    for i in range(half):
        y = (half - 1) / 2 * 2.54 - i * 2.54
        pads.append(th(str(i + 1), -x, y, square=(i == 0)))
        pads.append(th(str(n - i), x, y))
    top = (half - 1) / 2 * 2.54 + 1.27
    silk = [{"line": {"from": [-1.5, top + 0.6], "to": [1.5, top + 0.6], "width": 0.15}},
            {"arc": {"from": [-1.5, top + 0.6], "to": [1.5, top + 0.6], "center": [0, top + 0.6], "clockwise": False}}]
    silk = [{"circle": {"center": [-x - 1.6, top - 0.5], "diameter": 0.6, "width": 0.15}}]
    return fp(name, desc, pads, silk, row_mm + 2.6, top * 2 + 1.2, top + 1.6)


def quad(name, desc, n_side, pitch, center, pad_l, pad_w, body):
    """Quad flat pack, pin 1 top-left, counter-clockwise: left side top to bottom, bottom left to right, right bottom to top, top right to left."""
    pads = []
    k = 1
    span = (n_side - 1) / 2 * pitch
    for i in range(n_side):  # left, top to bottom
        pads.append(smd(str(k), -center, span - i * pitch, pad_l, pad_w)); k += 1
    for i in range(n_side):  # bottom, left to right
        pads.append(smd(str(k), -span + i * pitch, -center, pad_w, pad_l)); k += 1
    for i in range(n_side):  # right, bottom to top
        pads.append(smd(str(k), center, -span + i * pitch, pad_l, pad_w)); k += 1
    for i in range(n_side):  # top, right to left
        pads.append(smd(str(k), span - i * pitch, center, pad_w, pad_l)); k += 1
    silk = [{"circle": {"center": [-center - 0.4, center + 0.4], "diameter": 0.5, "width": 0.15}}]
    c = center + pad_l / 2 + 0.5
    return fp(name, desc, pads, silk, 2 * c, 2 * c, c + 0.8)


def dual(name, desc, n, pitch, center, pad_l, pad_w, body_w):
    """Dual-row SMD (SOIC, TSSOP, SOT-23-x, SC-88): pin 1 top-left, down the left, up the right."""
    half = n // 2
    span = (half - 1) / 2 * pitch
    pads = []
    for i in range(half):
        pads.append(smd(str(i + 1), -center, span - i * pitch, pad_l, pad_w))
    for i in range(half):
        pads.append(smd(str(half + i + 1), center, -span + i * pitch, pad_l, pad_w))
    silk = [{"circle": {"center": [-center - pad_l / 2 - 0.5, span], "diameter": 0.5, "width": 0.15}}]
    return fp(name, desc, pads, silk, 2 * center + pad_l + 1.0, span * 2 + pad_w + 1.0, span + pad_w / 2 + 1.1)


def sot23_5():
    pads = [smd("1", -1.1, 0.95, 1.06, 0.65), smd("2", -1.1, 0, 1.06, 0.65), smd("3", -1.1, -0.95, 1.06, 0.65),
            smd("4", 1.1, -0.95, 1.06, 0.65), smd("5", 1.1, 0.95, 1.06, 0.65)]
    return fp("SOT-23-5", "SOT-23-5, 0.95 mm pitch: pins 1-3 down the left, 4 bottom-right, 5 top-right", pads,
              [{"circle": {"center": [-2.1, 0.95], "diameter": 0.5, "width": 0.15}}], 3.8, 3.4, 2.2)


def sc88():
    pads = [smd("1", -1.05, 0.65, 1.0, 0.42), smd("2", -1.05, 0, 1.0, 0.42), smd("3", -1.05, -0.65, 1.0, 0.42),
            smd("4", 1.05, -0.65, 1.0, 0.42), smd("5", 1.05, 0, 1.0, 0.42), smd("6", 1.05, 0.65, 1.0, 0.42)]
    return fp("SC-88_SOT-363", "SC-88 / SOT-363 six-lead, 0.65 mm pitch: pins 1-3 down the left, 4-6 up the right", pads,
              [{"circle": {"center": [-2.0, 0.65], "diameter": 0.4, "width": 0.15}}], 3.6, 2.8, 1.9)


def sot143():
    pads = [smd("1", -0.95, -1.0, 1.2, 1.0), smd("2", 0.95, -1.0, 0.8, 1.0), smd("3", 0.95, 1.0, 0.8, 1.0), smd("4", -0.95, 1.0, 0.8, 1.0)]
    return fp("SOT-143", "SOT-143 four-lead: pin 1 (the wide lead) bottom-left, 2 bottom-right, 3 top-right, 4 top-left. VERIFY against the MAX811 datasheet before ordering", pads,
              [{"circle": {"center": [-2.0, -1.0], "diameter": 0.4, "width": 0.15}}], 4.0, 3.6, 2.3)


def xtal3225():
    pads = [smd("1", -1.1, -0.85, 1.4, 1.2), smd("2", 1.1, -0.85, 1.4, 1.2), smd("3", 1.1, 0.85, 1.4, 1.2), smd("4", -1.1, 0.85, 1.4, 1.2)]
    return fp("Crystal_3225_4Pin", "3.2 x 2.5 mm four-pad crystal: pads 1 and 3 are the crystal, 2 and 4 the case (ground). Pin 1 bottom-left", pads,
              [{"circle": {"center": [-2.2, -0.85], "diameter": 0.4, "width": 0.15}}], 4.4, 3.6, 2.3)


def osc7050():
    pads = [smd("1", -2.5, -2.0, 2.2, 1.4), smd("2", 2.5, -2.0, 2.2, 1.4), smd("3", 2.5, 2.0, 2.2, 1.4), smd("4", -2.5, 2.0, 2.2, 1.4)]
    return fp("Oscillator_7050_4Pin", "7.0 x 5.0 mm four-pad oscillator: 1 enable (bottom-left), 2 GND, 3 output, 4 VCC. VERIFY the pad drawing of the part chosen", pads,
              [{"circle": {"center": [-4.0, -2.0], "diameter": 0.5, "width": 0.15}}], 8.4, 6.2, 3.7)


def tact51():
    pads = [smd("1", -3.25, 1.85, 1.4, 1.0), smd("2", 3.25, 1.85, 1.4, 1.0), smd("3", 3.25, -1.85, 1.4, 1.0), smd("4", -3.25, -1.85, 1.4, 1.0)]
    return fp("SW_SMD_5.1x5.1_P3.7_LS6.5", "5.1 x 5.1 mm SMD tactile switch, pads at +-3.25 x, +-1.85 y (LCSC SW-SMD_4P-L5.1-W5.1-P3.70-LS6.5). Pads 1 and 3 are diagonal, so wiring across them is right whichever pair the part shorts internally. VERIFY pad sizes against the datasheet", pads,
              [], 7.6, 5.6, 3.4)


def slide():
    pads = [smd("1", -2.0, -1.2, 0.9, 1.6), smd("2", 0, -1.2, 0.9, 1.6), smd("3", 2.0, -1.2, 0.9, 1.6),
            smd("T1", -4.0, 0.9, 1.2, 1.6), smd("T2", 4.0, 0.9, 1.2, 1.6)]
    return fp("SW_Slide_MSK12C02", "MSK12C02 SPDT slide switch: three 2.0 mm pitch pads (2 is the common) and two mounting tabs. VERIFY against the datasheet before ordering", pads,
              [], 10.0, 4.6, 2.9)


def dipsw4():
    pads = []
    for i in range(4):
        x = -3.81 + i * 2.54
        pads.append(smd(str(i + 1), x, -3.9, 1.0, 1.8))
        pads.append(smd(str(8 - i), x, 3.9, 1.0, 1.8))
    return fp("SW_DIP_4_SMD_EM-04-Q", "Diptronics EM-04-Q 4-position SMD DIP switch, 2.54 mm pitch, pads at +-3.9 y: position k joins pad k (bottom) and pad 9-k (top). VERIFY against the datasheet before ordering", pads,
              [{"circle": {"center": [-5.3, -3.9], "diameter": 0.5, "width": 0.15}}], 11.0, 10.0, 5.6)


def header(rows, cols, name):
    pads = []
    k = 1
    for c in range(cols):
        for r in range(rows):
            pads.append(th(str(k), r * 2.54, -c * 2.54, square=(k == 1)))
            k += 1
    desc = f"{rows}x{cols} pin header, 2.54 mm; pin 1 square at the origin, odd pins in the first row" if rows == 2 else f"1x{cols} pin header, 2.54 mm; pin 1 square at the origin"
    f = fp(name, desc, pads, [], rows * 2.54 + 0.2, cols * 2.54 + 0.2, 1.9)
    # courtyard around the actual pin field, not centred on the origin
    w, h = rows * 2.54, cols * 2.54
    f["courtyard"] = [{"polyline": {"points": [[-1.27, 1.27], [w - 1.27, 1.27], [w - 1.27, 1.27 - h], [-1.27, 1.27 - h]]}}]
    f["label_at"] = [(rows - 1) * 1.27, 2.4]
    return f


def db9f():
    """DB9 female, right angle, face toward -y.  Pin 1 at +x (a female is the mirror of a male seen from the board).  VERIFY."""
    pads = []
    for i in range(5):
        pads.append(th(str(i + 1), 5.54 - i * 2.77, 0, drill=1.0, size=1.6, square=(i == 0)))
    for i in range(4):
        pads.append(th(str(6 + i), 4.155 - i * 2.77, -2.84, drill=1.0, size=1.6))
    pads.append({"name": "M1", "type": "through_hole", "shape": "circle", "at": [-12.5, -1.42], "size": [3.1, 3.1], "drill": 3.1, "plated": False})
    pads.append({"name": "M2", "type": "through_hole", "shape": "circle", "at": [12.5, -1.42], "size": [3.1, 3.1], "drill": 3.1, "plated": False})
    return fp("DSUB-9_Female_Horizontal", "DE-9 female, right angle, 2.77 mm pitch, rows 2.84 mm apart, two 3.1 mm mounting holes at +-12.5 mm; the face is toward -y and overhangs the board edge by about 6 mm. Pin 1 is at +x. VERIFY the numbering against the connector before ordering", pads,
              [{"line": {"from": [-15.5, -8.5], "to": [15.5, -8.5], "width": 0.2}}], 31.5, 12.0, 4.5)


def comp(name, desc, footprint, pins, lcsc=None, mpn=None, manufacturer=None, assembly=True, params=None):
    c = {"schema": "pcb-component/1", "name": name, "description": desc, "footprint": {"url": footprint},
         "pins": [{"name": p} if isinstance(p, str) else {"name": p[0], "pad": p[1], "description": p[2]} for p in pins],
         "metadata": {"assembly": assembly}}
    if lcsc: c["metadata"]["lcsc"] = lcsc
    if mpn: c["metadata"]["mpn"] = mpn
    if manufacturer: c["metadata"]["manufacturer"] = manufacturer
    if params: c["parameters"] = params
    return c


def nums(n):
    return [str(i) for i in range(1, n + 1)]


def write_library():
    if os.path.exists(LIB):
        shutil.rmtree(LIB)
    os.makedirs(os.path.join(LIB, "components"))
    os.makedirs(os.path.join(LIB, "footprints"))
    fps = [
        dip(24, 7.62, "DIP-24_W7.62mm_Socket", "DIP-24 300 mil socket (ATF22V10C is the narrow DIP); pin 1 top-left, long axis vertical"),
        dip(28, 15.24, "DIP-28_W15.24mm_Socket", "DIP-28 600 mil socket; pin 1 top-left, long axis vertical"),
        dip(32, 15.24, "DIP-32_W15.24mm_Socket", "DIP-32 600 mil socket; pin 1 top-left, long axis vertical"),
        quad("LQFP-48_7x7mm_P0.5mm", "LQFP-48, 7 x 7 mm body, 0.5 mm pitch, IPC nominal pads; pin 1 top-left", 12, 0.5, 4.35, 1.5, 0.3, 7.0),
        dual("TSSOP-16_4.4x5mm_P0.65mm", "TSSOP-16, 4.4 mm body, 0.65 mm pitch; pin 1 top-left", 16, 0.65, 2.95, 1.45, 0.45, 4.4),
        dual("SOIC-20W_7.5x12.8mm_P1.27mm", "SOIC-20 wide (7.5 mm body), 1.27 mm pitch; pin 1 top-left", 20, 1.27, 4.85, 1.9, 0.6, 7.5),
        sot23_5(), sc88(), sot143(), xtal3225(), osc7050(), tact51(), slide(), dipsw4(),
        header(2, 10, "PinHeader_2x10_P2.54mm"), header(1, 3, "PinHeader_1x03_P2.54mm"), db9f(),
    ]
    for f in fps:
        json.dump(f, open(os.path.join(LIB, "footprints", f["name"] + ".json"), "w"), indent=1)
    F = lambda n: f"../footprints/{n}.json"
    J = lambda n: os.path.join(JLC, "footprints", n + ".json")
    comps = [
        comp("socket-dip24-300", "DIP-24 300 mil socket for an ATF22V10C (CONNFLY DS1009-24AT1NX-0A2 or equivalent; LCSC number to source); the GAL is supplied and programmed separately", F("DIP-24_W7.62mm_Socket"), nums(24), mpn="DS1009-24AT1NX-0A2", manufacturer="CONNFLY"),
        comp("socket-dip28-600", "DIP-28 600 mil socket for an AS7C164A (CONNFLY DS1009-28AT1WX-0A2, LCSC C72121)", F("DIP-28_W15.24mm_Socket"), nums(28), lcsc="C72121", mpn="DS1009-28AT1WX-0A2", manufacturer="CONNFLY"),
        comp("socket-dip32-600", "DIP-32 600 mil socket for an SST39SF040 (CONNFLY DS1009-32AT1WX-0A2, LCSC C72122)", F("DIP-32_W15.24mm_Socket"), nums(32), lcsc="C72122", mpn="DS1009-32AT1WX-0A2", manufacturer="CONNFLY"),
        comp("tl16c550d-lqfp48", "TI TL16C550DPTR UART, LQFP-48 (PT); pins by package number. LCSC number to confirm", F("LQFP-48_7x7mm_P0.5mm"), nums(48), mpn="TL16C550DPTR", manufacturer="Texas Instruments"),
        comp("sp3232-tssop16", "MaxLinear SP3232EEY-L/TR RS-232 transceiver, TSSOP-16 (LCSC C13482)", F("TSSOP-16_4.4x5mm_P0.65mm"), nums(16), lcsc="C13482", mpn="SP3232EEY-L/TR", manufacturer="MaxLinear"),
        comp("74hc541-soic20w", "Nexperia 74HC541D,653 octal buffer, SOIC-20 wide (LCSC C126008)", F("SOIC-20W_7.5x12.8mm_P1.27mm"), nums(20), lcsc="C126008", mpn="74HC541D,653", manufacturer="Nexperia"),
        comp("74lvc1g157-sc88", "Nexperia 74LVC1G157GW,125 single 2:1 multiplexer, SC-88: 1 I0, 2 I1, 3 S, 4 Y, 5 GND, 6 VCC (LCSC C135822)", F("SC-88_SOT-363"), nums(6), lcsc="C135822", mpn="74LVC1G157GW,125", manufacturer="Nexperia"),
        comp("74lvc1g14-sot23-5", "TI SN74LVC1G14DBVR Schmitt inverter, SOT-23-5: 1 NC, 2 A, 3 GND, 4 Y, 5 VCC (LCSC C7835)", F("SOT-23-5"), nums(5), lcsc="C7835", mpn="SN74LVC1G14DBVR", manufacturer="Texas Instruments"),
        comp("max811-sot143", "Analog Devices MAX811LEUS+T reset supervisor 4.63 V, SOT-143: 1 GND, 2 RESET#, 3 MR#, 4 VCC. LCSC number to confirm", F("SOT-143"), nums(4), mpn="MAX811LEUS+T", manufacturer="Analog Devices"),
        comp("crystal-14.7456-3225", "14.7456 MHz crystal, 3225 four-pad, 12 pF (LCSC C2885591): pins 1 and 3 crystal, 2 and 4 case", F("Crystal_3225_4Pin"), nums(4), lcsc="C2885591"),
        comp("osc-4mhz-5v-7050", "4.000 MHz 5 V CMOS oscillator, 7050 four-pad: 1 EN, 2 GND, 3 OUT, 4 VCC. Part and LCSC number to pick", F("Oscillator_7050_4Pin"), nums(4)),
        comp("tact-5.1", "XKB TS-1187A-B-A-B tactile switch, 5.1 x 5.1 mm SMD (LCSC C318884); pads 1 and 3 are diagonal", F("SW_SMD_5.1x5.1_P3.7_LS6.5"), nums(4), lcsc="C318884", mpn="TS-1187A-B-A-B", manufacturer="XKB"),
        comp("slide-msk12c02", "SHOU HAN MSK12C02 SPDT slide switch, SMD (LCSC C431540); pin 2 common", F("SW_Slide_MSK12C02"), nums(3), lcsc="C431540", mpn="MSK12C02", manufacturer="SHOU HAN"),
        comp("dipsw-em04q", "Diptronics EM-04-Q 4-position SMD DIP switch (LCSC C501635); position k joins pins k and 9-k", F("SW_DIP_4_SMD_EM-04-Q"), nums(8), lcsc="C501635", mpn="EM-04-Q", manufacturer="Diptronics"),
        comp("header-2x10", "2x10 pin header, 2.54 mm, through hole; LCSC number to pick", F("PinHeader_2x10_P2.54mm"), nums(20)),
        comp("header-1x03", "1x3 pin header, 2.54 mm, through hole; LCSC number to pick", F("PinHeader_1x03_P2.54mm"), nums(3)),
        comp("db9-female-ra", "DE-9 female, right angle, through hole (JLCPCB assembly library C9900026339); may need hand soldering", F("DSUB-9_Female_Horizontal"), nums(9), lcsc="C9900026339"),
        comp("ao3401a-sot23", "AOS AO3401A P-channel MOSFET, SOT-23: 1 G, 2 S, 3 D (LCSC C15127, JLCPCB basic)", J("SOT-23-3"), [("G", "1", "gate"), ("S", "2", "source"), ("D", "3", "drain")], lcsc="C15127", mpn="AO3401A", manufacturer="AOS"),
        comp("fuse-1206", "PTC resettable fuse, 1206 (SMD1206P110TF/16, 1.1 A hold, LCSC C523825)", J("1206"), nums(2), lcsc="C523825", mpn="SMD1206P110TF/16", manufacturer="PTTC"),
    ]
    for c in comps:
        json.dump(c, open(os.path.join(LIB, "components", c["name"] + ".json"), "w"), indent=1)
    json.dump({"schema": "pcb-component-index/1", "name": "grit", "description": "Parts for the grit board that the flatland libraries do not have", "components": {}}, open(os.path.join(LIB, "index.json"), "w"), indent=1)
    return [c["name"] for c in comps]


# ------------------------------------------------------------- part mapping
def component_for(chip):
    """(component name, params, pin map board-pin -> component pin)."""
    name, part = chip["name"], chip["part"]
    ident = lambda n: {str(i): str(i) for i in range(1, n + 1)}
    if part.startswith("ATF22V10C"): return "socket-dip24-300", {}, ident(24)
    if part.startswith("SST39SF040"): return "socket-dip32-600", {}, ident(32)
    if part.startswith("AS7C164A"): return "socket-dip28-600", {}, ident(28)
    if part.startswith("TL16C550"): return "tl16c550d-lqcp48".replace("lqcp", "lqfp"), {}, ident(48)
    if part.startswith("SP3232"): return "sp3232-tssop16", {}, ident(16)
    if part.startswith("74HC541"): return "74hc541-soic20w", {}, ident(20)
    if part.startswith("74LVC1G157"): return "74lvc1g157-sc88", {}, ident(6)
    if part.startswith("SN74LVC1G14"): return "74lvc1g14-sot23-5", {}, ident(5)
    if part.startswith("MAX811"): return "max811-sot143", {}, ident(4)
    if part.startswith("Crystal"): return "crystal-14.7456-3225", {}, {"1": "1", "2": "3"}
    if part.startswith("Oscillator"): return "osc-4mhz-5v-7050", {}, ident(4)
    if part.startswith("TS-1187A"): return "tact-5.1", {}, ident(4)
    if part.startswith("MSK12C02"): return "slide-msk12c02", {}, ident(3)
    if part.startswith("EM-04-Q"): return "dipsw-em04q", {}, ident(8)
    if part.startswith("Header 2x10"): return "header-2x10", {}, ident(20)
    if part.startswith("Header 1x3"): return "header-1x03", {}, ident(3)
    if part.startswith("DB9"): return "db9-female-ra", {}, ident(9)
    if part.startswith("AO3401A"): return "ao3401a-sot23", {}, {"1": "G", "2": "S", "3": "D"}
    if part.startswith("Polyfuse"): return "fuse-1206", {}, ident(2)
    if part.startswith("CUI PJ-002AH"): return "dcjack-pj-002ah-smt", {}, {"1": "PIN", "2": "SW", "3": "SLEEVE"}
    if part.startswith("100uF"): return "capacitor-elec-6.3x7.7", {"value": "100u", "lcsc": "C970684"}, {"1": "+", "2": "-"}
    if part.startswith("10uF"): return "capacitor-0805", {"value": "10u", "lcsc": "C15850"}, ident(2)
    if part.startswith("100nF"): return "capacitor-0603", {"value": "100n", "lcsc": "C14663"}, ident(2)
    if part.startswith("18pF"): return "capacitor-0603", {"value": "18p", "lcsc": "C1653"}, ident(2)
    if part.startswith("LED"): return "led-0603-red", {}, {"1": "A", "2": "K"}
    if part.startswith("1k"): return "resistor-0603", {"value": "1k", "lcsc": "C21190"}, ident(2)
    if part.startswith("10k"): return "resistor-0603", {"value": "10k", "lcsc": "C25804"}, ident(2)
    raise SystemExit(f"no component for {name}: {part}")


# ----------------------------------------------------------------- floorplan
W, H = 230.0, 175.0
GAL_ROW_A = 129.0   # control + ALU
GAL_ROW_B = 93.0    # datapath
MEM_ROW = 46.0
GAL_X0, GAL_DX = 22.0, 12.5
MEM_X0, MEM_DX = 26.0, 20.0

PLACE = {}
for i, n in enumerate(["seq0", "seq1", "mir0", "mir1", "alu0", "alu1", "alu2", "alu3"]):
    PLACE[n] = (GAL_X0 + i * GAL_DX, GAL_ROW_A, 0)
for i, n in enumerate(["pc0", "pc1", "a0", "a1", "b0", "b1", "t0", "t1"]):
    PLACE[n] = (GAL_X0 + i * GAL_DX, GAL_ROW_B, 0)
for i, n in enumerate(["uc0", "uc1", "rom0", "rom1", "ram0", "ram1"]):
    PLACE[n] = (MEM_X0 + i * MEM_DX, MEM_ROW, 0)
# decoupling caps: just above each socket, rotated to lie along x
for n in list(PLACE):
    x, y, _ = PLACE[n]
    top = 16.0 if n[:2] in ("uc", "ro", "ra") and not n.startswith("t") else 17.0
    if n.startswith(("uc", "rom", "ram")):
        PLACE["cd_" + n] = (x + 5.0, y + 23.5, 0)
    else:
        PLACE["cd_" + n] = (x, y + 18.0, 0)
# clock corner, left of the GAL rows
PLACE.update({
    "osc0": (8.0, 118.0, 90), "cd_osc0": (8.0, 124.5, 0),
    "umux0": (8.0, 110.0, 0), "cd_umux0": (8.0, 106.5, 0),
    "uinv0": (8.0, 100.0, 0), "cd_uinv0": (8.0, 96.0, 0),
    "rstep0": (8.0, 92.0, 0), "cst0": (8.0, 89.0, 0),
    "swstep0": (9.0, 80.0, 0), "swm0": (9.0, 70.0, 0), "jclk0": (6.0, 148.0, 0),
})
# UART corner, right of the memory row
PLACE.update({
    "uart0": (160.0, 60.0, 0), "cd_uart0": (160.0, 70.0, 0),
    "x1": (150.0, 50.0, 0), "xc1": (146.0, 45.0, 90), "xc2": (154.0, 45.0, 90),
    "xcvr0": (185.0, 60.0, 0), "cd_xcvr0": (185.0, 67.0, 0),
    "c1": (178.0, 52.0, 90), "c2": (181.5, 52.0, 90), "c3": (188.5, 52.0, 90), "c4": (192.0, 52.0, 90),
    "j1": (205.0, 9.0, 0),
})
# reset, right of the datapath row
PLACE.update({"rst0": (140.0, 93.0, 0), "cd_rst0": (140.0, 97.5, 0), "swr0": (140.0, 83.0, 0)})
# bank switches
PLACE.update({"swb0": (160.0, 96.0, 0), "swb1": (175.0, 96.0, 0)})
for i in range(4):
    PLACE[f"rp_pbank{i}"] = (153.0 + i * 3.5, 105.0, 90)
for i in range(3):
    PLACE[f"rp_ubank{i}"] = (169.0 + i * 3.5, 105.0, 90)
PLACE.update({"rd0": (150.0, 128.0, 0), "rd1": (150.0, 131.0, 0)})
# power entry, bottom-left
PLACE.update({
    "jpwr0": (14.0, 14.0, 90), "f0": (26.0, 10.0, 0), "q0": (33.0, 10.0, 0), "cb0": (42.0, 12.0, 0),
    "rlp0": (26.0, 5.0, 0), "ledp0": (31.0, 5.0, 0),
    "cb1": (60.0, 74.0, 0), "cb2": (128.0, 74.0, 0), "cb3": (128.0, 111.0, 0), "cb4": (128.0, 140.0, 0),
})
# LED row along the top edge, buffers under it
LED_Y, R_Y, BUF_Y = 169.5, 165.5, 157.0
LED_ORDER = [
    ["CLK", "RESET", "PCDRV", "ADRV", "MEMRD_n", "WE_n", "ALUOE", "TDRV"],
    ["ALD", "BLD", "TLD", "PCLD", "PCINC", "IRLD", "NEL", "AUX0"],
    ["IR0", "IR1", "IR2", "IR3", "IR4", "STEP0", "STEP1", "STEP2"],
    ["STEP3", "F0", "F1", "AUX1", "RST_n"],
]
x = 20.0
for b, group in enumerate(LED_ORDER):
    PLACE[f"ubuf{b}"] = (x + 17.0, BUF_Y, 0)
    PLACE[f"cd_ubuf{b}"] = (x + 17.0 + 8.5, BUF_Y, 90)
    for sig in group:
        s = sig.lower()
        PLACE[f"led_{s}"] = (x, LED_Y, 90)
        PLACE[f"rl_{s}"] = (x, R_Y, 90)
        x += 5.0
    x += 6.0
# analyzer headers along the bottom edge
PLACE.update({"ja0": (50.0, 6.0, 90), "jd0": (82.0, 6.0, 90), "jc0": (114.0, 6.0, 90), "js0": (146.0, 6.0, 90)})
HOLES = [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]


# --------------------------------------------------------------------- main
def main():
    names = write_library()
    if os.path.exists(os.path.join(HERE, "pcb.json")):
        os.remove(os.path.join(HERE, "pcb.json"))
    r = subprocess.run([PCB, "init", "grit", "--dir", HERE, "--layers", LAYERS, "--index", os.path.join(JLC, "index.json"), "--force"], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr); sys.exit(1)
    for n in names:
        run("index", "register", LIB, os.path.join(LIB, "components", n + ".json"), quiet=True)
    run("index", "update", LIB, quiet=True)
    run("index", "add", LIB, quiet=True)
    run("drc", "add", "jlcpcb-fr4-4layer" if FOUR else "jlcpcb-fr4-2layer", quiet=True)
    run("rules", "set", "trace_width=0.25", "clearance=0.2", "via_drill=0.4", "via_diameter=0.8", "pour_clearance=0.3", "edge_clearance=0.5", quiet=True)
    run("rules", "class", "power", "--width", "0.6", "--clearance", "0.25", quiet=True)
    run("drc", "waive", "pth-annular-ring", "jpwr0", "j1", "--reason", "non-plated locating pegs of the jack and mounting holes of the DB9 carry no copper by design", quiet=True)
    run("drc", "waive", "pth-annular-ring-recommended", "jpwr0", "j1", "--reason", "same: non-plated holes", quiet=True)

    chips = BOARD["chips"]
    pinmaps = {}
    for c in chips:
        comp_name, params, pinmap = component_for(c)
        pinmaps[c["name"]] = pinmap
        args = ["add", c["name"], comp_name]
        for k, v in params.items():
            args += ["-P", f"{k}={v}"]
        run(*args, quiet=True)
    # nets
    nets = {}
    for c in chips:
        for p in c["pins"]:
            if not p.get("net"):
                continue
            cp = pinmaps[c["name"]].get(str(p["pin"]))
            if cp is None:
                continue
            nets.setdefault(p["net"], []).append(f"{c['name']}.{cp}")
    # extras the board file does not carry: crystal case pads, jack switch contact to the sleeve
    nets["GND"] += ["x1.2", "x1.4", "jpwr0.SW"]
    for net, pins in nets.items():
        if len(pins) < 2:
            print("single-pin net", net, pins)
            continue
        run("connect", *pins, "--net", net, quiet=True)
    for net in ("VCC", "GND", "VIN", "VF"):
        run("net", "class", net, "power", quiet=True)
    # board
    run("outline", "rect", W, H, "--radius", 3, quiet=True)
    for (x, y) in HOLES:
        run("hole", "add", f"{x},{y}", "--drill", 3.2, quiet=True)
    for c in chips:
        n = c["name"]
        if n not in PLACE:
            raise SystemExit(f"unplaced: {n}")
        x, y, r = PLACE[n]
        run("place", n, f"{x},{y}", "--rotation", r, quiet=True)
    if FOUR:
        run("pour", "new", "gnd", "--layer", "In1.Cu", "--net", "GND", "--follow-outline", quiet=True)
        run("pour", "new", "vcc", "--layer", "In2.Cu", "--net", "VCC", "--follow-outline", quiet=True)
    else:
        run("pour", "new", "gnd", "--layer", "B.Cu", "--net", "GND", "--follow-outline", quiet=True)
        run("pour", "new", "vcc", "--layer", "F.Cu", "--net", "VCC", "--follow-outline", quiet=True)
    run("status")
    if ROUTE:
        # freerouting writes build/grit.ses; import it with the redundancy pass
        # skipped (that pass takes hours on a board of this size)
        run("route", "--dsn-only", quiet=True)
        r = subprocess.run(["/Applications/freerouting.app/Contents/MacOS/freerouting", "-de", os.path.join(HERE, "build", "grit.dsn"), "-do", os.path.join(HERE, "build", "grit.ses"), "-mp", "14", "-dct", "0", "-da", "-dl", "--gui.enabled=false"], capture_output=True, text=True, env={**os.environ, "JAVA_TOOL_OPTIONS": "-Djava.awt.headless=true"})
        open(os.path.join(HERE, "build", "freerouting.log"), "w").write(r.stdout + r.stderr)
        run("route", "--import", os.path.join(HERE, "build", "grit.ses"), "--keep-redundant")
        run("check", check=False)
        run("visualize", "pcb")
        run("gerbers", check=False)
        run("bom", "--all", check=False)
        run("pnp", check=False)


if __name__ == "__main__":
    main()
