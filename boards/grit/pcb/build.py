#!/usr/bin/env python3
"""Build the grit PCB project for flatland (`pcb`) from the board file.

    python3 boards/grit/pcb/build.py            # library + netlist + placement
    python3 boards/grit/pcb/build.py --route    # ... then autoroute, check, outputs

Four layers (ground and 5 V on the inner planes) unless --2layer.

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
# freerouting 2.2.4 (the installed app) hangs on nets that mix protected
# wiring with pins still to route; 2.4.1 does not.  $FREEROUTING points at
# the launcher of the build to use.
FREEROUTING = os.environ.get("FREEROUTING", "/Applications/freerouting.app/Contents/MacOS/freerouting")
LIB = os.path.join(HERE, "lib")
ROUTE = "--route" in sys.argv
CORE_PREFIXES = ("seq", "mir", "pc", "a", "b", "t", "alu", "rom", "uc", "ram", "uart", "xcvr", "osc", "umux", "uinv", "rst")


def is_core(name):
    prefix = "".join(ch for ch in name if ch.isalpha())
    return prefix in CORE_PREFIXES
FOUR = "--2layer" not in sys.argv   # four layers is the design; --2layer only for comparison
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
# Built up in stages (--stage N), looking at the ratsnest and the routed
# result at each: 1 memories and UART on explicit bus wires; 2 + the
# address drivers a0/a1; 3 + pc, b, t; 4 + the ALU; 5 + the sequencer,
# microcode ROM and pipeline register; 6 + clock and reset; 7 everything.
STAGE = int(next((a.split("=")[1] for a in sys.argv if a.startswith("--stage=")), "1"))
W, H = 300.0, 175.0   # one chip row between the two buses needs the width
HOLES = [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]

# Placement A: the data bus band at the top, two chip rows between the
# buses, the address bus band at the bottom, every chip of the machine
# between the bands in stacks: the low byte's chip over the high byte's
# (rom0 over rom1, a0 over a1, ...), uc0 over uc1, mir0 over mir1, the
# UART over its transceiver.  The buses are literal top-layer wires.
# Each stack's far connections (the upper chip's address pins, the lower
# chip's data pins) run as a vertical mini-bus in the gap beside the
# stack; the router only gets the control nets.
W, H = 370.0, 178.0
HOLES = [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]
ROW_A, ROW_B = 122.0, 69.0
BUS_X0, BUS_X1 = 18.0, W - 34.0   # the left end leaves room for the bus headers, the right end for the LED banks
DBUS_Y0, ABUS_Y0, BUS_PITCH = 148.0, 43.0, 1.0
# the control band: the pipeline register's lines, CLK and RESET, below the address band
CTRL_NETS = ["PCDRV", "MEMRD_n", "ALUOE", "WE_n", "ALD", "BLD", "PCLD", "PCINC", "IRLD", "F0", "F1", "ADRV", "TLD", "TDRV", "CLK", "RESET", "NE3"]   # NE3: the ALU's not-equal, the length of the board to the flags chip
CTRL_Y0, CTRL_PITCH = 21.0, 0.8
BUS_WIDTH = 0.3
STUB_WIDTH = 0.25
VIA_DRILL, VIA_DIA = 0.2, 0.5
STUB_PITCH = 0.55
MINI_X0 = {"B.Cu": 2.85, "F.Cu": 3.0}   # first mini-bus line this far from the pad column, beyond the three outside stub slots
GAL_DX, MEM_DX = 23.0, 32.0   # a gap holds up to 26 vertical lines at 0.55 mm on one layer
NO_STUBS = "--stubs" not in sys.argv


def bus_y(name, i):
    if name == "CTRL":
        return CTRL_Y0 - i * CTRL_PITCH
    return DBUS_Y0 + i * BUS_PITCH if name == "D" else ABUS_Y0 - i * BUS_PITCH


def bus_of(net):
    """(bus name, index) of a net that runs on a band, else None."""
    if net.startswith("D") and net[1:].isdigit():
        return "D", int(net[1:])
    if net.startswith("ADDR") and net[4:].isdigit():
        return "ADDR", int(net[4:])
    if net in CTRL_NETS:
        return "CTRL", CTRL_NETS.index(net)
    return None


PLACE = {}
STAGE_CHIPS = {1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: []}
X_SEQ, X_UC, X_MIR, X_ROM, X_RAM = 30.0, 56.0, 80.0, 112.0, 144.0
X_GAL0 = 170.0
X_UART = X_GAL0 + 6 * GAL_DX + 8.0
COLUMN_XS = [X_SEQ, X_UC, X_MIR, X_ROM, X_RAM] + [X_GAL0 + k * GAL_DX for k in range(6)] + [X_UART]


def tap_columns():
    """x of the bus taps: the centre of every gap between chip columns,
    clear of the stub slots and mini-buses beside the columns; not beside
    the UART, whose escapes fill both gaps around it."""
    xs = sorted(COLUMN_XS)
    return [(a + b) / 2 for a, b in zip(xs, xs[1:]) if X_UART not in (a, b)]


# stage 1: memories and UART; the DIP memories rotated so their data pins face the data bus
PLACE.update({"rom0": (X_ROM, ROW_A, 180), "rom1": (X_ROM, ROW_B, 180), "ram0": (X_RAM, ROW_A, 180), "ram1": (X_RAM, ROW_B, 180), "uart0": (X_UART, ROW_A, 0)})
STAGE_CHIPS[1] = ["rom0", "rom1", "ram0", "ram1", "uart0"]
# stage 2: the address drivers, a0 over a1
PLACE.update({"a0": (X_GAL0, ROW_A, 0), "a1": (X_GAL0, ROW_B, 0)})
STAGE_CHIPS[2] = ["a0", "a1"]
# stage 3: pc, b, t
# pc, t, then the ALU with the B latch between its halves: b0's outputs
# face left to alu0/alu1 (b0 is turned round), b1's face right to
# alu2/alu3, so every B operand crosses one gap to an adjacent stack
PLACE.update({"pc0": (X_GAL0 + 1 * GAL_DX, ROW_A, 0), "pc1": (X_GAL0 + 1 * GAL_DX, ROW_B, 0), "t0": (X_GAL0 + 2 * GAL_DX, ROW_A, 0), "t1": (X_GAL0 + 2 * GAL_DX, ROW_B, 0),
              "b0": (X_GAL0 + 4 * GAL_DX, ROW_A, 180), "b1": (X_GAL0 + 4 * GAL_DX, ROW_B, 0)})
STAGE_CHIPS[3] = ["pc0", "pc1", "b0", "b1", "t0", "t1"]
# stage 4: the ALU, alu0 over alu1 left of the B latch, alu2 over alu3 right of it
PLACE.update({"alu0": (X_GAL0 + 3 * GAL_DX, ROW_A, 0), "alu1": (X_GAL0 + 3 * GAL_DX, ROW_B, 0), "alu2": (X_GAL0 + 5 * GAL_DX, ROW_A, 0), "alu3": (X_GAL0 + 5 * GAL_DX, ROW_B, 0)})
STAGE_CHIPS[4] = ["alu0", "alu1", "alu2", "alu3"]
# stage 5: sequencer, microcode ROM, pipeline register, stacked at the left end
PLACE.update({"seq0": (X_SEQ, ROW_A, 0), "seq1": (X_SEQ, ROW_B, 0), "uc0": (X_UC, ROW_A, 0), "uc1": (X_UC, ROW_B, 0), "mir0": (X_MIR, ROW_A, 0), "mir1": (X_MIR, ROW_B, 0)})
STAGE_CHIPS[5] = ["seq0", "seq1", "uc0", "uc1", "mir0", "mir1"]
# stage 6: clock and reset in the left strip, the transceiver under the UART
PLACE.update({"osc0": (8.0, 132.0, 90), "umux0": (8.0, 124.0, 0), "uinv0": (8.0, 117.0, 0), "rst0": (8.0, 100.0, 0), "xcvr0": (X_UART, ROW_B, 0)})
STAGE_CHIPS[6] = ["osc0", "umux0", "uinv0", "rst0", "xcvr0"]


# stage 7: everything else
def _place_rest():
    # decoupling: between each socket and its bus band, or beside an SMD chip
    by_name = {c["name"]: c for c in BOARD["chips"]}
    for n, (x, y, r) in list(PLACE.items()):
        if n in ("seq0", "a0", "pc0", "b0", "t0", "alu0", "alu2"):   # upper row: above
            # above the column whose stubs do not run upward (its data pins
            # go up, its address pins down); the stub tracks fill the space
            # above the other column
            pads = dip_pads(by_name[n])
            up, down = {-1: 0, 1: 0}, {-1: 0, 1: 0}
            for pin in by_name[n]["pins"]:
                net = pin.get("net") or ""
                if net.startswith("D") and net[1:].isdigit():
                    up[pads[pin["pin"]][2]] += 1
                elif net.startswith("ADDR"):
                    down[pads[pin["pin"]][2]] += 1
            for col, above in ((-1, True), (1, True), (-1, False), (1, False)):
                if (up if above else down)[col] == 0:
                    break
            PLACE["cd_" + n] = (x + col * 3.81, y + 18.5, 0)
        elif n in ("seq1", "a1", "pc1", "b1", "t1", "alu1", "alu3"):   # lower row: below
            PLACE["cd_" + n] = (x, y - 18.5, 0)
        elif n in ("rom0", "ram0"):
            PLACE["cd_" + n] = (x, y + 23.5, 0)
        elif n in ("rom1", "ram1"):
            PLACE["cd_" + n] = (x, y - 23.5, 0)
        elif n in ("mir0", "mir1"):
            PLACE["cd_" + n] = (x, y + 17.5, 0)
        elif n in ("uc0", "uc1"):
            PLACE["cd_" + n] = (x, y + 22.5, 0)
    PLACE.update({"cd_uart0": (X_UART - 8.0, ROW_A + 8.0, 90), "cd_xcvr0": (X_UART, ROW_B + 6.0, 0), "cd_osc0": (8.0, 166.5, 0), "cd_rst0": (8.0, 104.5, 0), "cd_umux0": (8.0, 148.5, 0), "cd_uinv0": (8.0, 141.5, 0)})
    # the UART's crystal between the UART and the transceiver
    PLACE.update({"x1": (X_UART, ROW_A - 16.0, 0), "xc1": (X_UART - 4.0, ROW_A - 20.0, 90), "xc2": (X_UART + 4.0, ROW_A - 20.0, 90)})
    # transceiver's charge-pump capacitors beside it, the DB9 on the bottom edge under them
    PLACE.update({"c1": (X_UART - 7.0, ROW_B, 90), "c2": (X_UART - 4.5, ROW_B, 90), "c3": (X_UART + 4.5, ROW_B, 90), "c4": (X_UART + 7.0, ROW_B, 90), "j1": (X_UART, 9.0, 0)})
    # clock corner: step button, slide switch, debounce, clock header
    PLACE.update({"swstep0": (9.0, 130.0, 0), "rstep0": (8.0, 136.5, 0), "cst0": (8.0, 139.5, 0), "swm0": (9.0, 120.0, 0), "jclk0": (5.0, 178.0, 90)})
    # reset button by the supervisor
    PLACE.update({"swr0": (9.0, 92.0, 0)})
    # bank switches and their pull-ups, and the select pull-downs, bottom-left
    PLACE.update({"swb0": (30.0, 8.0, 0), "swb1": (46.0, 8.0, 0)})
    for i in range(4):
        PLACE[f"rp_pbank{i}"] = (23.0 + i * 3.5, 15.5, 90)
    for i in range(3):
        PLACE[f"rp_ubank{i}"] = (39.0 + i * 3.5, 15.5, 90)
    PLACE.update({"rd0": (60.0, 12.0, 270), "rd1": (64.0, 12.0, 270)})   # pad 1 (the address line) toward the bus
    # power entry
    # power entry in the strip left of the sequencer, the plug entering from the left edge
    PLACE.update({"jpwr0": (8.0, 43.0, 90), "cb0": (8.0, 58.0, 0), "f0": (8.0, 65.0, 0), "q0": (8.0, 70.0, 0), "rlp0": (8.0, 75.0, 0), "ledp0": (8.0, 79.0, 0),
                  "cb1": (X_UART, 83.0, 0), "cb2": (X_GAL0 + 3.5 * GAL_DX, 83.0, 0), "cb3": (150.0, 160.0, 0), "cb4": (120.0, 160.0, 0)})
    # LEDs and their buffers along the top edge
    LED_Y, R_Y, BUF_Y = H - 5.0, H - 9.0, H - 17.0
    x = 20.0
    for b, group in enumerate(LED_ORDER):
        PLACE[f"ubuf{b}"] = (x + 17.0, BUF_Y, 0)
        PLACE[f"cd_ubuf{b}"] = (x + 17.0 + 8.5, BUF_Y, 90)
        for sig in group:
            s = sig.lower()
            PLACE[f"led_{s}"] = (x, LED_Y, 90)
            PLACE[f"rl_{s}"] = (x, R_Y, 90)
            x += 4.5
        x += 5.0
    # analyzer headers along the bottom edge
    # the address header under the address bus, the data header above the
    # data bus (top-left), the two control headers along the bottom
    # the address header under the stub-free stretch below B and T
    PLACE.update({"ja0": (110.0, 6.0, 90), "jd0": (120.0, 155.0, 90), "jc0": (140.0, 6.0, 90), "js0": (160.0, 155.0, 90)})


LED_ORDER = [
    ["CLK", "RESET", "PCDRV", "ADRV", "MEMRD_n", "WE_n", "ALUOE", "TDRV"],
    ["ALD", "BLD", "TLD", "PCLD", "PCINC", "IRLD", "NEL", "AUX0"],
    ["IR0", "IR1", "IR2", "IR3", "IR4", "STEP0", "STEP1", "STEP2"],
    ["STEP3", "F0", "F1", "AUX1", "RST_n"],
]


def finish_floorplan():
    _place_rest()
    dropped = lambda n: n.startswith(("led", "rl", "ubuf", "cd_ubuf"))   # no LEDs or their buffers on this board
    STAGE_CHIPS[7] = [c["name"] for c in BOARD["chips"] if c["name"] not in sum(STAGE_CHIPS.values(), []) and not dropped(c["name"])]


def stage_names():
    names = []
    for s in range(1, STAGE + 1):
        names += STAGE_CHIPS[s]
    return names


def dip_pads(chip):
    """Board coordinates of a placed DIP's pads: {pin: (x, y, column)} with
    column -1 for the left column, +1 for the right, after rotation."""
    n = int(chip["package"].split("-")[1].split(" ")[0])
    row = 7.62 if n == 24 else 15.24
    x0, y0, rot = PLACE[chip["name"]]
    half = n // 2
    out = {}
    for i in range(half):
        y = (half - 1) / 2 * 2.54 - i * 2.54
        for pin, x in ((i + 1, -row / 2), (n - i, row / 2)):
            if rot == 180:
                px, py = -x, -y
            else:
                px, py = x, y
            out[pin] = (x0 + px, y0 + py, -1 if px < 0 else 1)
    return out


VIA_XS = {}   # net -> [x of every via on its bus line]


def draw_stubs(chips):
    """Bottom-layer tracks beside the DIP columns.  Two kinds: a stub from
    a bus pin to a via on its bus line (only toward the near bus: the far
    bus, past the other row, is the router's), and a link between the
    same pin of two chips stacked in one column when they carry the same
    net (rom0/rom1's address pins, a0/a1's control pins).  A stacked pair
    is one column group, so the slot allocation sees every track beside
    the column at once: a track takes the innermost slot whose occupants
    it does not overlap in y, and never a slot further out than a track
    it would cross with one of its jogs.  DIP-24 columns have three slots
    outside and five inside; wider DIPs two and ten."""
    groups = {}   # (x of the column, package kind) -> {"n": pins, "items": [...]}
    dips = [c for c in chips if c["package"].startswith("DIP")]
    for c in dips:
        n = int(c["package"].split("-")[1].split(" ")[0])
        pads = dip_pads(c)
        upper = PLACE[c["name"]][1] > (ROW_A + ROW_B) / 2
        for p in c["pins"]:
            net = p.get("net") or ""
            bus = "D" if net.startswith("D") and net[1:].isdigit() else "ADDR" if net.startswith("ADDR") else None
            if bus is None:
                continue
            if (bus == "D") != upper and PLACE[c["name"]][1] < DBUS_Y0:
                continue   # the far bus, past the other row
            x, y, col = pads[p["pin"]]
            yb = bus_y(bus, int(net[len(bus):]))
            groups.setdefault((round(x, 2), n, "B.Cu"), []).append({"net": net, "x": x, "jogs": [y], "lo": min(y, yb), "hi": max(y, yb), "end": yb, "via": True, "col": col, "upper": upper})
    # links within a stack: only with --links.  Two parallel links between
    # the same pins of stacked chips have interleaved endpoints along the
    # column, so on one layer they must cross; the router does better
    # with both layers.
    by_pos = {}
    if "--links" not in sys.argv:
        by_pos = None
    for c in (dips if by_pos is not None else []):
        x, y, r = PLACE[c["name"]]
        by_pos.setdefault((round(x, 2), c["package"], r), []).append(c)
    for key, pair in (by_pos or {}).items():
        if len(pair) != 2:
            continue
        upper, lower = sorted(pair, key=lambda c: -PLACE[c["name"]][1])
        n = int(upper["package"].split("-")[1].split(" ")[0])
        pu, pl = dip_pads(upper), dip_pads(lower)
        nets_l = {p["pin"]: p.get("net") for p in lower["pins"]}
        for p in upper["pins"]:
            net = p.get("net")
            if not net or nets_l.get(p["pin"]) != net or net in ("VCC", "GND"):
                continue
            (xu, yu, col), (xl, yl, _) = pu[p["pin"]], pl[p["pin"]]
            groups.setdefault((round(xu, 2), n, "F.Cu"), []).append({"net": net, "x": xu, "jogs": [yu, yl], "lo": yl, "hi": yu, "end": yl, "via": False, "col": col})
    for (xcol, n, layer), items in groups.items():
        col = items[0]["col"]
        n_out = 3 if n == 24 else 2
        n_in = 5 if n == 24 else 10
        # greedy order: a track may go next if its span contains no other
        # remaining track's jog row
        order, rest = [], list(items)
        while rest:
            pick = next((it for it in rest if not any(it["lo"] < j < it["hi"] for o in rest if o is not it for j in o["jogs"])), None)
            if pick is None:
                pick = min(rest, key=lambda it: it["hi"] - it["lo"])
            order.append(pick)
            rest.remove(pick)
        for side, group, cap in (("out", order[:n_out], n_out), ("in", order[n_out:], n_in)):
            sign = col if side == "out" else -col
            slots, placed = [], []   # per slot: spans; placed: (slot, jog rows)
            for it in group:
                lo, hi = it["lo"] - 0.6, it["hi"] + 0.6
                s = 0
                while True:
                    if s < len(slots) and any(not (hi < a or lo > b) for (a, b) in slots[s]):
                        s += 1
                        continue
                    if any(ps > s and any(lo < j < hi for j in pj) for (ps, pj) in placed):
                        s += 1
                        continue
                    break
                while s >= len(slots):
                    slots.append([])
                slots[s].append((lo, hi))
                placed.append((s, it["jogs"]))
                it["slot"], it["sign"] = s, sign
            if len(slots) > cap:
                raise SystemExit(f"column at x={xcol} on {layer}: {len(slots)} {side}side slots needed, {cap} fit")
        for it in items:
            xt = it["x"] + it["sign"] * (1.2 + STUB_PITCH * it["slot"])
            y0 = it["jogs"][0]
            outside = it["sign"] == it["col"] and it["via"]
            if outside:
                # outside stubs run on the top layer, so that control pins on
                # the same column can jog outward on the bottom layer to the
                # control band without crossing them; they drop to the bottom
                # layer at their band's edge
                if it["end"] > y0:
                    y_edge = DBUS_Y0 - 1.5 - (1.0 if it["slot"] % 2 else 0.0)
                else:
                    y_edge = ABUS_Y0 + 1.5 + (1.0 if it["slot"] % 2 else 0.0)
                run("trace", "add", "--layer", "F.Cu", "--net", it["net"], "--width", STUB_WIDTH, f"{it['x']},{y0}", f"{xt:.3f},{y0}", f"{xt:.3f},{y_edge}", quiet=True)
                run("via", "add", f"{xt:.3f},{y_edge}", "--net", it["net"], "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                run("trace", "add", "--layer", "B.Cu", "--net", it["net"], "--width", STUB_WIDTH, f"{xt:.3f},{y_edge}", f"{xt:.3f},{it['end']}", quiet=True)
            else:
                pts = [f"{it['x']},{y0}", f"{xt:.3f},{y0}", f"{xt:.3f},{it['end']}"]
                if not it["via"]:
                    pts.append(f"{it['x']},{it['end']}")
                run("trace", "add", "--layer", layer, "--net", it["net"], "--width", STUB_WIDTH, *pts, quiet=True)
            if it["via"]:
                run("via", "add", f"{xt:.3f},{it['end']}", "--net", it["net"], "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                VIA_XS.setdefault(it["net"], []).append(round(xt, 3))


def draw_minibuses(chips):
    """The far connections of every DIP as vertical lines in the gap on
    the side of the pin's own column: the upper chip's address pins and
    every chip's control pins run down on the bottom layer to a via on
    their line; the lower chip's data pins run up on the top layer to
    the data band's edge, then a via and a short bottom-layer run to the
    via on their line.  Lines of one gap side and layer are pooled over
    the stack, the pin nearest its band by position taking the innermost
    line, so no jog crosses another line."""
    pools = {}   # (column x, col, layer) -> items
    for c in chips:
        if not c["package"].startswith("DIP"):
            continue
        pads = dip_pads(c)
        upper = PLACE[c["name"]][1] > (ROW_A + ROW_B) / 2
        for p in c["pins"]:
            net = p.get("net") or ""
            b = bus_of(net)
            if b is None:
                continue
            bus, i = b
            if bus != "CTRL" and (bus == "D") == upper:
                continue   # near pins have stubs
            x, y, col = pads[p["pin"]]
            up = bus == "D"
            pools.setdefault((round(x, 2), col, "F.Cu" if up else "B.Cu"), []).append({"net": net, "x": x, "y": y, "yb": bus_y(bus, i), "up": up})
    for (xcol, col, layer), items in pools.items():
        items.sort(key=lambda it: it["y"] if not it["up"] else -it["y"])
        off = MINI_X0[layer]
        for k, it in enumerate(items):
            if k and items[k - 1]["net"] == it["net"]:
                off += 0.15   # two vias of one net on one line: hole-to-hole wants 0.45
            xt = it["x"] + col * off
            off += STUB_PITCH
            if not it["up"]:
                run("trace", "add", "--layer", "B.Cu", "--net", it["net"], "--width", STUB_WIDTH, f"{it['x']},{it['y']}", f"{xt:.3f},{it['y']}", f"{xt:.3f},{it['yb']}", quiet=True)
            else:
                y_edge = DBUS_Y0 - 1.5 - (1.0 if k % 2 else 0.0)   # two rows of edge vias: 0.55 mm apart in x would touch
                run("trace", "add", "--layer", "F.Cu", "--net", it["net"], "--width", STUB_WIDTH, f"{it['x']},{it['y']}", f"{xt:.3f},{it['y']}", f"{xt:.3f},{y_edge}", quiet=True)
                run("via", "add", f"{xt:.3f},{y_edge}", "--net", it["net"], "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                run("trace", "add", "--layer", "B.Cu", "--net", it["net"], "--width", STUB_WIDTH, f"{xt:.3f},{y_edge}", f"{xt:.3f},{it['yb']}", quiet=True)
            run("via", "add", f"{xt:.3f},{it['yb']}", "--net", it["net"], "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
            VIA_XS.setdefault(it["net"], []).append(round(xt, 3))


def quad_pads(chip):
    """Board coordinates of a placed LQFP-48's pads: {pin: (x, y, side)},
    side in L B R T; rotation 0 only."""
    x0, y0, rot = PLACE[chip["name"]]
    assert rot == 0
    n_side, pitch, center = 12, 0.5, 4.35
    span = (n_side - 1) / 2 * pitch
    out = {}
    k = 1
    for i in range(n_side):
        out[k] = (x0 - center, y0 + span - i * pitch, "L"); k += 1
    for i in range(n_side):
        out[k] = (x0 - span + i * pitch, y0 - center, "B"); k += 1
    for i in range(n_side):
        out[k] = (x0 + center, y0 - span + i * pitch, "R"); k += 1
    for i in range(n_side):
        out[k] = (x0 + span - i * pitch, y0 + center, "T"); k += 1
    return out


ESCAPED = set()   # "chip-pin" of SMD pads that reach the router only through their escape via
ESC_VIAS = {}     # net -> [(x, y)] of those vias: one-pin parts in the router's DSN


def draw_smd_stubs(chips, nets):
    """Every netted pad of the LQFP escapes straight out on the top layer
    (the centre pad of a side furthest, the outer ones least, so a jog
    outward to the pad's own column crosses only escapes that have
    ended) to a small plated hole in a row at 1.0 mm pitch.  The hole is
    the pad as far as the router is concerned.  Bus pads continue on the
    bottom layer to a via on their bus line; power pads are reached by
    the planes; the rest the router picks up at the hole."""
    ESC, STEP, VPITCH = 1.3, 0.6, 1.0
    for c in chips:
        if not c["package"].startswith("LQFP"):
            continue
        pads = quad_pads(c)
        groups = {}
        inward = {}     # side -> the inward strip's coordinate (y for T/B, x for L/R)
        side_vias = {}  # net -> {side: [(x, y)]}
        for p in c["pins"]:
            net = p.get("net") or ""
            if not net or len(nets.get(net, [])) < 2:
                continue   # no such net in this stage
            b = bus_of(net)
            x, y, side = pads[p["pin"]]
            groups.setdefault(side, []).append({"net": net, "pin": p["pin"], "x": x, "y": y, "yb": bus_y(*b) if b else None})
        for side, items in groups.items():
            if side == "T": to_xy = lambda u, v: (u, v)
            elif side == "B": to_xy = lambda u, v: (u, -v)
            elif side == "L": to_xy = lambda u, v: (-v, u)
            else: to_xy = lambda u, v: (v, u)
            def uv(x, y):
                return {"T": (x, y), "B": (x, -y), "L": (y, -x), "R": (y, x)}[side]
            items.sort(key=lambda it: uv(it["x"], it["y"])[0])
            mid = (len(items) - 1) / 2
            v0 = uv(items[0]["x"], items[0]["y"])[1]
            v_top = v0 + ESC + (mid + 0.5) * STEP + 0.7
            inward[side] = to_xy(0, v_top - 0.8)[1] if side in "TB" else to_xy(0, v_top - 0.8)[0]
            for k, it in enumerate(items):
                u, v = uv(it["x"], it["y"])
                v_turn = v + ESC + (mid - abs(k - mid)) * STEP
                u_via = u + (k - mid) * (VPITCH - 0.5)
                pts = [to_xy(u, v), to_xy(u, v_turn), to_xy(u_via, v_turn), to_xy(u_via, v_top)]
                run("trace", "add", "--layer", "F.Cu", "--net", it["net"], "--width", STUB_WIDTH, *[f"{x:.3f},{y:.3f}" for x, y in pts], quiet=True)
                xv, yv = to_xy(u_via, v_top)
                run("via", "add", f"{xv:.3f},{yv:.3f}", "--net", it["net"], "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                ESCAPED.add(f"{c['name']}-{it['pin']}")
                ESC_VIAS.setdefault(it["net"], []).append((xv, yv))
                side_vias.setdefault(it["net"], {}).setdefault(side, []).append((xv, yv))
                it["via"] = (xv, yv)
            # a net with several pads on this side (the DTR# loop, the baud
            # clock): join its escape vias on the bottom layer, inward of the
            # via row where nothing else runs
            by_net = {}
            for it in items:
                by_net.setdefault(it["net"], []).append(it)
            strip = 0
            for net, its in by_net.items():
                if len(its) < 2 or net in ("VCC", "GND"):
                    continue   # the planes join power pads
                its.sort(key=lambda it: uv(*it["via"])[0])
                v_in = v_top - 0.8 - 0.6 * strip   # one strip per loop on this side
                strip += 1
                pts = [to_xy(uv(*its[0]["via"])[0], v_top), to_xy(uv(*its[0]["via"])[0], v_in), to_xy(uv(*its[-1]["via"])[0], v_in), to_xy(uv(*its[-1]["via"])[0], v_top)]
                run("trace", "add", "--layer", "B.Cu", "--net", net, "--width", STUB_WIDTH, *[f"{x:.3f},{y:.3f}" for x, y in pts], quiet=True)
                for it in its[1:-1]:
                    u_it = uv(*it["via"])[0]
                    run("trace", "add", "--layer", "B.Cu", "--net", net, "--width", STUB_WIDTH, *[f"{x:.3f},{y:.3f}" for x, y in (to_xy(u_it, v_top), to_xy(u_it, v_in))], quiet=True)
            bus_items = [it for it in items if it["yb"] is not None]
            if side in "TB":
                for it in bus_items:
                    xv, yv = it["via"]
                    run("trace", "add", "--layer", "B.Cu", "--net", it["net"], "--width", STUB_WIDTH, f"{xv:.3f},{yv:.3f}", f"{xv:.3f},{it['yb']}", quiet=True)
                    run("via", "add", f"{xv:.3f},{it['yb']}", "--net", it["net"], "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                    VIA_XS.setdefault(it["net"], []).append(round(xv, 3))
            else:
                dx = -1 if side == "L" else 1
                ups = sorted([it for it in bus_items if it["yb"] > it["y"]], key=lambda it: -it["y"])
                downs = sorted([it for it in bus_items if it["yb"] < it["y"]], key=lambda it: it["y"])
                for group in (ups, downs):
                    for j, it in enumerate(group):
                        xv, yv = it["via"]
                        xt = xv + dx * VPITCH * (j + 1)
                        run("trace", "add", "--layer", "B.Cu", "--net", it["net"], "--width", STUB_WIDTH, f"{xv:.3f},{yv:.3f}", f"{xt:.3f},{yv:.3f}", f"{xt:.3f},{it['yb']}", quiet=True)
                        run("via", "add", f"{xt:.3f},{it['yb']}", "--net", it["net"], "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                        VIA_XS.setdefault(it["net"], []).append(round(xt, 3))
        draw_corner_loops(side_vias, inward)


def draw_corner_loops(side_vias, inward):
    """A net with escape vias on two adjacent sides of the LQFP: join one
    via of each side on the bottom layer along the inward strips, round
    the corner between them."""
    for net, sides in side_vias.items():
        if net in ("VCC", "GND"):
            continue
        names = list(sides)
        for a in range(len(names)):
            for b in range(a + 1, len(names)):
                s1, s2 = names[a], names[b]
                if {s1, s2} in ({"T", "B"}, {"L", "R"}):
                    continue   # opposite sides: left to the router
                tb, lr = (s1, s2) if s1 in "TB" else (s2, s1)
                (xt, yt) = sides[tb][-1] if lr == "R" else sides[tb][0]
                (xl, yl) = sides[lr][-1] if tb == "T" else sides[lr][0]
                y_in, x_in = inward[tb], inward[lr]
                pts = [(xt, yt), (xt, y_in), (x_in, y_in), (x_in, yl), (xl, yl)]
                run("trace", "add", "--layer", "B.Cu", "--net", net, "--width", STUB_WIDTH, *[f"{x:.3f},{y:.3f}" for x, y in pts], quiet=True)


def draw_header_stubs(chips):
    """Analyzer-header pins on a bus net: a bottom-layer track from the
    pin to a via on its bus line.  The header is two rows of pins at the
    same x, so the pin in the row nearer the bus goes straight and the
    other jogs 1.27 mm sideways first."""
    for c in chips:
        if not c["part"].startswith("Header 2x10"):
            continue
        x0, y0, rot = PLACE[c["name"]]
        assert rot == 90
        for p in c["pins"]:
            net = p.get("net") or ""
            bus = "D" if net.startswith("D") and net[1:].isdigit() else "ADDR" if net.startswith("ADDR") else None
            if bus is None:
                continue
            k = p["pin"] - 1
            col, row = k // 2, k % 2          # footprint: odd pins row 0, even pins row 1
            x, y = x0 + col * 2.54, y0 + row * 2.54
            yb = bus_y(bus, int(net[len(bus):]))
            up = yb > y
            straight = (row == 1) == up       # the pin nearer the bus goes straight
            xt = x if straight else x + 1.27
            pts = [f"{x:.3f},{y:.3f}"] + ([] if straight else [f"{xt:.3f},{y:.3f}"]) + [f"{xt:.3f},{yb}"]
            run("trace", "add", "--layer", "B.Cu", "--net", net, "--width", STUB_WIDTH, *pts, quiet=True)
            run("via", "add", f"{xt:.3f},{yb}", "--net", net, "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
            VIA_XS.setdefault(net, []).append(round(xt, 3))


def component_pads(comp_name):
    """{pin name: [(x, y, is_smd)]} of a library component, from its files."""
    import glob
    for d in (os.path.join(LIB, "components"), os.path.join(JLC, "components")):
        path = os.path.join(d, comp_name + ".json")
        if os.path.exists(path):
            break
    else:
        raise SystemExit(f"no component file for {comp_name}")
    comp = json.load(open(path))
    fp = json.load(open(os.path.normpath(os.path.join(os.path.dirname(path), os.path.expanduser(comp["footprint"]["url"])))))
    pads = {pd["name"]: (pd["at"][0], pd["at"][1], pd["type"] == "smd") for pd in fp["pads"]}
    out = {}
    for pin in comp["pins"]:
        names = pin.get("pad", pin["name"])
        if isinstance(names, str):
            names = [names]
        out[pin["name"]] = [pads[nm] for nm in names]
    return out


def draw_power_vias(chips, pinmaps, comps, extras=()):
    """Every SMD pad on 5 V, ground or a bus net: a short escape straight
    away from the part's centre, then a via; a bus pad continues on the
    bottom layer to a via on its bus line.  (The router never adds plane
    vias, and these nets are hidden from it as finished.)"""
    import math
    for c in chips:
        name = c["name"]
        x0, y0, rot = PLACE[name]
        pads = component_pads(comps[name])
        th = math.radians(rot)
        wanted = [(pinmaps[name].get(str(p["pin"])), p.get("net") or "") for p in c["pins"]]
        wanted += [(pin, net) for (chip, pin, net) in extras if chip == name]
        for pin, net in wanted:
            b = bus_of(net)
            bus = b[0] if b else None
            if net not in ("VCC", "GND") and bus is None:
                continue
            if pin is None:
                continue
            for (px, py, smd) in pads[pin]:
                if not smd:
                    continue
                rx, ry = px * math.cos(th) - py * math.sin(th), px * math.sin(th) + py * math.cos(th)
                x, y = x0 + rx, y0 + ry
                L = math.hypot(rx, ry) or 1.0
                ux, uy = rx / L, ry / L
                if abs(ux) >= abs(uy):
                    ux, uy = (1.0 if ux > 0 else -1.0), 0.0
                else:
                    ux, uy = 0.0, (1.0 if uy > 0 else -1.0)
                xe, ye = x + ux * 1.0, y + uy * 1.0
                run("trace", "add", "--layer", "F.Cu", "--net", net, "--width", STUB_WIDTH, f"{x:.3f},{y:.3f}", f"{xe:.3f},{ye:.3f}", quiet=True)
                run("via", "add", f"{xe:.3f},{ye:.3f}", "--net", net, "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                if bus is not None:
                    yb = bus_y(bus, b[1])
                    run("trace", "add", "--layer", "B.Cu", "--net", net, "--width", STUB_WIDTH, f"{xe:.3f},{ye:.3f}", f"{xe:.3f},{yb}", quiet=True)
                    run("via", "add", f"{xe:.3f},{yb}", "--net", net, "--drill", VIA_DRILL, "--diameter", VIA_DIA, quiet=True)
                    VIA_XS.setdefault(net, []).append(round(xe, 3))


def draw_buses(nets):
    """The explicit bus wires: one protected trace per bit, only for bits
    that have pins in this stage."""
    for name, count in (("D", 16), ("ADDR", 16), ("CTRL", len(CTRL_NETS))):
        for i in range(count):
            net = f"{name}{i}" if name != "CTRL" else CTRL_NETS[i]
            if net not in nets or len(nets[net]) < 2:
                continue
            y = bus_y(name, i)
            # a vertex at every via, so the router's connectivity sees the junctions
            # taps: a plated hole on the line every TAP_DX, adjacent lines
            # staggered by half that so the rings clear each other.  The
            # router only connects pins to pins, never to a wire, so the
            # taps are what let it reach the bus; they are probe points too.
            if name == "CTRL":   # 0.8 mm pitch: smaller rings, three columns
                x_hole, drill, dia = BUS_X0 - 1.4 * (i % 3), 0.5, 0.9
            else:
                x_hole, drill, dia = BUS_X0 - (1.6 if i % 2 else 0.0), 0.6, 1.1
            taps = [x_hole]   # only the probe hole at the left end; the gaps hold the mini-buses
            xs = sorted(set(taps + [x for x in VIA_XS.get(net, []) if BUS_X0 < x < BUS_X1])) + [BUS_X1]
            run("trace", "add", "--layer", "F.Cu", "--net", net, "--width", BUS_WIDTH, *[f"{x},{y}" for x in xs], quiet=True)
            # a probe point at the left end of every line: a plated hole on
            # the net, which is also the pin that tells freerouting the wire
            # belongs to something (a protected wire attached to no pin makes
            # it hang)
            run("hole", "add", f"{x_hole},{y}", "--drill", drill, "--diameter", dia, "--net", net, quiet=True)


def hide_hand_wiring(dsn):
    """Rewrite the DSN so freerouting never sees the hand-wired bus nets:
    their nets are dropped from the network (their pins become plain
    obstacles) and every protected wire and via becomes a keepout.
    Freerouting otherwise treats protected wiring it cannot 'complete' as
    unrouted work, and either hangs or stops placing vias."""
    import re, math
    s = open(dsn).read()
    st = json.loads(subprocess.run([PCB, "-p", os.path.join(HERE, "pcb.json"), "status", "--json"], capture_output=True, text=True).stdout)
    done = {n["name"] for n in st["nets"] if len(n["islands"]) <= 1}
    not_done = sorted(set(VIA_XS) - done)
    if not_done:
        print("hand-wired but not complete:", not_done)
    # the escape vias of nets the router still has to finish become
    # one-pin parts ESC<n>, the way flatland exports netted holes
    esc_parts = {}   # net -> [part names]
    comps, images = [], []
    n = 0
    for net, pts in ESC_VIAS.items():
        if net in done:
            continue
        for (x, y) in pts:
            name = f"ESC{n}"
            n += 1
            esc_parts.setdefault(net, []).append(name)
            comps.append(f"    (component {name}\n      (place {name} {x * 1000:.0f} {y * 1000:.0f} front 0 (lock_type position))\n    )")
            images.append(f"    (image {name}\n      (pin Round[A]Pad_600_um 1 0 0)\n    )")
    if comps:
        i = s.index("  (placement\n") + len("  (placement\n")
        s = s[:i] + "\n".join(comps) + "\n" + s[i:]
        i = s.index("  (library\n") + len("  (library\n")
        pad = "    (padstack Round[A]Pad_600_um\n" + "".join(f"      (shape (circle {l} 600))\n" for l in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")) + "      (attach off)\n    )\n"
        s = s[:i] + "\n".join(images) + "\n" + pad + s[i:]
    # drop the completed nets from the network section; in the others,
    # a pad that escapes to a via is represented by the ESC part instead
    def net_line(m):
        name = m.group(1).strip('"')
        if name in done:
            return ""
        pins = [t for t in m.group(2).split() if t.strip('"') not in ESCAPED]
        pins += [f"{e}-1" for e in esc_parts.get(name, [])]
        return f"\n    (net {m.group(1)} (pins {' '.join(pins)}))"
    s = re.sub(r'\n\s*\(net (\S+) \(pins([^)]*)\)\)', net_line, s)
    # and from the classes
    def strip_class(m):
        names = [n for n in m.group(2).split() if n.strip('"') not in done]
        return m.group(1) + " ".join(names) + m.group(3)
    s = re.sub(r'(\(class \S+ )([^()\n]*)(\n)', strip_class, s)
    # protected wiring -> keepouts
    clearance = 0.15 * 1000
    keepouts = []
    wiring = re.search(r'\n  \(wiring\n(.*?)\n  \)\n', s, re.S)
    keep = []   # protected wiring of nets the router still has to finish
    if wiring:
        for line in wiring.group(1).split("\n"):
            m = re.search(r'\(net (\S+)\)', line)
            if m and m.group(1).strip('"') not in done:
                keep.append(line)
        for layer, w, pts, net in re.findall(r'\(wire \(path (\S+) (\d+)((?: -?[\d.]+)+)\) \(net (\S+)\)', wiring.group(1)):
            if net.strip('"') not in done:
                continue
            nums = [float(v) for v in pts.split()]
            h = float(w) / 2 + clearance
            for (x1, y1, x2, y2) in zip(nums[0::2], nums[1::2], nums[2::2], nums[3::2]):
                dx, dy = x2 - x1, y2 - y1
                L = math.hypot(dx, dy) or 1.0
                ux, uy = dx / L, dy / L
                px, py = -uy * h, ux * h
                ex, ey = ux * h, uy * h
                corners = [(x1 - ex + px, y1 - ey + py), (x2 + ex + px, y2 + ey + py), (x2 + ex - px, y2 + ey - py), (x1 - ex - px, y1 - ey - py)]
                keepouts.append(f'    (keepout "" (polygon {layer} 0 ' + " ".join(f"{x:.1f} {y:.1f}" for x, y in corners) + "))")
        esc_pts = {(round(x * 1000), round(y * 1000)) for net, pts in ESC_VIAS.items() if net not in done for (x, y) in pts}
        for x, y, net in re.findall(r'\(via \S+ (-?[\d.]+) (-?[\d.]+) \(net (\S+)\)', wiring.group(1)):
            if (round(float(x)), round(float(y))) in esc_pts or net.strip('"') not in done:
                continue
            r = 300 + clearance
            for layer in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu"):
                pts = " ".join(f"{float(x) + r * math.cos(a):.1f} {float(y) + r * math.sin(a):.1f}" for a in [i * math.pi / 6 for i in range(12)])
                keepouts.append(f'    (keepout "" (polygon {layer} 0 {pts}))')
        s = s[:wiring.start()] + "\n" + ("  (wiring\n" + "\n".join(keep) + "\n  )\n" if keep else "") + s[wiring.end():]
    # keepouts go into the structure section, after the boundary
    i = s.index("(boundary")
    j = s.index("\n", i)
    s = s[:j + 1] + "\n".join(keepouts) + "\n" + s[j + 1:]
    open(dsn, "w").write(s)


# --------------------------------------------------------------------- main
def main():
    finish_floorplan()
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
    run("rules", "set", "trace_width=0.25", "clearance=0.15", f"via_drill={VIA_DRILL}", f"via_diameter={VIA_DIA}", "pour_clearance=0.3", "edge_clearance=0.5", quiet=True)
    run("rules", "class", "power", "--width", "0.6", "--clearance", "0.25", quiet=True)
    wanted = set(stage_names())
    if "jpwr0" in wanted:
        run("drc", "waive", "pth-annular-ring", "jpwr0", "j1", "--reason", "non-plated locating pegs of the jack and mounting holes of the DB9 carry no copper by design", quiet=True)
        run("drc", "waive", "pth-annular-ring-recommended", "jpwr0", "j1", "--reason", "same: non-plated holes", quiet=True, check=False)

    wanted = set(stage_names())
    if "--no-uart" in sys.argv:
        wanted.discard("uart0")
    only = next((a.split("=")[1] for a in sys.argv if a.startswith("--only=")), None)
    if only:
        wanted = set(only.split(","))
    chips = [c for c in BOARD["chips"] if c["name"] in wanted]
    pinmaps = {}
    comps = {}
    for c in chips:
        comp_name, params, pinmap = component_for(c)
        pinmaps[c["name"]] = pinmap
        comps[c["name"]] = comp_name
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
    extras = []
    if "x1" in wanted:
        nets["GND"] += ["x1.2", "x1.4"]
        extras += [("x1", "2", "GND"), ("x1", "4", "GND")]
    if "jpwr0" in wanted:
        nets["GND"] += ["jpwr0.SW"]
        extras += [("jpwr0", "SW", "GND")]
    for net, pins in nets.items():
        if len(pins) < 2:
            print("single-pin net", net, pins)
            continue
        run("connect", *pins, "--net", net, quiet=True)
    for net in ("VCC", "GND", "VIN", "VF"):
        if net in nets and len(nets[net]) >= 2:
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
    if "--no-stubs" not in sys.argv:
        if not NO_STUBS:
            draw_stubs(chips)
            draw_minibuses(chips)
            draw_header_stubs(chips)
        draw_smd_stubs(chips, nets)
        draw_power_vias([c for c in chips if not c["package"].startswith(("DIP", "LQFP"))], pinmaps, comps, extras)
        draw_buses(nets)
    if FOUR:
        run("pour", "new", "gnd", "--layer", "In1.Cu", "--net", "GND", "--follow-outline", quiet=True)
        run("pour", "new", "vcc", "--layer", "In2.Cu", "--net", "VCC", "--follow-outline", quiet=True)
    else:
        run("pour", "new", "gnd", "--layer", "B.Cu", "--net", "GND", "--follow-outline", quiet=True)
        run("pour", "new", "vcc", "--layer", "F.Cu", "--net", "VCC", "--follow-outline", quiet=True)
    run("status", quiet=True)
    pre = run("check", check=False, quiet=True).stdout
    errs = [l for l in pre.splitlines() if l.startswith("error") and "nets-routed" not in l]
    if errs:
        print("hand wiring / placement errors before routing:")
        print("\n".join(errs[:12]))
        sys.exit(1)
    if ROUTE:
        # freerouting writes build/grit.ses; import it with the redundancy pass
        # skipped (that pass takes hours on a board of this size)
        run("visualize", "pcb", "-o", os.path.join(HERE, "build", f"stage{STAGE}-placed.png"), quiet=True)
        run("route", "--dsn-only", quiet=True)
        dsn = os.path.join(HERE, "build", "grit.dsn")
        # the LQFP's 0.5 mm pads sit exactly at the 0.2 mm clearance; give
        # the router's pad-to-pad and trace-to-pad classes some slack so it
        # is willing to leave those pads at all (the design check still
        # holds every trace to the project's 0.2 mm)
        s = open(dsn).read()
        s = s.replace("(clearance 200 (type default_smd))", "(clearance 150 (type default_smd))").replace("(clearance 200 (type smd_smd))", "(clearance 100 (type smd_smd))")
        open(dsn, "w").write(s)
        hide_hand_wiring(dsn)
        if FOUR:
            # the inner layers are planes: keep the router off them
            s = open(dsn).read()
            for layer in ("In1.Cu", "In2.Cu"):
                s = s.replace(f"(layer {layer} (type signal)", f"(layer {layer} (type power)")
            open(dsn, "w").write(s)
        passes = next((a.split("=")[1] for a in sys.argv if a.startswith("--passes=")), "6")
        ses = os.path.join(HERE, "build", "grit.ses")
        if os.path.exists(ses):
            os.remove(ses)
        with open(os.path.join(HERE, "build", "freerouting.log"), "w") as log:
            subprocess.run(["timeout", next((a.split("=")[1] for a in sys.argv if a.startswith("--timeout=")), "600"), FREEROUTING, "-de", dsn, "-do", ses, "-mp", passes, "-dct", "0", "-da", "-dl", "--gui.enabled=false"], stdout=log, stderr=subprocess.STDOUT, text=True, env={**os.environ, "JAVA_TOOL_OPTIONS": "-Djava.awt.headless=true"})
        run("route", "--import", os.path.join(HERE, "build", "grit.ses"), "--keep-redundant")
        run("check", check=False)
        run("visualize", "pcb", "-o", os.path.join(HERE, "build", f"stage{STAGE}-routed.png"), quiet=True)
    if "--outputs" in sys.argv:
        run("gerbers", check=False)
        run("bom", "--all", check=False)
        run("pnp", check=False)


if __name__ == "__main__":
    main()
