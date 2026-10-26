"""
SCPI driver for the OpenMagnetics relay switching board (rev B).

ARCHITECTURE
------------
Four DUT terminals A, B, C, D reach three internal rails through a full
crossbar: every terminal can be tied to HI, LO or LINK independently.

    HI    -- Bode 100 port, "hot" side
    LO    -- Bode 100 port, return side
    LINK  -- floating tie bus, for shorts and winding links that must NOT
             touch either measurement node

Twelve matrix relays (4 terminals x 3 rails) express every topology the
Cogitore/Blache method needs, and nothing is hard-wired into a fixed set of
"configurations" the way rev A was.  Two isolation relays are CHANGEOVERS:
de-energized they connect each terminal node to its clamp (the DUT); energized
they connect it to the calibration standards instead.  That is what makes OSL
automatic -- the DUT can stay clamped while an OPEN standard is presented --
and it keeps exactly one relay contact between the terminal node and either the
DUT or the standard, so the calibration path mirrors the measurement path.

Four standards relays switch a copper short or a 100 ohm 0.1% resistor across
each measurement pair.

Because calibration is automatic, every configuration can be calibrated in its
OWN relay state.  Rev A grouped 21 configurations onto 4 shared calibrations,
which meant relays inside a group changed the fixture between calibration and
measurement -- fine for inductance, not for picofarads.

The STM32 firmware lives in firmware/ in this repository.
"""

import pyvisa

# --------------------------------------------------------------------------
# Relay map -- must match firmware/src/relay_map.h
# --------------------------------------------------------------------------

TERMINALS = ("A", "B", "C", "D")
RAILS = ("HI", "LO", "LINK")

#: Shift-register bit for each matrix connection.
MATRIX_BITS = {
    ("A", "HI"): 0, ("A", "LO"): 1, ("A", "LINK"): 2,
    ("B", "HI"): 3, ("B", "LO"): 4, ("B", "LINK"): 5,
    ("C", "HI"): 6, ("C", "LO"): 7, ("C", "LINK"): 8,
    ("D", "HI"): 9, ("D", "LO"): 10, ("D", "LINK"): 11,
}
#: K13/K14: the LOAD-standard column -- one 100R 0.1% between two full-length
#: column buses, reached through the same contact structure as a terminal.
LOAD_HI_BIT = 12             # K13: CAL_E -> RAIL_HI
LOAD_LO_BIT = 13             # K14: CAL_F -> RAIL_LO
#: K15..K18: per-terminal isolation. COM = terminal node, NC = clamp,
#: NO = nothing (the open contact IS the open standard).
ISOLATE_BITS = {"A": 14, "B": 15, "C": 16, "D": 17}
RELAY_COUNT = 18

CALIBRATION_LOAD_OHM = 100.0


# --------------------------------------------------------------------------
# Measurement configurations
# --------------------------------------------------------------------------
#
# Each entry says which terminals go to which rail.  The signal path -- and so
# the calibration -- is fully determined by this dict, which is why the board
# no longer needs a hand-maintained CALIBRATION_GROUPS table that could drift
# out of step with the wiring.

def _config(number, name, hi, lo, link=(), purpose=""):
    return {
        "number": number,
        "name": name,
        "HI": tuple(hi),
        "LO": tuple(lo),
        "LINK": tuple(link),
        "purpose": purpose,
    }


CONFIGS = {c["number"]: c for c in [
    # --- the four canonical two-port impedances, [COG94] II-A -------------
    _config(1, "Z0", "A", "B", (),
            "primary, secondary open -- L0"),
    _config(2, "Zsc", "A", "B", ("C", "D"),
            "primary, secondary shorted -- Lsc"),
    _config(3, "Z0p", "C", "D", (),
            "secondary, primary open -- L0'"),
    _config(4, "Zscp", "C", "D", ("A", "B"),
            "secondary, primary shorted -- Lsc'. NEW in rev B: completes the "
            "set so Z0*Zsc' = Z0'*Zsc can verify the relays every run"),

    # --- mutual inductance cross-check ------------------------------------
    _config(5, "Lcum", "A", "D", ("B", "C"),
            "series aiding (B-C joined) -- M = (Lcum-Ldif)/4"),
    _config(6, "Ldif", "A", "C", ("B", "D"),
            "series opposing (B-D joined)"),

    # --- direct interwinding capacitance, [BLA94] II-D --------------------
    _config(7, "C33", ("A", "B"), ("C", "D"), (),
            "both windings shorted, measure between them -- C33 directly"),

    # --- winding links for the six-capacitance extraction -----------------
    # With the secondary SHORTED, linking B-D and B-C give the same topology,
    # as do A-C and A-D. [BLA94] Table 1 agrees: their C1+C3 columns are equal
    # in each pair. Rev A spent four configurations on what are two states.
    # B and D joined on LO. Firmware <= 1.1.1 put D alone on LINK here, which
    # never joined B-D: the sweep was a floating-secondary Z0 plus the LINK
    # rail's capacitance. The driver now drives every relay itself.
    _config(8, "link_BD_open", "A", ("B", "D"), (),
            "B-D linked, secondary open"),
    _config(9, "link_BD_short", "A", ("B", "C", "D"), (),
            "B-D (or B-C) linked, secondary shorted"),
    _config(10, "link_AC_open", ("A", "C"), "B", (),
            "A-C linked, secondary open"),
    _config(11, "link_AC_short", ("A", "C", "D"), "B", (),
            "A-C (or A-D) linked, secondary shorted"),
    _config(12, "link_BC_open", "A", ("B", "C"), (),
            "B-C linked, secondary open"),
    _config(13, "link_AD_open", ("A", "D"), "B", (),
            "A-D linked, secondary open"),

    # --- secondary-side mirrors -------------------------------------------
    # [COG94] III: "Among these four measurements, the least accurate is
    # eliminated ... some extra measurements often allow useful checkings."
    # Measuring the same link from the other winding gives a redundant
    # equation whose accuracy is better whenever the secondary impedance sits
    # in a friendlier part of the analyzer's range.
    _config(14, "link_BD_short_sec", "C", ("A", "B", "D"), (),
            "B-D linked and primary shorted, measured from the secondary "
            "-- redundant with config 9"),
    _config(15, "link_AC_short_sec", ("A", "B", "C"), "D", (),
            "A-C linked and primary shorted, measured from the secondary "
            "-- redundant with config 11"),

    # --- electrostatic-only states: each winding shorted on itself --------
    # No magnetics at all (V1 = V2 = 0), so these are clean absolute
    # capacitances. The Bode LO side is ground: together with config 7 they
    # separate the interwinding capacitance from each winding's capacitance
    # to ground, which the ground-free [BLA94] model does not have and which
    # otherwise biases every link sweep. Driven by the Python relay table
    # (firmware <= 1.2 has no CONF:MEAS entry for 16-19).
    _config(16, "C33_rev", ("C", "D"), ("A", "B"), (),
            "config 7 reversed: interwinding + primary-to-ground"),
    _config(17, "Cground", ("A", "B", "C", "D"), (), (),
            "whole DUT to ground (LO rail empty)"),
    _config(18, "Cground_P", ("A", "B"), (), ("C", "D"),
            "primary to ground, secondary floating shorted"),
    _config(19, "Cground_S", ("C", "D"), (), ("A", "B"),
            "secondary to ground, primary floating shorted"),
]}

#: Which winding-link case in TransformerModel.WINDING_LINKS each pair of
#: configurations feeds.  "floating" reuses Z0/Zsc, which is correct: with no
#: link the measurement is identical and only the interpretation differs.
LINK_CONFIGS = {
    "B-D": {"open": 8, "short": 9},
    "A-C": {"open": 10, "short": 11},
    "B-C": {"open": 12, "short": 9},
    "A-D": {"open": 13, "short": 11},
    "floating": {"open": 1, "short": 2},
}

def signal_path(config):
    """Stable key identifying the fixture state, used to name calibrations."""
    hi = "".join(sorted(config["HI"]))
    lo = "".join(sorted(config["LO"]))
    link = "".join(sorted(config["LINK"]))
    return f"{hi}-{lo}" + (f"-L{link}" if link else "")


def floating_terminals(config):
    """Terminals on no rail. Their clamp must not drag the crossbar along."""
    assigned = set(config["HI"]) | set(config["LO"]) | set(config["LINK"])
    return tuple(t for t in TERMINALS if t not in assigned)


def measurement_word(config):
    """Relay word for a configuration in MEAS mode.

    Matrix bit for every (terminal, rail) assignment, plus the isolation
    relay of every FLOATING terminal. Without that, a terminal meant to float
    stays tied to its column bus, whose open matrix contacts and copper load
    the DUT node with a few pF to HI and to LO/GND. The OSL never sees that
    capacitance -- calibration isolates every terminal -- so it lands in the
    result: on a 1:1 transformer it put ~15 pF of phantom C33 into the
    B-C/A-D link sweeps. Isolating floating terminals makes the measurement
    state differ from the OPEN standard only by the DUT itself.
    """
    word = 0
    for rail in RAILS:
        for terminal in config[rail]:
            word |= 1 << MATRIX_BITS[(terminal, rail)]
    for terminal in floating_terminals(config):
        word |= 1 << ISOLATE_BITS[terminal]
    return word


def calibration_overlay(config, mode):
    """Extra relay bits to energize on top of a configuration for one standard.

    OPEN   isolate all four terminals; the matrix stays in the measurement
           state, so the calibration sees the exact fixture it will correct.
    SHORT  isolate, then ALSO close the LO crossbar relay of the first HI
           terminal: that terminal's bus then bridges RAIL_HI to RAIL_LO --
           a short presented through the same contact plane as the DUT.
    LOAD   isolate, then close K13+K14: the 100R column bridges the rails
           through one contact and a full column bus per side, mimicking a
           two-terminal DUT (Keysight impedance handbook: measure the load
           "in the same way as the DUT will be measured").
    """
    mode = mode.upper()
    if mode == "MEAS":
        return 0
    overlay = sum(1 << bit for bit in ISOLATE_BITS.values())
    if mode == "SHORT":
        first_hi = config["HI"][0]
        overlay |= 1 << MATRIX_BITS[(first_hi, "LO")]
    elif mode == "LOAD":
        overlay |= (1 << LOAD_HI_BIT) | (1 << LOAD_LO_BIT)
    elif mode != "OPEN":
        raise ValueError(f"Unknown calibration mode {mode!r}")
    return overlay


class RelayBoardController:
    IDN_PREFIX = "OpenMagnetics,RelayBoard"

    def __init__(self, com_port=None, baud_rate=115200, timeout_ms=5000):
        self.baud_rate = baud_rate
        self.timeout_ms = timeout_ms
        self.resource_string = com_port or self._auto_detect()
        self.visa_session = self._connect(self.resource_string)
        self.identity = self.visa_session.query("*IDN?").strip()
        # Board revision from *IDN? ("OpenMagnetics,RelayBoard,revC,2.0.0"):
        # rev C has the Kelvin crossbar and its own relay map (RelayMapRevC).
        fields = self.identity.split(",")
        self.revision = fields[2].strip() if len(fields) > 2 else "revB"
        if self.revision == "revC":
            import RelayMapRevC
            self.relay_count = RelayMapRevC.RELAY_COUNT
        else:
            self.relay_count = RELAY_COUNT
        self.kelvin = False
        self.current_config = None
        self.current_path = None
        self.cal_mode = "MEAS"
        self.board_serial = self.serial()

    # ------------------------------------------------------------ transport

    RECONNECT_ATTEMPTS = 15             # x 1 s: re-enumeration takes 1-3 s

    def _reconnect(self):
        """Reopen the board after a USB drop (an ESD hit or a watchdog reset
        re-enumerates it: EN 61326-1 criterion B wants recovery without the
        user). Firmware >= 2.0.0 restores its relay word after a watchdog
        reset; the caller re-drives the state it wanted regardless. Refuses a
        different board (serial mismatch)."""
        import time
        try:
            self.visa_session.close()
        except Exception:
            pass
        last_error = None
        for _ in range(self.RECONNECT_ATTEMPTS):
            time.sleep(1.0)
            try:
                try:
                    session = self._connect(self.resource_string)
                except Exception:
                    self.resource_string = self._auto_detect()   # COM number moved
                    session = self._connect(self.resource_string)
            except Exception as error:
                last_error = error
                continue
            self.visa_session = session
            serial = self.serial()
            if self.board_serial and serial and serial != self.board_serial:
                raise RuntimeError(f"Reconnected to a different relay board ({serial}, "
                                   f"expected {self.board_serial})")
            print(f"Relay board reconnected at {self.resource_string}")
            return
        raise RuntimeError(f"Relay board did not come back after a USB drop: {last_error}")

    def _auto_detect(self):
        manager = pyvisa.ResourceManager()
        for resource in manager.list_resources():
            if "ASRL" not in resource:
                continue
            session = None
            try:
                session = manager.open_resource(resource)
                session.timeout = 2000
                session.read_termination = "\n"
                session.write_termination = "\n"
                session.baud_rate = self.baud_rate
                identity = session.query("*IDN?")
                if self.IDN_PREFIX in identity:
                    print(f"Found relay board at {resource}: {identity}")
                    return resource
            except Exception:
                continue
            finally:
                if session is not None:
                    try:
                        session.close()
                    except Exception:
                        pass
        raise RuntimeError(
            "Could not auto-detect the relay board. Check the USB cable and that "
            "the board enumerates as a serial port."
        )

    def _connect(self, resource_string):
        print(f"Connecting to relay board at {resource_string}")
        session = pyvisa.ResourceManager().open_resource(resource_string)
        session.timeout = self.timeout_ms
        session.read_termination = "\n"
        session.write_termination = "\n"
        session.baud_rate = self.baud_rate
        identity = session.query("*IDN?")
        if self.IDN_PREFIX not in identity:
            raise RuntimeError(f"Device at {resource_string} is not a relay board: {identity}")
        print(f"Connected: {identity}")
        return session

    def _command(self, text):
        self.visa_session.write(text)
        self.visa_session.query("*OPC?")
        error = self.visa_session.query("SYST:ERR?")
        if not error.strip().startswith("0"):
            raise RuntimeError(f"Relay board rejected {text!r}: {error.strip()}")

    # ------------------------------------------------- revision-aware words

    def _measurement_word(self, config):
        if self.revision == "revC":
            import RelayMapRevC
            return RelayMapRevC.measurement_word(config, kelvin=self.kelvin)
        return measurement_word(config)

    def _calibration_word(self, config, mode):
        if self.revision == "revC":
            import RelayMapRevC
            if mode.upper() == "MEAS":
                return RelayMapRevC.measurement_word(config, kelvin=self.kelvin)
            return RelayMapRevC.calibration_word(config, mode, kelvin=self.kelvin)
        return measurement_word(config) | calibration_overlay(config, mode)

    def set_kelvin(self, enabled):
        """Rev C only: sense rails to the front-end and CH1 on its output.

        Takes effect immediately on the current state. The Bode CH1 input
        should be set to 1 Mohm in Kelvin mode (simulate_frontend.py); the
        calibration names carry a -K suffix so 2-wire and Kelvin OSLs never
        mix."""
        if self.revision != "revC":
            raise RuntimeError(f"Kelvin mode needs a rev C board (this is {self.revision})")
        self.kelvin = bool(enabled)
        if self.current_config is not None:
            self.set_calibration_mode(self.cal_mode)

    # ----------------------------------------------------------- switching

    def set_config(self, config_number):
        if config_number not in CONFIGS:
            raise ValueError(f"Unknown config {config_number}; valid: {sorted(CONFIGS)}")
        config = CONFIGS[config_number]
        # Relays are driven one by one from the table above rather than with
        # CONF:MEAS, so the switching cannot depend on whichever relay table a
        # given firmware build carries (1.1.1 had config 8 wrong).
        self._apply_word(self._measurement_word(config))
        self.current_config = config_number
        self.current_path = signal_path(config)
        self.cal_mode = "MEAS"
        print(f"Config {config_number} ({config['name']}): "
              f"HI={'+'.join(config['HI'])} LO={'+'.join(config['LO'])}"
              + (f" LINK={'+'.join(config['LINK'])}" if config["LINK"] else ""))
        return config

    def get_config(self):
        return int(self.visa_session.query("CONF:MEAS?").strip())

    def set_calibration_mode(self, mode):
        """MEAS | OPEN | SHORT | LOAD, applied on top of the current config.

        OPEN/SHORT/LOAD energize the isolation relays, disconnecting the DUT and
        substituting the on-board standard -- so the DUT can stay clamped.
        """
        mode = mode.upper()
        allowed = ("MEAS", "OPEN", "SHORT", "LOAD")
        if self.revision == "revC":
            allowed += ("CHECK_R1", "CHECK_C10")     # on-board verification standards
        if mode not in allowed:
            raise ValueError(f"Bad calibration mode {mode!r} for {self.revision}")
        if self.current_config is None:
            raise RuntimeError("Set a measurement configuration before a calibration mode.")
        config = CONFIGS[self.current_config]
        self._apply_word(self._calibration_word(config, mode))
        self.cal_mode = mode
        return mode

    def calibration_name(self, config_number=None):
        """File-safe name of the calibration owned by a configuration."""
        config = CONFIGS[config_number if config_number is not None else self.current_config]
        if self.revision == "revC":
            suffix = "-K" if self.kelvin else ""
            return f"relay_board_revC_{signal_path(config)}{suffix}.mcalx"
        return f"relay_board_{signal_path(config)}.mcalx"

    def needs_recalibration(self, config_number):
        return signal_path(CONFIGS[config_number]) != self.current_path

    # ------------------------------------------------------------ low level

    def set_relay(self, relay_number, state):
        if not 0 <= relay_number < self.relay_count:
            raise ValueError(f"Relay must be 0..{self.relay_count - 1}, got {relay_number}")
        self._command(f"RELAY {relay_number} {1 if state else 0}")

    def _apply_word(self, word):
        """Drive the board to exactly `word`, then read it back.

        Relays being released go first so a transition never closes a new
        path while an old one is still made (no momentary HI-LO bridge).
        A USB drop on the way (pyvisa error) reconnects once and re-drives.
        """
        try:
            self._drive_word(word)
        except pyvisa.errors.VisaIOError:
            self._reconnect()
            self._drive_word(word)

    def _drive_word(self, word):
        current = self.get_relay_states()
        count = self.relay_count
        releases = [i for i in range(count) if current[i] and not (word >> i) & 1]
        energizes = [i for i in range(count) if not current[i] and (word >> i) & 1]
        for index in releases:
            self.set_relay(index, 0)
        for index in energizes:
            self.set_relay(index, 1)
        readback = self.get_relay_states()
        expected = [(word >> i) & 1 for i in range(count)]
        if readback != expected:
            raise RuntimeError(f"Relay board state {readback} does not match requested {expected}")

    # ------------------------------------------------ firmware >= 2.0 extras

    def _optional_query(self, command):
        """Query that older firmware does not know: None instead of an error
        (firmware < 2.0.0 is not asked at all: it would never reply)."""
        try:
            version = tuple(int(v) for v in self.identity.split(",")[-1].strip().split("."))
        except ValueError:
            version = (0,)
        if version < (2, 0, 0):
            return None
        try:
            reply = self.visa_session.query(command).strip()
        except Exception:
            return None
        error = self.visa_session.query("SYST:ERR?").strip()
        return None if not error.startswith("0") else reply

    def serial(self):
        """MCU factory unique ID (24 hex digits): which physical board."""
        return self._optional_query("SYST:SER?")

    def temperature(self):
        """Fixture temperature in degC (rev C TMP117), or None."""
        reply = self._optional_query("MEAS:TEMP?")
        try:
            return float(reply)
        except (TypeError, ValueError):
            return None

    def relay_counts(self):
        """Lifetime actuation count per relay (K1 first), or None."""
        reply = self._optional_query("RELAY:COUNT?")
        return [int(v) for v in reply.split(",")] if reply else None

    def coil_supply(self):
        """Rev C coil switch: 1 on, 0 off, -1 overcurrent fault; None if absent."""
        reply = self._optional_query("SYST:COIL?")
        return int(reply) if reply not in (None, "") else None

    def get_relay_states(self):
        response = self.visa_session.query("RELAY:ALL?").strip()
        return [int(bit) for bit in response.split(",")]

    def self_test(self):
        """Firmware-side check: shift register readback and coil supply rail."""
        return self.visa_session.query("*TST?").strip()

    def enter_dfu(self):
        """Reboot the board into the STM32 ROM DFU bootloader (firmware >= 1.1.0).

        All relays are released and the serial port disappears; the board
        re-enumerates as "STM32 BOOTLOADER" (0483:DF11), ready for
        STM32_Programmer_CLI -c port=usb1 or dfu-util. No reply is sent.
        """
        self.visa_session.write("SYST:DFU")
        try:
            self.visa_session.close()
        except Exception:
            pass

    def reset(self):
        self._command("*RST")
        self.current_config = None
        self.current_path = None
        self.cal_mode = "MEAS"
        print("Relay board reset: all relays de-energized, DUT connected")

    def close(self):
        try:
            self.reset()
        finally:
            self.visa_session.close()


if __name__ == "__main__":
    print("=" * 70)
    print("Relay board self-test")
    print("=" * 70)
    controller = RelayBoardController()
    print(f"Firmware self-test: {controller.self_test()}")

    paths = {}
    for number in sorted(CONFIGS):
        config = controller.set_config(number)
        path = signal_path(config)
        paths.setdefault(path, []).append(number)
        print(f"    relays: {controller.get_relay_states()}")
        print(f"    calibration: {controller.calibration_name(number)}")
        print(f"    {config['purpose']}")

    print("\nDistinct signal paths (one calibration each):")
    for path, numbers in sorted(paths.items()):
        print(f"  {path:16s} <- configs {numbers}")

    print("\nCalibration overlays per config (relay bits added to the state):")
    for number, config in sorted(CONFIGS.items()):
        overlays = {m: calibration_overlay(config, m) for m in ("OPEN", "SHORT", "LOAD")}
        print(f"  config {number:2d} {config['name']:18s} "
              f"OPEN={overlays['OPEN']:05x} SHORT={overlays['SHORT']:05x} "
              f"LOAD={overlays['LOAD']:05x}")

    controller.close()
