import contextlib
import importlib.util
import io
import json
from pathlib import Path
import re
import subprocess
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("can_profile", ROOT / "tools/can_profile.py")
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)


def timings(path):
    return {name: int(value) for name, value in re.findall(
        r"(?:\.|->)Init\.(\w+)\s*=\s*(\d+)U?\s*;", path.read_text())}


class CanProfileTests(unittest.TestCase):
    def test_mcu_bit_times_match_raspberry_including_sjw(self):
        paths = [ROOT / f"Boards/{board}/Core/Src/fdcan.c" for board in ("g431", "g474")]
        paths += [ROOT / "Boot/fdcan.c"]
        for path in paths:
            text = path.read_text()
            self.assertRegex(text, r"ClockDivider\s*=\s*FDCAN_CLOCK_DIV1")
            self.assertRegex(text, r"AutoRetransmission\s*=\s*ENABLE")
            self.assertRegex(text, r"FrameFormat\s*=\s*FDCAN_FRAME_FD_BRS")
            fields = timings(path)
            for name, phase in (("nominal", "Nominal"), ("data", "Data")):
                pi = profile.phase_values(profile.PROFILES["1m4"][name])
                tq_ns = fields[phase + "Prescaler"] * 1e9 / 160_000_000
                tseg1, tseg2 = fields[phase + "TimeSeg1"], fields[phase + "TimeSeg2"]
                with self.subTest(source=str(path), phase=name):
                    self.assertEqual(1e9 / (tq_ns * (1 + tseg1 + tseg2)), pi["bitrate"])
                    self.assertEqual((1 + tseg1) / (1 + tseg1 + tseg2), pi["sample_point"])
                    self.assertEqual(tq_ns * fields[phase + "SyncJumpWidth"], pi["tq"] * pi["sjw"])
                    self.assertLessEqual(fields[phase + "SyncJumpWidth"], tseg2)
            # M_CAN TDC offset uses minimum CAN-clock quanta, not divided data TQ.
            self.assertEqual(fields["DataPrescaler"] * fields["DataTimeSeg1"], 30)
            self.assertLessEqual(fields["DataPrescaler"], 2)

    def test_clock_sources_are_160mhz(self):
        for board in ("g431", "g474"):
            main = (ROOT / f"Boards/{board}/Core/Src/main.c").read_text()
            hal = (ROOT / f"Boards/{board}/Core/Inc/stm32g4xx_hal_conf.h").read_text()
            self.assertRegex(hal, r"HSE_VALUE\s+\(8000000UL\)")
            for pattern in (r"PLLM\s*=\s*RCC_PLLM_DIV1", r"PLLN\s*=\s*40;", r"PLLQ\s*=\s*RCC_PLLQ_DIV2"):
                self.assertRegex(main, pattern)
            source = ROOT / ("Boards/g431/App/fdcan.cpp" if board == "g431" else "Boards/g474/Core/Src/fdcan.c")
            self.assertRegex(source.read_text(), r"FdcanClockSelection\s*=\s*RCC_FDCANCLKSOURCE_PLL")
        boot = (ROOT / "ThirdParty/VBBoot/Core/Src/main.c").read_text()
        for pattern in (r"PLLN\s*=\s*40;", r"PLLQ\s*=\s*RCC_PLLQ_DIV2", r"PLLR\s*=\s*RCC_PLLR_DIV2"):
            self.assertRegex(boot, pattern)

    def test_legacy_bootloader_request_abi(self):
        boot = (ROOT / "ThirdParty/VBBoot/App/app.c").read_text()
        for name, value in (("NODE_ID_MASK", "0x7FF"), ("NOMINAL_SHIFT", "11"),
                            ("DATA_SHIFT", "19"), ("FD_MODE_BIT", "27"), ("BITRATE_SWITCH_BIT", "28")):
            self.assertRegex(boot, rf"#define BOOT_CFG_{name}\s+{value}U?L?")
        old = (ROOT / "ThirdParty/VBBoot/App/state_manager.c").read_text()
        self.assertIn("FDCAN_CLOCK_DIV2", old)
        values = timings(ROOT / "ThirdParty/VBBoot/App/state_manager.c")
        self.assertEqual(80_000_000 // (1 + values["NominalTimeSeg1"] + values["NominalTimeSeg2"]), 1_000_000)
        self.assertEqual(80_000_000 // (2 * (1 + values["DataTimeSeg1"] + values["DataTimeSeg2"])), 4_000_000)
        assertions = "\n".join(
            f"static_assert(ruka3::protocol::boot_transport_1m4({node}) == {0x18100800 | node}U);"
            for node in range(101, 107))
        subprocess.run(["c++", "-std=c++17", "-fsyntax-only", "-x", "c++", "-I", str(ROOT / "App"), "-"],
                       input='#include "protocol.h"\n' + assertions, text=True, check=True)
        for path in ("Boards/g431/App/state_manager.cpp", "Boards/g431/App/bootstrap.cpp", "Boards/g474/App/bootstrap.cpp"):
            self.assertRegex((ROOT / path).read_text(), r"BKP1R\s*=\s*ruka3::protocol::boot_transport_1m4")

    def test_default_cli_never_calls_ip(self):
        with patch.object(profile.subprocess, "run") as run, patch.object(profile.subprocess, "check_output") as read:
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(profile.main([]), 0)
            run.assert_not_called()
            read.assert_not_called()
            self.assertIn("PREVIEW ONLY", output.getvalue())

    def test_exact_4m_commands_and_8m_rollback(self):
        for name, bitrate, point in (("1m4", 4_000_000, 0.8), ("1m8", 8_000_000, 0.6)):
            self.assertEqual(profile.phase_values(profile.PROFILES[name]["data"])["bitrate"], bitrate)
            self.assertEqual(profile.phase_values(profile.PROFILES[name]["data"])["sample_point"], point)
            command = profile.commands("can0", name)[1]
            self.assertNotIn("bitrate", command)  # Use exact quanta, not kernel timing estimation.
            self.assertIn("dtq", command)

    def test_controller_validation_and_active_profile(self):
        limits = {"name": "mcp251xfd", "tseg1": {"min": 2, "max": 256},
                  "tseg2": {"min": 1, "max": 128}, "sjw": {"min": 1, "max": 128},
                  "brp": {"min": 1, "max": 256}}
        link = {"flags": ["UP"], "txqlen": 65535, "linkinfo": {"info_kind": "can", "info_data": {
            "clock": 40_000_000, "bittiming_const": limits, "data_bittiming_const": limits,
            "ctrlmode": ["FD", "BERR-REPORTING"], "state": "ERROR-ACTIVE", "restart_ms": 1000,
            "bittiming": profile.phase_values(profile.PROFILES["1m4"]["nominal"]),
            "data_bittiming": profile.phase_values(profile.PROFILES["1m4"]["data"]),
        }}}
        self.assertEqual(profile.verify_active(link, "1m4"), "ERROR-ACTIVE")
        with self.assertRaises(ValueError):
            profile.verify_active(link, "1m8")
        link["linkinfo"]["info_data"]["clock"] = 20_000_000
        with self.assertRaises(ValueError):
            profile.validate_controller(link, "1m4")

    def test_apply_rejects_wrong_clock_before_taking_can_down(self):
        wrong_clock = {"linkinfo": {"info_kind": "can", "info_data": {"clock": 20_000_000}}}
        with patch.object(profile, "read_link", return_value=wrong_clock), \
                patch.object(profile.os, "geteuid", return_value=0), \
                patch.object(profile.subprocess, "run") as run:
            with self.assertRaises(ValueError):
                profile.main(["--apply"])
            run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
