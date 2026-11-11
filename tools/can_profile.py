#!/usr/bin/env python3
"""Prepare/apply exact MCP251xfd timings. Default mode only prints commands."""
import argparse
import json
import os
import shlex
import subprocess

CLOCK = 40_000_000
PROFILES = {
    "1m4": {"nominal": (25, 14, 15, 10, 1), "data": (25, 3, 4, 2, 2)},
    "1m8": {"nominal": (25, 14, 15, 10, 5), "data": (25, 1, 1, 2, 1)},
}


def phase_values(timing):
    tq, prop, phase1, phase2, sjw = timing
    quanta = 1 + prop + phase1 + phase2
    return {
        "bitrate": 1_000_000_000 // (tq * quanta),
        "sample_point": (1 + prop + phase1) / quanta,
        "tq": tq, "prop_seg": prop, "phase_seg1": phase1,
        "phase_seg2": phase2, "sjw": sjw, "brp": CLOCK * tq // 1_000_000_000,
    }


def commands(interface, profile):
    result = [["ip", "link", "set", "dev", interface, "down"]]
    timing = []
    for name, prefix in (("nominal", ""), ("data", "d")):
        values = PROFILES[profile][name]
        for option, value in zip(("tq", "prop-seg", "phase-seg1", "phase-seg2", "sjw"), values):
            timing += [prefix + option, str(value)]
    result += [
        ["ip", "link", "set", "dev", interface, "type", "can", *timing,
         "fd", "on", "fd-non-iso", "off", "loopback", "off", "listen-only", "off",
         "berr-reporting", "on", "restart-ms", "1000"],
        ["ip", "link", "set", "dev", interface, "txqueuelen", "65535"],
        ["ip", "link", "set", "dev", interface, "up"],
    ]
    return result


def read_link(interface):
    return json.loads(subprocess.check_output(
        ["ip", "-details", "-json", "link", "show", "dev", interface], text=True))[0]


def validate_controller(link, profile):
    if link["linkinfo"]["info_kind"] != "can":
        raise ValueError("Interface is not CAN")
    data = link["linkinfo"]["info_data"]
    if data["clock"] != CLOCK:
        raise ValueError(f"Expected 40 MHz CAN controller, got {data['clock']}")
    for name, key in (("nominal", "bittiming_const"), ("data", "data_bittiming_const")):
        limits = data[key]
        if limits["name"] != "mcp251xfd":
            raise ValueError(f"Expected mcp251xfd, got {limits['name']}")
        expected = phase_values(PROFILES[profile][name])
        for field, value in (("tseg1", expected["prop_seg"] + expected["phase_seg1"]),
                             ("tseg2", expected["phase_seg2"]),
                             ("sjw", expected["sjw"]), ("brp", expected["brp"])):
            if not limits[field]["min"] <= value <= limits[field]["max"]:
                raise ValueError(f"Unsupported {name} {field}={value}")
    return data


def verify_active(link, profile):
    data = validate_controller(link, profile)
    if "UP" not in link["flags"]:
        raise ValueError("Interface is down")
    modes = set(data["ctrlmode"])
    if not {"FD", "BERR-REPORTING"} <= modes or modes & {"FD-NON-ISO", "LOOPBACK", "LISTEN-ONLY"}:
        raise ValueError("Expected normal ISO CAN-FD with error reporting")
    if data["restart_ms"] != 1000 or link["txqlen"] != 65535:
        raise ValueError("Expected restart-ms=1000 and txqueuelen=65535")
    for name, key in (("nominal", "bittiming"), ("data", "data_bittiming")):
        actual = data[key]
        for field, value in phase_values(PROFILES[profile][name]).items():
            if field == "sample_point":
                if abs(float(actual[field]) - value) < 0.00001:
                    continue
            elif actual[field] == value:
                continue
            raise ValueError(f"{name} {field}: expected {value}, got {actual[field]}")
    return data["state"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=PROFILES, default="1m4")
    parser.add_argument("--interfaces", nargs="+", default=["can0"])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--inspect", action="store_true", help="Read controller capabilities; no changes")
    mode.add_argument("--check", action="store_true", help="Verify active profile; no changes")
    mode.add_argument("--apply", action="store_true", help="Actually take CAN down and change its timing")
    args = parser.parse_args(argv)
    if args.apply and os.geteuid() != 0:
        parser.error("--apply requires root")
    # Validate every requested controller before changing any interface.
    if args.apply or args.inspect or args.check:
        for interface in args.interfaces:
            link = read_link(interface)
            validate_controller(link, args.profile)
            if args.check:
                print(f"{interface}: {args.profile}, state={verify_active(link, args.profile)}")
            elif args.inspect:
                print(f"{interface}: 40 MHz mcp251xfd supports {args.profile}; no changes")
    if args.inspect or args.check:
        return 0
    for interface in args.interfaces:
        for command in commands(interface, args.profile):
            print(shlex.join(command), flush=True)
            if args.apply:
                subprocess.run(command, check=True)
        if args.apply:
            print(f"{interface}: verified {args.profile}, state={verify_active(read_link(interface), args.profile)}")
    if not args.apply:
        print("PREVIEW ONLY. No interface or persistent configuration was changed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
