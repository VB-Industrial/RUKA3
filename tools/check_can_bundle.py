#!/usr/bin/env python3
"""Audit prepared CAN/SWD images and write their identities; never uses hardware."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP_START = 0x08003000


def hex_bytes(path):
    result, base, eof = {}, 0, False
    for line in path.read_text().splitlines():
        if not line:
            continue
        if eof or not line.startswith(":"):
            raise ValueError(f"Malformed HEX {path}")
        record = bytes.fromhex(line[1:])
        if len(record) != record[0] + 5 or sum(record) & 255:
            raise ValueError(f"Invalid record length/checksum in {path}")
        count, address, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        payload = record[4:4 + count]
        if kind == 4:
            base = int.from_bytes(payload, "big") << 16
        elif kind == 0:
            for offset, value in enumerate(payload):
                target = base + address + offset
                if target in result:
                    raise ValueError(f"Duplicate HEX address in {path}")
                result[target] = value
        elif kind == 1:
            eof = True
        elif kind not in (3, 5):
            raise ValueError(f"Unsupported HEX record {kind} in {path}")
    if not eof or not result:
        raise ValueError(f"Incomplete HEX {path}")
    return result


def audit_image(app_path, full_path, app_end, ram_end):
    app, full = hex_bytes(app_path), hex_bytes(full_path)
    if not all(APP_START <= address < app_end for address in app):
        raise ValueError(f"App outside its partition: {app_path}")
    if not all(0x08000000 <= address < app_end for address in full):
        raise ValueError(f"Full image outside flash: {full_path}")
    if app != {address: value for address, value in full.items() if address >= APP_START}:
        raise ValueError(f"Full image contains a different application: {full_path}")
    for image, start, end in ((app, APP_START, app_end), (full, 0x08000000, APP_START)):
        vector = bytes(image[start + offset] for offset in range(8))
        sp, entry = int.from_bytes(vector[:4], "little"), int.from_bytes(vector[4:], "little")
        if not 0x20000000 < sp <= ram_end or sp % 8 or not start <= (entry & ~1) < end or not entry & 1:
            raise ValueError(f"Invalid reset vector in {full_path} at {start:#x}")
    return {
        "application": str(app_path.relative_to(ROOT)),
        "full_swd": str(full_path.relative_to(ROOT)),
        "application_sha256": hashlib.sha256(app_path.read_bytes()).hexdigest(),
        "full_swd_sha256": hashlib.sha256(full_path.read_bytes()).hexdigest(),
    }


def cache(path):
    return {line.split(":", 1)[0]: line.split("=", 1)[1]
            for line in path.read_text().splitlines()
            if "=" in line and ":" in line.split("=", 1)[0] and not line.startswith("//")}


def audit_joint(joint, g474_mode="bootstrap"):
    board = "g474" if joint <= 4 else "g431"
    name = "g474" if joint <= 4 else "VBDrive"
    directory = ROOT / f"build/{board}-joint{joint}"
    app_end = 0x08080000 if joint <= 4 else 0x08020000
    ram_end = 0x20020000 if joint <= 4 else 0x20008000
    candidates = list(directory.glob(f"{name}.hex")) + list(directory.glob(f"Boards/{board}/{name}.hex"))
    if len(candidates) != 1:
        raise ValueError(f"Expected one current image for J{joint}, found {candidates}")
    app = candidates[0]
    result = audit_image(app, app.with_name(name + "_full.hex"), app_end, ram_end)
    current_cache = cache(directory / "CMakeCache.txt")
    if current_cache["RUKA3_JOINT_INDEX"] != str(joint):
        raise ValueError(f"Incorrect joint index: J{joint}")
    bootstrap = current_cache["RUKA3_BOOTSTRAP"]
    if bootstrap != ("ON" if joint <= 4 and g474_mode == "bootstrap" else "OFF"):
        raise ValueError(f"Unexpected build mode J{joint}: bootstrap={bootstrap}")
    boot_dir = app.parent / "VBBoot"
    boot_cache = cache(boot_dir / "CMakeCache.txt")
    if boot_cache["BOOTLOADER_DEFAULT_NODE_ID"] != str(100 + joint):
        raise ValueError(f"Incorrect bootloader node: J{joint}")
    if boot_cache["BOOTLOADER_USE_EEPROM_CONFIG"] != "OFF" or boot_cache["BOOTLOADER_FDCAN_ID_FORMAT"] != "extended":
        raise ValueError(f"Unexpected bootloader transport for J{joint}")
    if boot_cache["BOOTLOADER_FDCAN_CLOCK_SOURCE"] != ("PLL" if joint <= 4 else "PCLK1"):
        raise ValueError(f"Incorrect bootloader clock for J{joint}")
    commands = json.loads((boot_dir / "compile_commands.json").read_text())
    files = [Path(command["file"]).resolve() for command in commands]
    if files.count(ROOT / "Boot/fdcan.c") != 1 or ROOT / "ThirdParty/VBBoot/App/state_manager.c" in files:
        raise ValueError(f"Legacy bootloader timing initializer still compiled for J{joint}")
    rollback = directory / "rollback-1m8" / f"{name}.hex"
    result["rollback_1m8"] = audit_image(rollback, rollback.with_name(name + "_full.hex"), app_end, ram_end)
    if joint <= 4 and g474_mode == "current":
        can_only = directory / "can-only-1m4" / f"{name}.hex"
        result["rollback_can_only_1m4"] = audit_image(can_only, can_only.with_name(name + "_full.hex"), app_end, ram_end)
    result.update(joint=joint, node_id=100 + joint, board=board, bootstrap=bootstrap)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--g474-mode", choices=("bootstrap", "current"), default="bootstrap")
    args = parser.parse_args()
    manifest = {"nominal_bitrate": 1_000_000, "data_bitrate": 4_000_000,
                "nominal_sample_point": 0.75, "data_sample_point": 0.8,
                "hardware_verified": False, "applied": False,
                "can_only_upload_replaces_bootloader": False,
                "g474_mode": args.g474_mode,
                "joints": [audit_joint(joint, args.g474_mode) for joint in range(1, 7)]}
    for joint in manifest["joints"]:
        print(f"J{joint['joint']}: node={joint['node_id']} {joint['board']} bootstrap={joint['bootstrap']}; "
              "CAN/full-SWD/rollback HEX checksums, partitions and boot profile OK")
    if args.manifest:
        args.manifest.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"Manifest: {args.manifest}")


if __name__ == "__main__":
    main()
