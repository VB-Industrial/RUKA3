"""Short J4-only commissioning pulse; run on the SocketCAN Raspberry Pi."""
import argparse
import asyncio
import json
import time

import pycyphal.application
import uavcan.node
import uavcan.register
import uavcan.primitive.array


async def main(amps, duration):
    node = pycyphal.application.make_node(uavcan.node.GetInfo_1_0.Response())
    client = node.make_client(uavcan.register.Access_1_0, 104)
    client.response_timeout = 0.25
    node.start()

    async def access(name, value=None):
        request = uavcan.register.Access_1_0.Request(
            name=uavcan.register.Name_1_0(name=name))
        if value is not None:
            request.value = uavcan.register.Value_1_0(
                real32=uavcan.primitive.array.Real32_1_0(value=[value]))
        result = await client.call(request)
        if result is None:
            raise RuntimeError(f"No response: {name}")
        result = result[0].value
        return pycyphal.dsdl.to_builtin(result)

    async def log(name):
        value = await access(name)
        print(json.dumps({"time": time.monotonic(), "register": name, "value": value}), flush=True)
        return value

    try:
        await access("arm", 0)
        await log("fault_act")
        await log("fault_latch")
        await log("get_pos")
        await log("get_current")
        print(json.dumps({"event": "arm", "amps": amps, "duration": duration}), flush=True)
        result = await access("arm", 1)
        print(json.dumps({"arm_response": result}), flush=True)
        if not result.get("integer32", {}).get("value", [0])[0]:
            raise RuntimeError("Arm rejected")
        print(json.dumps({"current_response": await access("set_current", amps)}), flush=True)
        deadline = asyncio.get_running_loop().time() + duration
        refresh_at = asyncio.get_running_loop().time() + 0.1
        while asyncio.get_running_loop().time() < deadline:
            if asyncio.get_running_loop().time() >= refresh_at:
                await access("set_current", amps)
                refresh_at = asyncio.get_running_loop().time() + 0.1
            sample = await log("get_current")
            if sample["real32"]["value"][19] == 0:
                raise RuntimeError("Power stage stopped during the pulse")
            if sample["real32"]["value"][18] != 5:
                raise RuntimeError("Current mode was replaced during the pulse")
            await log("get_pos")
            await asyncio.sleep(0.01)
    finally:
        try:
            try:
                await access("set_current", 0)
            finally:
                await access("arm", 0)
            await log("fault_act")
            await log("fault_latch")
            await log("get_pos")
            await log("get_current")
            await log("tmc_diag")
        finally:
            client.close()
            node.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--amps", type=float, default=0)
    parser.add_argument("--duration", type=float, default=0.1)
    args = parser.parse_args()
    if not (-1.0 <= args.amps <= 1.0 and 0 < args.duration <= (7.0 if abs(args.amps) <= 0.4 else 5.0)):
        parser.error("J4 bounds: ±1 A up to 5 s; calibration at ±0.4 A up to 7 s")
    asyncio.run(main(args.amps, args.duration))
