# SIYI SDK Quickstart Guide

This guide shows how to connect to a SIYI gimbal camera and run basic commands using the async SDK. All examples run against either `MockTransport` (for testing) or real transports (UDP, TCP, Serial).

## UDP Example

The fastest way to get started is via UDP. By default, the SDK targets `192.168.144.25:37260` (the gimbal's default IP and port).

```python
import asyncio
from siyi_sdk import connect_udp

async def main() -> None:
    """Connect to gimbal over UDP and read firmware version and attitude."""
    async with await connect_udp("192.168.144.25", 37260) as client:
        fw = await client.get_firmware_version()
        print(f"Camera FW: {fw.camera}, Gimbal FW: {fw.gimbal}, Zoom FW: {fw.zoom}")
        att = await client.get_gimbal_attitude()
        print(f"Yaw={att.yaw_deg:.1f}°  Pitch={att.pitch_deg:.1f}°  Roll={att.roll_deg:.1f}°")

if __name__ == "__main__":
    asyncio.run(main())
```

### Testing with MockTransport

To test without hardware, use `MockTransport`:

```python
import asyncio
from unittest.mock import MagicMock
from siyi_sdk import SIYIClient
from siyi_sdk.transport.mock import MockTransport
from siyi_sdk.models import FirmwareVersion, GimbalAttitude

async def main() -> None:
    """Connect to mock gimbal and read firmware version and attitude."""
    transport = MockTransport()
    
    # Queue mock responses
    fw_response = FirmwareVersion(camera="1.2.3", gimbal="4.5.6", zoom="0.0.0")
    att_response = GimbalAttitude(
        yaw_deg=30.0, pitch_deg=-45.0, roll_deg=0.0,
        yaw_speed=0, pitch_speed=0, roll_speed=0,
        mag_x=0, mag_y=0, mag_z=0
    )
    transport.queue_response(fw_response)
    transport.queue_response(att_response)
    
    async with SIYIClient(transport) as client:
        fw = await client.get_firmware_version()
        print(f"Camera FW: {fw.camera}, Gimbal FW: {fw.gimbal}, Zoom FW: {fw.zoom}")
        att = await client.get_gimbal_attitude()
        print(f"Yaw={att.yaw_deg:.1f}°  Pitch={att.pitch_deg:.1f}°  Roll={att.roll_deg:.1f}°")

if __name__ == "__main__":
    asyncio.run(main())
```

## TCP Example

TCP connections also work (with automatic heartbeat support):

```python
import asyncio
from siyi_sdk import connect_tcp

async def main() -> None:
    """Connect to gimbal over TCP and read firmware version and attitude."""
    async with await connect_tcp("192.168.144.25", 37260) as client:
        fw = await client.get_firmware_version()
        print(f"Camera FW: {fw.camera}, Gimbal FW: {fw.gimbal}, Zoom FW: {fw.zoom}")
        att = await client.get_gimbal_attitude()
        print(f"Yaw={att.yaw_deg:.1f}°  Pitch={att.pitch_deg:.1f}°  Roll={att.roll_deg:.1f}°")

if __name__ == "__main__":
    asyncio.run(main())
```

## Serial Example

For serial connections (e.g., USB-to-UART), specify the port and baud rate:

```python
import asyncio
from siyi_sdk import connect_serial

async def main() -> None:
    """Connect to gimbal over serial and read firmware version and attitude."""
    # Default baud rate is 115200; adjust /dev/ttyUSB0 to your port
    async with await connect_serial("/dev/ttyUSB0", 115200) as client:
        fw = await client.get_firmware_version()
        print(f"Camera FW: {fw.camera}, Gimbal FW: {fw.gimbal}, Zoom FW: {fw.zoom}")
        att = await client.get_gimbal_attitude()
        print(f"Yaw={att.yaw_deg:.1f}°  Pitch={att.pitch_deg:.1f}°  Roll={att.roll_deg:.1f}°")

if __name__ == "__main__":
    asyncio.run(main())
```

## More Examples

For runnable command examples, see the `examples/` directory:

- **udp_heartbeat.py** — Connect and read firmware + attitude.
- **set_attitude.py** — Move gimbal to target angles and return to centre.
- **subscribe_attitude_stream.py** — Stream gimbal attitude at 10 Hz.
- **thermal_spot_temperature.py** — Read spot temperature on thermal sensor.
- **laser_ranging.py** — Enable laser and poll distance.

## Configuration

### Environment Variables

**`SIYI_LOG_LEVEL`** — Control logging verbosity:

```bash
export SIYI_LOG_LEVEL=DEBUG  # Verbose frame-level logs
export SIYI_LOG_LEVEL=INFO   # Default: command dispatch + ACKs
export SIYI_LOG_LEVEL=WARNING  # Only warnings and errors
export SIYI_LOG_LEVEL=ERROR  # Only errors
```

**`SIYI_PROTOCOL_TRACE`** — Enable hex dump of every frame (TX/RX):

```bash
export SIYI_PROTOCOL_TRACE=1  # Adds `payload_hex` to every frame log record
```

### Client Configuration

```python
async with await connect_udp("192.168.144.25", 37260) as client:
    # Read-only: customize transport per-instance (see SIYIClient.__init__)
    pass
```

## Error Handling

The SDK raises structured exceptions for common failure modes:

```python
import asyncio
from siyi_sdk import connect_udp
from siyi_sdk.exceptions import NotConnectedError, TimeoutError

async def main() -> None:
    try:
        async with await connect_udp("192.168.144.25", 37260) as client:
            fw = await client.get_firmware_version()
    except TimeoutError:
        print("Device did not respond within 1 second")
    except NotConnectedError:
        print("Connection lost or never established")

if __name__ == "__main__":
    asyncio.run(main())
```

## Next Steps

- Read the [Protocol Reference](protocol.md) for wire-level details.
- Check the [API Reference](../README.md#api-reference) for the full list of commands.
- See `CONTRIBUTING.md` for development setup and testing guidelines.
