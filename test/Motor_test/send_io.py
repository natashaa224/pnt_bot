#!/usr/bin/env python3
"""
Send a CMD_IO (0x05) packet to the PMC firmware over TCP.

Usage:
    ./send_io.py <host> <port> [--flags 0xNN] [named flags...]

    --flags takes the raw io_flags value directly, e.g.:
        --flags 0x04   solenoid only
        --flags 0x05   solenoid + ssr
        --flags 0x00   everything off

    Or set individual channels by name (OR'd together), e.g.:
        ./send_io.py 192.168.1.100 5000 --solenoid
        ./send_io.py 192.168.1.100 5000 --solenoid --relay1

    --flags and named flags are mutually exclusive.

Bit layout (see IO_* in include/protocol_definitions.h — v2.1):
    bit0 IO_SSR          bit1 IO_RELAY1       bit2 IO_SOLENOID     bit3 IO_RELAY2
    bit4 IO_RELAY3       bit5 IO_RELAY4       bit6 IO_TOWER_GREEN  bit7 IO_TOWER_AMBER
    bit8 IO_TOWER_RED    bit9 IO_BUZZER
    (bit position N for N=0..7 is DO channel N; bits 8-9 are the J8 outputs.
     Bits 10-15 reserved.)

io_flags is a uint16_t (PayloadIoCmd.io_flags) — payload is 2 bytes,
little-endian, and payload_len in the frame must be 0x02 or the firmware
rejects the packet ("IO_CMD — bad payload length, ignored").
"""
import argparse
import socket
import sys

# ── IO_* bitmask — must match include/protocol_definitions.h exactly ──────────
IO_SSR          = 1 << 0   # DO00 — ssr_on
IO_RELAY1       = 1 << 1   # DO01 — relay1_on
IO_SOLENOID     = 1 << 2   # DO02 — solenoid_on
IO_RELAY2       = 1 << 3   # DO03 — relay2_on
IO_RELAY3       = 1 << 4   # DO04 — relay3_on
IO_RELAY4       = 1 << 5   # DO05 — relay4_on
IO_TOWER_GREEN  = 1 << 6   # DO06 — tower_green
IO_TOWER_AMBER  = 1 << 7   # DO07 — tower_amber
IO_TOWER_RED    = 1 << 8   # DIO02-OUT — tower_red
IO_BUZZER       = 1 << 9   # DIO03-OUT — buzzer_on

NAMED_FLAGS = {
    "ssr":         IO_SSR,
    "relay1":      IO_RELAY1,
    "solenoid":    IO_SOLENOID,
    "relay2":      IO_RELAY2,
    "relay3":      IO_RELAY3,
    "relay4":      IO_RELAY4,
    "tower_green": IO_TOWER_GREEN,
    "tower_amber": IO_TOWER_AMBER,
    "tower_red":   IO_TOWER_RED,
    "buzzer":      IO_BUZZER,
}

CMD_IO = 0x05


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_io_packet(io_flags: int) -> bytes:
    # PayloadIoCmd.io_flags is uint16_t -> 2-byte little-endian payload.
    payload = bytes([io_flags & 0xFF, (io_flags >> 8) & 0xFF])
    body = bytes([CMD_IO, len(payload)]) + payload
    crc = crc16_modbus(body)
    return bytes([0xAA, 0x55]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def describe(io_flags: int) -> str:
    on = [name for name, bit in NAMED_FLAGS.items() if io_flags & bit]
    return "+".join(on) if on else "all off"


def main():
    parser = argparse.ArgumentParser(
        description="Send a CMD_IO packet to the PMC firmware over TCP.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    parser.add_argument("--flags", type=lambda s: int(s, 0),
                         help="raw io_flags value, e.g. 0x04 (mutually exclusive with named flags)")
    for name in NAMED_FLAGS:
        parser.add_argument(f"--{name.replace('_', '-')}", action="store_true",
                             dest=name, help=f"turn on {name} (bit 0x{NAMED_FLAGS[name]:04x})")

    args = parser.parse_args()
    named_set = [name for name in NAMED_FLAGS if getattr(args, name)]

    if args.flags is not None and named_set:
        parser.error("--flags cannot be combined with named flags")

    if args.flags is not None:
        io_flags = args.flags & 0xFFFF
    else:
        io_flags = 0
        for name in named_set:
            io_flags |= NAMED_FLAGS[name]

    packet = build_io_packet(io_flags)

    with socket.create_connection((args.host, args.port), timeout=3) as s:
        s.sendall(packet)
    print(f"Sent io_flags=0x{io_flags:04x} ({describe(io_flags)}) -> {packet.hex(' ')}")


if __name__ == "__main__":
    main()
