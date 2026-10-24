#!/usr/bin/env python3
"""
pmc_io_map.py

Loads pmc_io_map.yaml and provides logical<->bit translation for the bridge.
The bridge is the ONLY place that touches bits; everything else uses logical
signal names on static topics.

    io = PmcIoMap('/path/to/pmc_io_map.yaml')

    # inbound: raw DI byte -> logical booleans
    signals = io.decode_inputs(di_raw)      # {'estop': True, 'ceiling_detect': False, ...}

    # outbound: logical output states -> DO word for CMD_IO
    word = io.compose_outputs({'solenoid': True, 'tower_green': True})

Rules:
  - input asserted = (bit set) XOR active_low ; disabled inputs always False.
  - compose_outputs sets a bit only for enabled outputs that are True.
"""

import yaml


class PmcIoMap:
    def __init__(self, path):
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
        io = cfg.get('pmc_io', {})
        self._inputs  = io.get('digital_inputs', {})
        self._outputs = io.get('digital_outputs', {})

    # ── inputs ─────────────────────────────────────────────────────────────────
    def input_names(self):
        return list(self._inputs.keys())

    def decode_input(self, name, di_raw):
        """One logical input from the raw byte. Disabled -> always False."""
        spec = self._inputs.get(name)
        if spec is None or not spec.get('enabled', False):
            return False
        bit = (di_raw >> spec['bit']) & 1
        return (bit == 0) if spec.get('active_low', False) else (bit == 1)

    def decode_inputs(self, di_raw):
        """All logical inputs from the raw byte -> dict."""
        return {n: self.decode_input(n, di_raw) for n in self._inputs}

    # ── outputs ────────────────────────────────────────────────────────────────
    def output_names(self):
        return list(self._outputs.keys())

    def output_bit(self, name):
        """Returns (type, bit) for an enabled output, else None. type is 'do'|'pio'."""
        spec = self._outputs.get(name)
        if spec is None or not spec.get('enabled', False):
            return None
        return (spec.get('type', 'do'), spec['bit'])

    def compose_outputs(self, states):
        """
        states: {logical_name: bool}. Returns (do_word, pio_word) — a bit set for
        each enabled output whose state is True, routed to the word matching its
        'type' (do or pio). Unknown/disabled names ignored.
        """
        do_word = 0
        pio_word = 0
        for name, on in states.items():
            if not on:
                continue
            res = self.output_bit(name)
            if res is None:
                continue
            otype, bit = res
            if otype == 'pio':
                pio_word |= (1 << bit)
            else:
                do_word |= (1 << bit)
        return do_word, pio_word


# quick self-test
if __name__ == '__main__':
    import sys
    io = PmcIoMap(sys.argv[1] if len(sys.argv) > 1 else 'pmc_io_map.yaml')
    print('inputs :', io.input_names())
    print('outputs:', io.output_names())
    # estop is active_low -> bit3=0 means asserted
    print('di_raw=0x00 ->', {k: v for k, v in io.decode_inputs(0x00).items() if v})
    print('di_raw=0xFF ->', {k: v for k, v in io.decode_inputs(0xFF).items() if v})
    do_word, pio_word = io.compose_outputs({'solenoid': True, 'tower_green': True})
    print(f'solenoid+green -> do=0x{do_word:02x} pio=0x{pio_word:02x}')
