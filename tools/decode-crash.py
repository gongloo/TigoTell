#!/usr/bin/env python3
"""Translate a TigoTell crash report into function names.

Usage:  python3 decode-crash.py firmware.elf "task async_tcp, PC 0x4201..., backtrace 0x4201... 0x4037..."
Needs:  pip install pyelftools
The ELF must be from the same firmware version that crashed.
"""
import bisect, re, sys
from elftools.elf.elffile import ELFFile

def load_symbols(path):
    with open(path, "rb") as f:
        elf = ELFFile(f)
        syms = []
        for sec in elf.iter_sections():
            if sec.name != ".symtab":
                continue
            for s in sec.iter_symbols():
                if s["st_info"]["type"] == "STT_FUNC" and s["st_value"]:
                    syms.append((s["st_value"] & ~1, s["st_size"], s.name))
        syms.sort()
        return syms

def demangle(name):
    try:
        import subprocess
        return subprocess.run(["c++filt", name], capture_output=True, text=True).stdout.strip() or name
    except Exception:
        return name

def lookup(syms, starts, addr):
    i = bisect.bisect_right(starts, addr) - 1
    if i >= 0:
        start, size, name = syms[i]
        if addr < start + max(size, 1) + 64:
            return f"{demangle(name)} +0x{addr - start:x}"
    return "?"

if len(sys.argv) < 3:
    sys.exit(__doc__)
syms = load_symbols(sys.argv[1])
starts = [s[0] for s in syms]
report = " ".join(sys.argv[2:])
print(report.split(",")[0])
for addr in re.findall(r"0x[0-9a-fA-F]{8}", report):
    a = int(addr, 16)
    if a == 0:
        continue
    # Xtensa backtrace entries point just after the call; subtract 3 for the call site.
    print(f"  {addr}  {lookup(syms, starts, a if 'PC 0x' + addr[2:] in report else a - 3)}")
