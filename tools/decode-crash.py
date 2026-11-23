#!/usr/bin/env python3
"""Translate a TigoTell crash report into function names.

Usage:  python3 decode-crash.py firmware.elf "task async_tcp, PC 0x4201..., backtrace 0x4201... 0x4037..."
Needs:  pip install pyelftools
The ELF must be from the same firmware version that crashed.
"""
import bisect, re, sys

def load_symbols(path):
    from elftools.elf.elffile import ELFFile
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

def parse_report(report: str):
    """Parse a crash report into header, PC, fault addr, backtrace, and corrupted flag."""
    header = report.split(",")[0].strip()
    pc_match = re.search(r"\bPC\s+(0x[0-9a-fA-F]+)", report)
    addr_match = re.search(r"\baddr\s+(0x[0-9a-fA-F]+)", report)
    bt_match = re.search(r"\bbacktrace\b(.*?)(?:\|corrupted|$)", report, re.DOTALL)
    bt_addrs = re.findall(r"0x[0-9a-fA-F]+", bt_match.group(1)) if bt_match else []
    return {
        "header": header,
        "pc": pc_match.group(1) if pc_match else None,
        "addr": addr_match.group(1) if addr_match else None,
        "backtrace": [a for a in bt_addrs if int(a, 16) != 0],
        "corrupted": "|corrupted" in report,
    }

def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    syms = load_symbols(sys.argv[1])
    starts = [s[0] for s in syms]
    report = " ".join(sys.argv[2:])
    parsed = parse_report(report)
    print(parsed["header"])
    if parsed["pc"] and int(parsed["pc"], 16) != 0:
        print(f"  PC   {parsed['pc']}  {lookup(syms, starts, int(parsed['pc'], 16))}")
    if parsed["addr"]:
        print(f"  addr {parsed['addr']}  (fault address)")
    for addr in parsed["backtrace"]:
        a = int(addr, 16)
        # Xtensa backtrace entries point just after the call; subtract 3 for the call site.
        print(f"  {addr}  {lookup(syms, starts, a - 3)}")
    if parsed["corrupted"]:
        print("  (backtrace corrupted)")

if __name__ == "__main__":
    main()
