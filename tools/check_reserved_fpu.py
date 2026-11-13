"""Reject FPU instructions outside the assembly that owns the bus registers.

The project reserves s0-s31 with -ffixed-sN, but those compiler options do not
affect prebuilt newlib. Even snprintf("%s") can overwrite d7/s14 and break MREQ.
Check the final linked ELF, including library routines, before producing firmware.
"""
import argparse
import re
import subprocess
import sys


def unsafe_instructions(disassembly):
    labels = {}
    instructions = []
    owner = "?"
    for line in disassembly.splitlines():
        label = re.match(r"^\s*([0-9a-fA-F]+) <([^>]+)>:", line)
        if label:
            owner = label[2]
            labels[owner] = int(label[1], 16)
            continue
        fields = line.split("\t")
        if len(fields) < 3 or not re.fullmatch(r"\s*[0-9a-fA-F]+:", fields[0]):
            continue
        address = int(fields[0].strip()[:-1], 16)
        operation = " ".join(field.strip() for field in fields[2:]).strip()
        if not operation:
            continue
        mnemonic, _, operands = operation.partition(" ")
        instructions.append((address, owner, mnemonic, operands.strip()))

    allowed = []
    for entry, tail in (("init_fpu_regs", "init_fpu_regs"),
                        ("EXTI4_IRQHandler", "end_EXTI4_IRQHandler")):
        if entry not in labels or tail not in labels:
            raise ValueError("Missing bus-register owner symbol: " + entry + "/" + tail)
        returns = [address for address, _, op, args in instructions
                   if address >= labels[tail] and op in ("bx", "bx.w") and args == "lr"]
        if not returns:
            raise ValueError("Cannot locate end of " + entry)
        allowed.append((labels[entry], min(returns) + 2))

    return [instruction for instruction in instructions
            if instruction[2].startswith("v") and
            not any(start <= instruction[0] < end for start, end in allowed)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf")
    parser.add_argument("--objdump", default="arm-none-eabi-objdump")
    args = parser.parse_args()
    result = subprocess.run([args.objdump, "-d", "-j", ".text", args.elf],
                            text=True, capture_output=True, check=True)
    try:
        violations = unsafe_instructions(result.stdout)
    except ValueError as error:
        parser.exit(1, str(error) + "\n")
    if violations:
        print("ERROR: linked code uses FPU registers reserved by the live bus ISR:", file=sys.stderr)
        for address, owner, op, operands in violations[:12]:
            print(f"  {address:08x} <{owner}>: {op} {operands}", file=sys.stderr)
        print(f"{len(violations)} unsafe instruction(s). Avoid printf-family/float libraries; "
              "-ffixed-sN does not rebuild libc.", file=sys.stderr)
        return 1
    print("PASS linked ARM ELF: FPU registers used only by their bus-assembly owners")
    return 0


if __name__ == "__main__":
    sys.exit(main())
