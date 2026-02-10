# Copyright (c) 2025
# Update ARM checkpoint thread-context (xc) regs.vector to current SVE size.
# Old checkpoints had a smaller vector register file; current gem5 expects
# NumVecRegs * NumVecElemPerVecReg * sizeof(VecElem) = 44 * 64 * 4 = 11264 bytes.

depends = "arm-sve"

# Expected size: 44 vec regs * 64 words * 4 bytes (serialized as one token per byte)
ARM_VEC_REGS_SIZE = 11264


def upgrader(cpt):
    """
    Expand regs.vector in ARM xc (thread context) sections to ARM_VEC_REGS_SIZE
    when the checkpoint has fewer tokens (old format). Pad with zeros.
    """
    if not cpt.has_option("root", "isa") or cpt.get("root", "isa") != "arm":
        return
    import re

    for sec in cpt.sections():
        if re.search(r".*\.xc\.\d+$", sec) and cpt.has_option(
            sec, "regs.vector"
        ):
            tokens = cpt.get(sec, "regs.vector").split()
            if len(tokens) < ARM_VEC_REGS_SIZE:
                cpt.set(
                    sec,
                    "regs.vector",
                    " ".join("0" for _ in range(ARM_VEC_REGS_SIZE)),
                )
