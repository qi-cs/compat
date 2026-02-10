def addSSOptions(parser):
    parser.add_argument(
        "--arch",
        choices=["ARM", "RISCV", "X86"],
        help="ARM or RISCV or X86 must be specified",
    )


def addO3Options(parser):
    parser.add_argument(
        "--num-ROB",
        default=9,
        action="store",
        type=int,
        help="num ROB entries",
    )

    parser.add_argument(
        "--num-IQ", default=9, action="store", type=int, help="num IQ entries"
    )

    parser.add_argument(
        "--num-LQ", default=9, action="store", type=int, help="num LQ entries"
    )

    parser.add_argument(
        "--num-SQ", default=9, action="store", type=int, help="num SQ entries"
    )

    parser.add_argument(
        "--num-PhysReg",
        default=9,
        action="store",
        type=int,
        help="num physical registers",
    )
