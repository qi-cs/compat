from common import ObjectList
from common.Caches import *

import m5
from m5.objects import *


def get_compressor(options):
    if options.compressor == "Base64Delta8":
        compressor = Base64Delta8()
    elif options.compressor == "Base64Delta16":
        compressor = Base64Delta16()
    elif options.compressor == "Base64Delta32":
        compressor = Base64Delta32()
    elif options.compressor == "Base32Delta8":
        compressor = Base32Delta8()
    elif options.compressor == "Base32Delta16":
        compressor = Base32Delta16()
    elif options.compressor == "Base32Delta32":
        compressor = Base32Delta32()
    elif options.compressor == "CPack":
        compressor = CPack()
        compressor.dictionary_size = 16384
    elif options.compressor == "FPC":
        compressor = FPC()
        compressor.dictionary_size = 16384
    elif options.compressor == "FPCD":
        compressor = FPCD()
    elif options.compressor == "FrequentValuesCompressor":
        compressor = FrequentValuesCompressor()
    elif options.compressor == "RepeatedQwordsCompressor":
        compressor = RepeatedQwordsCompressor()
    elif options.compressor == "ZeroCompressor":
        compressor = ZeroCompressor()
    elif options.compressor == "PerfectCompressor":
        compressor = PerfectCompressor()
        compressor.max_compression_ratio = 4
    elif options.compressor == "BDI":
        compressor = BDI()

    compressor.block_size = options.compPktLength
    compressor.chunk_size_bits = options.chunk_size_bits

    return compressor


def get_metacompressor(options):
    if options.metacompressor == "Base64Delta8":
        compressor = Base64Delta8()
    elif options.metacompressor == "Base64Delta16":
        compressor = Base64Delta16()
    elif options.metacompressor == "Base64Delta32":
        compressor = Base64Delta32()
    elif options.metacompressor == "Base32Delta8":
        compressor = Base32Delta8()
    elif options.metacompressor == "Base32Delta16":
        compressor = Base32Delta16()
    elif options.metacompressor == "Base32Delta32":
        compressor = Base32Delta32()
    elif options.metacompressor == "CPack":
        compressor = CPack()
        compressor.dictionary_size = 16384
    elif options.metacompressor == "FPC":
        compressor = FPC()
        compressor.dictionary_size = 16384
    elif options.metacompressor == "FPCD":
        compressor = FPCD()
    elif options.metacompressor == "FrequentValuesCompressor":
        compressor = FrequentValuesCompressor()
    elif options.metacompressor == "RepeatedQwordsCompressor":
        compressor = RepeatedQwordsCompressor()
    elif options.metacompressor == "ZeroCompressor":
        compressor = ZeroCompressor()
    elif options.metacompressor == "PerfectCompressor":
        compressor = PerfectCompressor()
        compressor.max_compression_ratio = 4
    elif options.metacompressor == "BDI":
        compressor = BDI()

    compressor.block_size = options.compPktLength
    compressor.chunk_size_bits = options.meta_chunk_size_bits

    return compressor


def get_dualcompressor(options):
    if options.dualCompressor == "Base64Delta8":
        compressor = Base64Delta8()
    elif options.dualCompressor == "Base64Delta16":
        compressor = Base64Delta16()
    elif options.dualCompressor == "Base64Delta32":
        compressor = Base64Delta32()
    elif options.dualCompressor == "Base32Delta8":
        compressor = Base32Delta8()
    elif options.dualCompressor == "Base32Delta16":
        compressor = Base32Delta16()
    elif options.dualCompressor == "Base32Delta32":
        compressor = Base32Delta32()
    elif options.dualCompressor == "CPack":
        compressor = CPack()
        compressor.dictionary_size = 16384
    elif options.dualCompressor == "FPC":
        compressor = FPC()
        compressor.dictionary_size = 16384
    elif options.dualCompressor == "FPCD":
        compressor = FPCD()
    elif options.dualCompressor == "FrequentValuesCompressor":
        compressor = FrequentValuesCompressor()
    elif options.dualCompressor == "RepeatedQwordsCompressor":
        compressor = RepeatedQwordsCompressor()
    elif options.dualCompressor == "ZeroCompressor":
        compressor = ZeroCompressor()
    elif options.dualCompressor == "PerfectCompressor":
        compressor = PerfectCompressor()
        compressor.max_compression_ratio = 4
    elif options.dualCompressor == "BDI":
        compressor = BDI()

    compressor.block_size = options.compPktLength

    return compressor


def get_patternchecker(options):
    patternChecker = PatternChecker()
    return patternChecker
