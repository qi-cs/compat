# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Steve Reinhardt

# Simple test script
#
# "m5 test.py"

import argparse
import optparse
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import (
    addToPath,
    fatal,
    warn,
)

addToPath("../")

from common import (
    CacheConfig,
    CompressConfig,
    CpuConfig,
    MemConfig,
    ObjectList,
    Options,
    Simulation,
    SSConfig,
    SSOptions,
)
from common.Caches import *
from get_spec_proc import Spec17
from ruby import Ruby


def get_one_spec_2017_process(
    spec17_obj, benchmark_name, benchmark_mode, idx=0
):
    process = spec17_obj.gen_proc(benchmark_name, benchmark_mode, idx)
    if process:
        return process
    else:
        print(f"No SPEC2017 benchmark named {benchmark_name}! Exiting.")
        sys.exit(1)


arch_suffix = {
    "ARM": "_base.aarch64test-64",
    "RISCV": "_r_base.rv64g-gcc-8.2-64",
    #'X86': '_s_base.mytest-m64',
    "X86": "_base.mytest-m64",
}


def get_spec_2017_processes(options):
    benchmarks = options.benchmark.split(";")
    stdouts = options.benchmark_stdout.split(";")
    stderrs = options.benchmark_stderr.split(";")
    if not options.arch:
        print(
            "No Arch(ARM, RISCV, X86) specified. Exiting!\n", file=sys.stderr
        )
        sys.exit(1)

    archsuffix = ""
    if options.spec_mode == "speed":
        archsuffix = "_s" + arch_suffix[options.arch]
    else:
        archsuffix = "_r" + arch_suffix[options.arch]

    spec17 = Spec17(archsuffix, options.spec_size, options.spec_mode)
    num_processes = len(benchmarks)
    multiprocesses = []
    assert len(stderrs) == len(benchmarks)
    assert len(stdouts) == len(benchmarks)
    index = 0
    if options.num_cpus != 1 and options.num_bms != 1:
        for numcpu in range(options.num_cpus):
            for bm in benchmarks:
                proc = get_one_spec_2017_process(
                    spec17, bm, options.spec_mode, index
                )
                proc.output = stdouts[0] + str(index)
                proc.errout = stderrs[0] + str(index)
                multiprocesses.append(proc)
            index += 1
    else:
        for bm in benchmarks:
            proc = get_one_spec_2017_process(spec17, bm, options.spec_mode)
            proc.output = stdouts[index]
            proc.errout = stderrs[index]
            multiprocesses.append(proc)
        index += 1

    return multiprocesses, num_processes


def get_processes(options):
    """Interprets provided options and returns a list of processes"""

    if options.spec_2017_bench:
        return get_spec_2017_processes(options)

    multiprocesses = []
    inputs = []
    outputs = []
    errouts = []
    pargs = []

    workloads = options.cmd.split(";")

    print("workload:", workloads)
    if options.input != "":
        inputs = options.input.split(";")
    if options.output != "":
        outputs = options.output.split(";")
    if options.errout != "":
        errouts = options.errout.split(";")
    if options.options != "":
        pargs = options.options.split(";")

    idx = 0
    for wrkld in workloads:
        process = Process(pid=100 + idx)
        process.executable = wrkld
        process.cwd = os.getcwd()

        if options.env:
            with open(options.env) as f:
                process.env = [line.rstrip() for line in f]

        if len(pargs) > idx:
            process.cmd = [wrkld] + pargs[idx].split()
        else:
            process.cmd = [wrkld]

        if len(inputs) > idx:
            process.input = inputs[idx]
        if len(outputs) > idx:
            process.output = outputs[idx]
        if len(errouts) > idx:
            process.errout = errouts[idx]

        multiprocesses.append(process)
        idx += 1

    if options.smt:
        assert options.cpu_type == "DerivO3CPU"
        return multiprocesses, idx
    else:
        return multiprocesses, 1


parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)
Options.addSpec2006Options(parser)
SSOptions.addSSOptions(parser)
SSOptions.addO3Options(parser)

if "--ruby" in sys.argv:
    Ruby.define_options(parser)

options = parser.parse_args()

# if args:
#    print("Error: script doesn't take any positional arguments")
#    sys.exit(1)

multiprocesses = []
numThreads = 1

if options.bench:
    apps = options.bench.split("-")
    if len(apps) != options.num_cpus:
        print("number of benchmarks not equal to set num_cpus!")
        sys.exit(1)

    for app in apps:
        try:
            if buildEnv["TARGET_ISA"] == "alpha":
                exec(
                    "workload = %s('alpha', 'tru64', '%s')"
                    % (app, options.spec_input)
                )
            elif buildEnv["TARGET_ISA"] == "arm":
                exec(
                    "workload = %s('arm_%s', 'linux', '%s')"
                    % (app, options.arm_iset, options.spec_input)
                )
            else:
                exec(
                    "workload = %s(buildEnv['TARGET_ISA', 'linux', '%s')"
                    % (app, options.spec_input)
                )
            print("workload??? ", workload)
            multiprocesses.append(workload.makeProcess())
        except:
            print(
                "Unable to find workload for %s: %s"
                % (buildEnv["TARGET_ISA"], app),
                file=sys.stderr,
            )
            sys.exit(1)
elif options.cmd or options.spec_2017_bench:
    multiprocesses, numThreads = get_processes(options)
else:
    print("No workload specified. Exiting!\n", file=sys.stderr)
    sys.exit(1)


(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
print(CPUClass, FutureClass)
CPUClass.numThreads = numThreads
# CPUClass.instTrace = True if options.instTrace else False

# Check -- do not allow SMT with multiple CPUs
if options.smt and options.num_cpus > 1:
    fatal("You cannot use SMT with multiple CPUs!")

print(CPUClass, test_mem_mode, FutureClass)

np = options.num_cpus
system = System(
    cpu=[CPUClass(cpu_id=i) for i in range(np)],
    mem_mode=test_mem_mode,
    mem_ranges=[AddrRange(options.mem_size)],
    cache_line_size=options.cacheline_size,
)

if numThreads > 1:
    system.multi_thread = True

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage=options.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(
    clock=options.sys_clock, voltage_domain=system.voltage_domain
)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(
    clock=options.cpu_clock, voltage_domain=system.cpu_voltage_domain
)

# If elastic tracing is enabled, then configure the cpu and attach the elastic
# trace probe
if options.elastic_trace_en:
    CpuConfig.config_etrace(CPUClass, system.cpu, options)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

for process in multiprocesses:
    print("multproc:", multiprocesses)

# checkif empty workload helps
for i in range(np):
    if options.smt:
        system.cpu[i].workload = multiprocesses
    elif len(multiprocesses) == 1:
        system.cpu[i].workload = multiprocesses[0]
    else:
        system.cpu[i].workload = multiprocesses[i]

    if options.simpoint_profile:
        system.cpu[i].addSimPointProbe(options.simpoint_interval)

    if options.checker:
        system.cpu[i].addCheckerCpu()

    print(system.cpu[i], "-- create threads")
    print(system.cpu[i].workload, "-- workload")
for i in range(np):
    system.cpu[i].createThreads()


MemClass = Simulation.setMemClass(options)
system.membus = SystemXBar(width=64)
system.membus.clk_domain = system.cpu_clk_domain

CacheConfig.config_cache(options, system)
system.system_port = system.membus.cpu_side_ports

MemConfig.config_mem(options, system)

for i in range(len(system.mem_ctrls)):
    system.mem_ctrls[i].compressor = CompressConfig.get_compressor(options)
    system.mem_ctrls[i].compressor.size_threshold_percentage = 99
    system.mem_ctrls[i].metacompressor = CompressConfig.get_metacompressor(
        options
    )
    system.mem_ctrls[i].metacompressor.size_threshold_percentage = 99

for i in range(np):
    system.cpu[i].isStoreReqRecordEn = options.isStoreReqRecordEn
    system.cpu[i].objConfThres = options.objConfThres
    system.cpu[i].objIdenEntryNum = options.objIdenEntryNum

mp0_path = multiprocesses[0].executable
system.workload = SEWorkload.init_compatible(mp0_path)

root = Root(full_system=False, system=system)
Simulation.run(options, root, system, FutureClass)

# Simulation.save_checkpoint("/home/qshao/Project/IPDPS/checkpoint_template")
