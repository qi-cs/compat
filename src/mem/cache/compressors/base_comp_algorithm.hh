/*
 * Copyright (c) 2012-2013, 2017-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed here under.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Declaration of the base generator class for all generators.
 */

#ifndef __BASE_COMPRESSION_ALGORITHM__
#define __BASE_COMPRESSION_ALGORITHM__

#include <cstdint>
#include <string>

#include "base/logging.hh"
#include "base/types.hh"
#include "mem/cache/compressors/base.hh"
#include "mem/packet.hh"
#include "mem/packet_queue.hh"
#include "mem/port.hh"
#include "mem/qport.hh"
#include "mem/request.hh"
#include "params/BaseCompAlgorithm.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_exit.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{
GEM5_DEPRECATED_NAMESPACE(Compressor, compression);

namespace compression

{

class SimObject;
class System;
struct Params;

class CompBlk
{
  public:
    std::unique_ptr<Base::CompressionData> compData;

    //TODO delete this is redundant
    Addr addr;

    uint32_t compSize;

    bool valid;

    bool isZero;
  public:
    CompBlk();

    CompBlk(std::unique_ptr<Base::CompressionData> cpData_,
            Addr addr_, uint32_t compSize_);

    ~CompBlk();

    Base::CompressionData* getCompData();

};

class CompPage
{
  public:
    std::array<CompBlk*, 64> blkArray;
    Addr pageAddr;

  public:
    bool isHitInCompPage(Addr addr);

    Base::CompressionData* getCompBlkData(Addr addr);

};

/**
 * Base class for all generators, with the shared functionality and
 * virtual functions for entering, executing and leaving the
 * generator.
 */
class BaseCompAlgorithm  : public ClockedObject
{

  protected:

    /** Name to use for status and debug printing */
    const std::string _name;

    /** The RequestorID used for generating requests */
    //const RequestorID requestorId;
    RequestorID requestorId;

    std::deque<PacketPtr> reqPkts;
    std::vector<PacketPtr> pendingPkts;

    Tick sendReqFailedTick = 0;

    /** Event for scheduling updates */
    EventFunctionWrapper updateEvent;

    bool storeCompData = false;

    /** check req by compRecord to avoid
    *   sending duplicate request
    */
    bool duplicateDataFilter = false;

    uint64_t blkSize = 64;

    // Align Size, by default, it is just byte
    uint64_t alignBitSize = 8;

    /** Compression method being used. */
    compression::Base* compressor;

  public:
    /** map from page addr to compressed page */
    std::map<Addr,CompPage> compMem;

  protected:
    /** data has been compressed **/

    std::set<Addr> compRecord;

    /** pages that are being processed by compressor
    *   cpu req can cam this and get blked
    */
    std::set<Addr> processingPages;

    std::deque<PacketPtr> blkedPackets;

    std::vector<PacketPtr> pendingBlkedPkts;

  public:
    /**
     * Create a base generator.
     *
     * @param obj simobject owning the generator
     * @param requestor_id RequestorID set on each request
     */

    BaseCompAlgorithm(const Params &p);

    ~BaseCompAlgorithm() { }

    void init() override;

    void updateProcPages(Addr addr);

    void releaseMemReqInfo();

    void releaseBlkReqInfo();

    void setCompressor(compression::Base* compressor_) {
        compressor = compressor_;
    }

    void setCompStoreData(bool storeCompData_) {
        storeCompData = storeCompData_;
    }

    void setRequestorId(RequestorID requestorId_) {
        requestorId = requestorId_;
    }

    /**
     * Get the name, useful for DPRINTFs.
     *
     * @return the given name
     */
    std::string getName() const { return _name; }

    void switchOut();

    /**
     *   check compress pkt
    */
    PacketPtr peekPacket();

    void popPacket();

    bool pktsDone();

    /**
     *   check normal req pkt
    */
    PacketPtr peekBlkedPacket();

    void popBlkedPacket();

    bool blkedPktsDone();

    uint32_t getCompressLength(PacketPtr pkt);

    uint32_t getDataCompressLength(void * dataPtr, uint32_t length=8);

    bool recvResp(PacketPtr pkt, bool isAlgoMaster = true);

    void packetDone(PacketPtr pkt);

    CompBlk* camStoreData(Addr addr);

    bool checkSameData(PacketPtr pkt);

    Addr getPageAddr(Addr addr){
        return (addr >> 12) << 12;
    }

    Addr getBlkOffset(Addr addr) {
        return (addr >> 6) & 63;
    }

    void setAlignBitSize(uint64_t alignBitSize_);

    void showCompRatio(uint32_t compBlkLength = 64);

    void genReadReq(Addr startAddr, Addr endAddr, unsigned blockSize = 64);

    /**
     * Generate a new request and associated packet
     *
     * @param addr Physical address to use
     * @param size Size of the request
     * @param cmd Memory command to send
     * @param flags Optional request flags
     */
    PacketPtr createPacket(Addr addr, unsigned size, const MemCmd& cmd,
                        uint8_t* pkt_data = nullptr,
                        Request::FlagsType flags = Request::UNCACHEABLE);

    void informZeroPage(Addr pageAddr);

    void update();

    bool isCmdReqHitMemReq(PacketPtr pkt);

    bool isCmdReqHitPages(PacketPtr pkt);

    void recvBlkedPacket(PacketPtr pkt);

    struct StatGroup : public statistics::Group
    {
        StatGroup(statistics::Group *parent);

        statistics::Scalar numReads;

        statistics::Scalar numResps;

        statistics::Scalar numRetries;
    } stats;
};

}
} // namespace gem5

#endif //BASE_COMPRESSION_ALGORITHM
