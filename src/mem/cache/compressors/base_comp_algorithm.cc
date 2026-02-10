/*
 * Copyright (c) 2012-2013, 2016-2018 ARM Limited
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

#include "mem/cache/compressors/base_comp_algorithm.hh"

#include <algorithm>

#include "base/logging.hh"
#include "debug/BaseCompAlgorithm.hh"
#include "mem/cache/compressors/base.hh"
#include "params/BaseCompAlgorithm.hh"
#include "sim/sim_exit.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{
GEM5_DEPRECATED_NAMESPACE(Compressor, compression);
namespace compression
{
CompBlk::CompBlk()
{
    compData = nullptr;
    addr = (Addr) -1;
    compSize = (uint32_t) -1;
}

CompBlk::CompBlk(std::unique_ptr<Base::CompressionData> cpData_,
            Addr addr_, uint32_t compSize_):
            compData(std::move(cpData_)),
            addr(addr_),
            compSize(compSize_),
            valid(true),
            isZero(false)
{
}

CompBlk::~CompBlk()
{
    addr = (Addr) -1;
    compSize = (uint32_t) -1;
    if (compData!=nullptr) {
        compData.reset();
    }
    compData = nullptr;
    valid = false;
    isZero = false;
}

Base::CompressionData*
CompBlk::getCompData() {
    return compData.get();
}

bool
CompPage::isHitInCompPage(Addr addr) {
    // TODO
    // notice I just consider one blk here
    Addr blkIdx = (addr >> 8) & 63;
    if (blkArray[blkIdx]->valid) {
        return true;
    } else {
        return false;
    }
}

Base::CompressionData*
CompPage::getCompBlkData(Addr addr) {
    assert(!isHitInCompPage(addr) &&
    "Data is not hit in CompPage!");

    Addr blkIdx = (addr >> 8) & 63;
    return blkArray[blkIdx]->getCompData();
}

BaseCompAlgorithm::BaseCompAlgorithm(const Params &p)
    : ClockedObject(p),
      _name(name()+"Compress Algorithm"),
      updateEvent([this]{ update(); }, name()),
      stats(this)
{
    sendReqFailedTick = 0;

    duplicateDataFilter = false;
}

void
BaseCompAlgorithm::init()
{
    ClockedObject::init();
}


void
BaseCompAlgorithm::updateProcPages(Addr addr){
    Addr pageAddr = getPageAddr(addr);
    processingPages.insert(pageAddr);
}

void
BaseCompAlgorithm::releaseMemReqInfo()
{
    processingPages.clear();
}

void
BaseCompAlgorithm::releaseBlkReqInfo()
{
}

void
BaseCompAlgorithm::switchOut()
{
    if (updateEvent.scheduled())
        deschedule(updateEvent);
}

PacketPtr
BaseCompAlgorithm::peekPacket()
{
    if (reqPkts.empty())
        return nullptr;

    PacketPtr pkt = reqPkts.front();

    return pkt;
}

void
BaseCompAlgorithm::popPacket()
{
    PacketPtr pkt = reqPkts.front();
    reqPkts.pop_front();
    pendingPkts.push_back(pkt);
}

bool
BaseCompAlgorithm::pktsDone()
{
    return reqPkts.empty() && pendingPkts.empty();
}

PacketPtr
BaseCompAlgorithm::peekBlkedPacket()
{
    if (blkedPackets.empty())
        return nullptr;

    PacketPtr pkt = blkedPackets.back();

    return pkt;
}

void
BaseCompAlgorithm::popBlkedPacket()
{
    PacketPtr pkt = blkedPackets.back();
    blkedPackets.pop_back();
    pendingBlkedPkts.push_back(pkt);
}

bool
BaseCompAlgorithm::blkedPktsDone()
{
    return blkedPackets.empty() && pendingBlkedPkts.empty();
}

uint32_t
BaseCompAlgorithm::getCompressLength(PacketPtr pkt)
{
    if (compressor && pkt->hasData()) {
        Cycles compression_lat = Cycles(0);
        Cycles decompression_lat = Cycles(0);

        std::unique_ptr<Base::CompressionData> comp_data
        = compressor->compress(
        pkt->getConstPtr<uint64_t>(),
        compression_lat, decompression_lat);
        return comp_data->getSizeBits();
    }

    return (uint32_t) -1;
}

uint32_t
BaseCompAlgorithm::getDataCompressLength(void * dataPtr, uint32_t length)
{
    uint64_t* data64Ptr = (uint64_t*) dataPtr;

    Cycles compression_lat = Cycles(0);
    Cycles decompression_lat = Cycles(0);
    std::unique_ptr<Base::CompressionData> comp_data = compressor->compress(
    data64Ptr, compression_lat, decompression_lat);

    return comp_data->getSizeBits();
}

bool
BaseCompAlgorithm::recvResp(PacketPtr pkt, bool isAlgoMaster)
{
    stats.numResps+=1;

    Cycles compression_lat = Cycles(0);
    Cycles decompression_lat = Cycles(0);

    // If a compressor is being used, it is called to compress data before
    // insertion. Although in Gem5 the data is stored uncompressed, even if a
    // compressor is used, the compression/decompression methods are called to
    // calculate the amount of extra cycles needed to read or write compressed
    // blocks.
    if (compressor && pkt->hasData()) {
        std::unique_ptr<Base::CompressionData> comp_data =
            compressor->compress(pkt->getConstPtr<uint64_t>(),
                                 compression_lat, decompression_lat);
        uint32_t blk_size_bits = comp_data->getSizeBits();

        Addr pageAddr = getPageAddr(pkt->getAddr());
        Addr blkIdx = getBlkOffset(pkt->getAddr());

        if (compMem.count(pageAddr) == 0) {
            compMem[pageAddr].pageAddr = pageAddr;
        }

        CompBlk* blk = new CompBlk(
            nullptr, pkt->getAddr(), blk_size_bits);

        if (compMem[pageAddr].blkArray[blkIdx] != nullptr) {
            CompBlk* blk = camStoreData(pkt->getAddr());
            if (blk_size_bits != blk->compSize) {
            }
            delete compMem[pageAddr].blkArray[blkIdx];
        }
        compMem[pageAddr].blkArray[blkIdx] = blk;

        if (storeCompData) {
            // decompression check
            uint64_t decomp_data[blkSize/8];

            // Apply decompression
            compressor->decompress(comp_data.get(), decomp_data);

            // Check if decompressed line matches original cache line
            fatal_if(
                std::memcmp(pkt->getConstPtr<uint64_t>(),
                            decomp_data, blkSize),
                "Decompressed line does not match original line.");

            CompBlk* blk = new CompBlk(
                std::move(comp_data), pkt->getAddr(), blk_size_bits);
            compMem[pageAddr].blkArray[blkIdx] = blk;

            //blk.compData = std::move(comp_data);
        }
    }

    if (isAlgoMaster)
        packetDone(pkt);

    return true;
}

void
BaseCompAlgorithm::informZeroPage(Addr pageAddr) {
    compMem.erase(pageAddr);
}

void
BaseCompAlgorithm::packetDone(PacketPtr pkt) {
    //release allocate memory
    auto iter = std::find_if(
        pendingPkts.begin(), pendingPkts.end(),
        [&](const PacketPtr& pendingPkt) {
            return pendingPkt->getAddr() == pkt->getAddr();
        });

    if (iter!=pendingPkts.end()) {
        PacketPtr pendingPkt = *(iter);
        pendingPkt->deleteData();
        delete pendingPkt;
        pendingPkts.erase(iter);
    } else {
        assert(0 && "Resp should be found in pending pkts.\n");
    }
}

CompBlk*
BaseCompAlgorithm::camStoreData(Addr addr)
{
    Addr pageAddr = getPageAddr(addr);
    Addr blkIdx = getBlkOffset(addr);

    if (compMem.count(pageAddr) == 0)
        return nullptr;

    return compMem[pageAddr].blkArray[blkIdx];
}

bool
BaseCompAlgorithm::checkSameData(PacketPtr pkt)
{
    if (!compressor || !pkt->hasData())
       return false;

    // If no data is compressed, return compressed successfully
    if (!storeCompData) {
        return true;
    }

    CompBlk* blk = camStoreData(pkt->getAddr());
    if (!blk || !blk->valid) {
        return false;
    }

    uint64_t decomp_data[blkSize/8];

    // Apply decompression
    compressor->decompress((blk->compData).get(), decomp_data);

    const uint64_t* respDataPtr = pkt->getConstPtr<uint64_t>();

    auto chunk = compressor->toChunks(respDataPtr);

    for (uint32_t i = 0; i < blkSize/8; i++) {
        if (decomp_data[i]!= chunk[i]) {
            return false;
        }
    }

    return true;
}

void
BaseCompAlgorithm::setAlignBitSize(uint64_t alignBitSize_) {
    alignBitSize = alignBitSize_;
}

void
BaseCompAlgorithm::showCompRatio(uint32_t compBlkLength){

    uint64_t totalCompSize= 0;
    uint64_t singleCompSize = 0;
    uint64_t uncompressPageCnt = 0;

    uint64_t totalExceptNum = 0;

    uint64_t totalLCPCompSize = 0;

    for (auto iter = compMem.begin(); iter!=compMem.end(); iter++){

        std::map<int,int> compSizeRecord;

        compSizeRecord[1] = 0;

        for (auto i = 0; i < 7 ; i++) {
            compSizeRecord[(1 << i )*16] = 0;
        }

        std::stringstream ss;
        Addr pageAddr = iter->first;
        ss << "PageAddr " << std::hex << pageAddr << ":";


        uint64_t compressedSize = 0;

        std::map<uint32_t, uint32_t> compLengthDict;

        for (Addr addr = pageAddr; addr < pageAddr + 4096;
             addr += compBlkLength) {
            Addr blkIdx = getBlkOffset(addr);
            CompBlk* compBlk = compMem[pageAddr].blkArray[blkIdx];

            // Align block size to word
            if (compBlk->compSize % alignBitSize != 0) {
                compBlk->compSize =
                    ((compBlk->compSize / alignBitSize) + 1) * alignBitSize;
            }

            ss << std::setfill('0') << std::setw(2)
            << std::dec<< compBlk->compSize/8 << "|";

            compressedSize += compBlk->compSize;

            if (compLengthDict.count(compBlk->compSize) != 0) {
                compLengthDict[compBlk->compSize] =
                compLengthDict[compBlk->compSize] + 1;
            } else {
                compLengthDict[compBlk->compSize] = 1;
            }

            if (compBlk->isZero) {
                compSizeRecord[1]++;
                continue;
            }

            uint64_t blkSize = 1;
            while (compBlk->compSize > (blkSize << 4)) {
                blkSize = blkSize << 1;
            }

            compSizeRecord[blkSize<<4]++;
        }

        // Most Frequent Length
        std::map<uint32_t, uint32_t>::iterator mostFrequentLength =
            std::max_element(
                compLengthDict.begin(), compLengthDict.end(),
                [](const std::pair<uint32_t, uint32_t>& a,
                   const std::pair<uint32_t, uint32_t>& b) {
                    return a.second < b.second;
                });
        totalExceptNum += (64-mostFrequentLength->second);

        // size when store the most and left as exception
        totalLCPCompSize +=
            (mostFrequentLength->second * mostFrequentLength->first) +
            512 * (64 - mostFrequentLength->second);

        std::stringstream ssFreq;
        ssFreq << "MostFreqBlkLength " << mostFrequentLength->first
               << " counter " << mostFrequentLength->second;

        totalCompSize += compressedSize;

        double compRatio = double(4096*8)/compressedSize;
        std::stringstream ssRecord;
        ssRecord << "Record of " << std::hex << pageAddr << ":"
                 << std::dec << compRatio << " # ";
        for (auto iterRecord = compSizeRecord.begin();
             iterRecord != compSizeRecord.end(); iterRecord++) {
            ssRecord << std::dec << iterRecord->first << ":"
                     << iterRecord->second << "|";
        }

        uint64_t maxCompRatioCnt = 0;
        uint64_t maxCompRatio = 0;
        for (auto iterRecord = compSizeRecord.begin();
        iterRecord!=compSizeRecord.end(); iterRecord++) {
            if (iterRecord->second > maxCompRatioCnt) {
                maxCompRatioCnt = iterRecord->second;
                maxCompRatio = iterRecord->first;
            }
        }

        uint64_t singlePageCompSize = maxCompRatio*maxCompRatioCnt +
        (4096/compBlkLength-maxCompRatioCnt)*compBlkLength*8;
        singleCompSize += singlePageCompSize;

        if (singlePageCompSize == 4096*8)
            uncompressPageCnt +=1;
    }

    double exceptionRatio = double(totalExceptNum*100)/(compMem.size()*64);
    double singleCompRatio= double(4096*8*compMem.size())/singleCompSize;
    double totalCompRatio= double(4096*8*compMem.size())/totalCompSize;
    double totalLCPCompRatio= double(4096*8*compMem.size())/totalLCPCompSize;

    double footprint = compMem.size()*4/1024;


    DPRINTF(BaseCompAlgorithm,
        "Total Comp Ratio: %f, Single Comp Ratio: %f, Exception Ratio: %f, "
        "Total LCP Comp Ratio: %f, Footprint: %f GB\n",
        totalCompRatio,
        singleCompRatio,
        exceptionRatio,
        totalLCPCompRatio,
        footprint);

}

void
BaseCompAlgorithm::genReadReq(Addr startAddr, Addr endAddr, unsigned blockSize)
{
    Addr pageAddr = getPageAddr(startAddr);

    /** check whether this page has been compressed */
    if (duplicateDataFilter) {
        if (compRecord.count(pageAddr) == 1)
            return;
        else
            compRecord.insert(pageAddr);
    }

    for (Addr addr = startAddr; addr < endAddr; addr+=blockSize) {
        PacketPtr newPkt = createPacket(addr, blockSize, MemCmd::ReadReq);
        reqPkts.push_back(newPkt);
    }

    if (!updateEvent.scheduled()) {
        schedule(updateEvent, curTick());
    }
}

PacketPtr
BaseCompAlgorithm::createPacket(Addr addr, unsigned size, const MemCmd& cmd,
                   uint8_t* pkt_data, Request::FlagsType flags)
{
    // Create new request
    RequestPtr req = std::make_shared<Request>(addr, size, flags,
                                               requestorId);
    // Dummy PC to have PC-based prefetchers latch on; get entropy into higher
    // bits
    req->setPC(((Addr)requestorId) << 2);

    // Embed it in a packet
    PacketPtr pkt = new Packet(req, cmd);

    if (pkt_data != nullptr) {
        pkt->dataDynamic(pkt_data);
    } else {
        pkt_data = new uint8_t[req->getSize()];
        pkt->dataDynamic(pkt_data);

        if (cmd.isWrite()) {
            std::fill_n(pkt_data, req->getSize(), (uint8_t)requestorId);
        }
    }

    return pkt;
}

void
BaseCompAlgorithm::update() {
}

bool
BaseCompAlgorithm::isCmdReqHitMemReq(PacketPtr pkt) {
    Addr addr = pkt->getAddr();

    auto iter_v = std::find_if(pendingPkts.begin(), pendingPkts.end(),
    [&](const PacketPtr& pendingPkt){return (pendingPkt->getAddr() == addr);});

    if (iter_v !=pendingPkts.end())
        return true;

    auto iter_d = std::find_if(reqPkts.begin(), reqPkts.end(),
    [&](const PacketPtr& reqPkt){return (reqPkt->getAddr() == addr);});

    if (iter_d!=reqPkts.end())
        return true;
    else
        return false;
}

bool
BaseCompAlgorithm::isCmdReqHitPages(PacketPtr pkt) {
    Addr addr = pkt->getAddr();

    Addr pageAddr = getPageAddr(addr);

    auto iter = std::find_if(processingPages.begin(), processingPages.end(),
    [&](const Addr& processingPage){return (processingPage == pageAddr);});

    if (iter!=processingPages.end())
        return true;
    else
        return false;
}

void
BaseCompAlgorithm::recvBlkedPacket(PacketPtr pkt) {
    blkedPackets.push_back(pkt);
}


BaseCompAlgorithm::StatGroup::StatGroup(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numReads, statistics::units::Count::get(),
               "Number of read request"),
      ADD_STAT(numResps, statistics::units::Count::get(),
               "Number of read resp"),
      ADD_STAT(numRetries, statistics::units::Count::get(),
               "Number of retry")
{
}
}
} // namespace gem5
