//===-- Memory.cpp --------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Memory.h"

#include "Context.h"
#include "klee/Expr.h"
#include "klee/Solver.h"
#include "klee/util/BitArray.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/util/ArrayCache.h"

#include "ObjectHolder.h"
#include "MemoryManager.h"

#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Value.h"

#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

#include <cassert>
#include <sstream>
#include <iostream>

using namespace llvm;
using namespace klee;

namespace {
  cl::opt<bool>
  UseConstantArrays("use-constant-arrays",
                    cl::init(true));
}

//TASE Additions
uint16_t poison_val = 0xDEAD;
extern "C" bool tase_buf_has_taint(void * ptr, int size);

/***/

ObjectHolder::ObjectHolder(const ObjectHolder &b) : os(b.os) { 
  if (os) ++os->refCount; 
}

ObjectHolder::ObjectHolder(ObjectState *_os) : os(_os) { 
  if (os) ++os->refCount; 
}

ObjectHolder::~ObjectHolder() { 
  if (os && --os->refCount==0) delete os; 
}
  
ObjectHolder &ObjectHolder::operator=(const ObjectHolder &b) {
  if (b.os) ++b.os->refCount;
  if (os && --os->refCount==0) delete os;
  os = b.os;
  return *this;
}

/***/



int MemoryObject::counter = 0;

MemoryObject::~MemoryObject() {
  if (parent)
    parent->markFreed(this);
}

void MemoryObject::getAllocInfo(std::string &result) const {
  llvm::raw_string_ostream info(result);

  info << "MO" << id << "[" << size << "]";

  if (allocSite) {
    info << " allocated at ";
    if (const Instruction *i = dyn_cast<Instruction>(allocSite)) {
      info << i->getParent()->getParent()->getName() << "():";
      info << *i;
    } else if (const GlobalValue *gv = dyn_cast<GlobalValue>(allocSite)) {
      info << "global:" << gv->getName();
    } else {
      info << "value:" << *allocSite;
    }
  } else {
    info << " (no allocation info)";
  }
  
  info.flush();
}

/***/


//ABH: Added "forTASE" arg to avoid an unnecessary malloc for some
//use cases, ex tase_map_buf.
ObjectState::ObjectState(const MemoryObject *mo, bool forTASE)
  : refCount(0),
    object(mo),
    concreteStore(forTASE ? NULL : new uint8_t[mo->size]),
    concreteMask(0),
    flushMask(0),
    knownSymbolics(0),
    updates(0, 0),
    size(mo->size),
    readOnly(false) {
  mo->refCount++;
  if (!UseConstantArrays) {
    static unsigned id = 0;
    const Array *array =
        getArrayCache()->CreateArray("tmp_arr" + llvm::utostr(++id), size);
    updates = UpdateList(array, 0);
  }
  
  if (!forTASE) 
    memset(concreteStore, 0, size);
  
}

//ABH: Added "forTASE" arg to avoid an unnecessary malloc for some
//use cases
ObjectState::ObjectState(const MemoryObject *mo, const Array *array, bool forTASE)
  : refCount(0),
    object(mo),
    concreteStore(forTASE ? NULL : new uint8_t[mo->size]),
    concreteMask(0),
    flushMask(0),
    knownSymbolics(0),
    updates(array, 0),
    size(mo->size),
    readOnly(false) {
  mo->refCount++;
  //AH Changed  memset to 0 before call to makeSymbolic.
  if (!forTASE) {
    memset(concreteStore, 0, size);
    makeSymbolic();
  } else {
    concreteStore = (uint8_t *) mo->address;
    memset(concreteStore, 0, size);
    makeSymbolic();
  }
}

ObjectState::ObjectState(const ObjectState &os) 
  : refCount(0),
    object(os.object),
    concreteStore(new uint8_t[os.size]),
    concreteMask(os.concreteMask ? new BitArray(*os.concreteMask, os.size) : 0),
    flushMask(os.flushMask ? new BitArray(*os.flushMask, os.size) : 0),
    knownSymbolics(0),
    updates(os.updates),
    size(os.size),
    readOnly(false) {
  assert(!os.readOnly && "no need to copy read only object?");
  if (object)
    object->refCount++;

  if (os.knownSymbolics) {
    knownSymbolics = new ref<Expr>[size];
    for (unsigned i=0; i<size; i++)
      knownSymbolics[i] = os.knownSymbolics[i];
  }

  memcpy(concreteStore, os.concreteStore, size*sizeof(*concreteStore));
}

ObjectState::~ObjectState() {
  delete concreteMask;
  delete flushMask;
  delete[] knownSymbolics;
  //Removed the call to delete for TASE because
  //we're backing persistent parts of the address space (e.g., globals
  //and the stack) with object states.  This is probably causing memory
  //leaks for some temporary object state allocations, so we should
  //possibly add a "forTASE" arg to disambiguate those cases.
  //delete[] concreteStore;

  if (object)
  {
    assert(object->refCount > 0);
    object->refCount--;
    if (object->refCount == 0)
    {
      delete object;
    }
  }
}

ArrayCache *ObjectState::getArrayCache() const {
  assert(object && "object was NULL");
  return object->parent->getArrayCache();
}

/***/

const UpdateList &ObjectState::getUpdates() const {
  // Constant arrays are created lazily.
  if (!updates.root) {
    // Collect the list of writes, with the oldest writes first.
    
    // FIXME: We should be able to do this more efficiently, we just need to be
    // careful to get the interaction with the cache right. In particular we
    // should avoid creating UpdateNode instances we never use.
    unsigned NumWrites = updates.head ? updates.head->getSize() : 0;
    std::vector< std::pair< ref<Expr>, ref<Expr> > > Writes(NumWrites);
    const UpdateNode *un = updates.head;
    for (unsigned i = NumWrites; i != 0; un = un->next) {
      --i;
      Writes[i] = std::make_pair(un->index, un->value);
    }

    std::vector< ref<ConstantExpr> > Contents(size);

    // Initialize to zeros.
    for (unsigned i = 0, e = size; i != e; ++i)
      Contents[i] = ConstantExpr::create(0, Expr::Int8);

    // Pull off as many concrete writes as we can.
    unsigned Begin = 0, End = Writes.size();
    for (; Begin != End; ++Begin) {
      // Push concrete writes into the constant array.
      ConstantExpr *Index = dyn_cast<ConstantExpr>(Writes[Begin].first);
      if (!Index)
        break;

      ConstantExpr *Value = dyn_cast<ConstantExpr>(Writes[Begin].second);
      if (!Value)
        break;

      Contents[Index->getZExtValue()] = Value;
    }

    static unsigned id = 0;
    const Array *array = getArrayCache()->CreateArray(
        "const_arr" + llvm::utostr(++id), size, &Contents[0],
        &Contents[0] + Contents.size());
    updates = UpdateList(array, 0);

    // Apply the remaining (non-constant) writes.
    for (; Begin != End; ++Begin)
      updates.extend(Writes[Begin].first, Writes[Begin].second);
  }

  return updates;
}

void ObjectState::makeConcrete() {
  delete concreteMask;
  delete flushMask;
  delete[] knownSymbolics;
  concreteMask = 0;
  flushMask = 0;
  knownSymbolics = 0;
}

void ObjectState::makeSymbolic() {
  assert(!updates.head &&
         "XXX makeSymbolic of objects with symbolic values is unsupported");

  // XXX simplify this, can just delete various arrays I guess
  for (unsigned i=0; i<size; i++) {
    markByteSymbolic(i);
    setKnownSymbolic(i, 0);
    markByteFlushed(i);
  }
}

void ObjectState::applyPsnOnMakeSymbolic() {

  //Sanity Checks
  uint64_t addr = this->getObject()->address;
  unsigned size = this->getObject()->size;
  if (addr % 2 != 0) {
    printf("ERROR: Encountered memory object on unaligned address 0x%lx \n", addr);
    std::cout.flush();
    std::exit(EXIT_FAILURE);
  }
  if (size %2 != 0) {
    printf("ERROR: Encountered memory object with odd size %u \n", size);
    std::cout.flush();
    std::exit(EXIT_FAILURE);
  }


  //Apply the poison
  uint16_t * psnPtr = (uint16_t *) addr;
  for (int i = 0; i < size/2;  i++)
    *(psnPtr + i) = poison_val;
  
}

void ObjectState::initializeToZero() {
  makeConcrete();
  memset(concreteStore, 0, size);
}

void ObjectState::initializeToRandom() {  
  makeConcrete();
  for (unsigned i=0; i<size; i++) {
    // randomly selected by 256 sided die
    concreteStore[i] = 0xAB;
  }
}

/*
Cache Invariants
--
isByteKnownSymbolic(i) => !isByteConcrete(i)
isByteConcrete(i) => !isByteKnownSymbolic(i)
!isByteFlushed(i) => (isByteConcrete(i) || isByteKnownSymbolic(i))

Extra TASE Invariants:
1:   !isBytePoison(i) => isByteConcrete(i)  && !isByteKnownSymbolic(i) && !isByteFlushed(i)
Converse isn't true.  Contrapositive is useful to think about.
!isByteConcrete(i) || isByteKnowSymbolic(i) || isByteFlushed(i) => isBytePoison(i)

2:   isByteConcrete(i) => !isByteFlushed(i)
We need this to hold because reads and writes during native
execution have no ability to update the isByteFlushed(i) indicator.

If this invariant did not hold, writes to bytes that were concrete (isByteConcrete(i))
and flushed (isByteFlushed(i)) would not correctly set the byte to unflushed (!isByteflushed(i))
and subsequent flushing operations in the interpreter would incorrectly fail to flush the updated
byte at index i (ex during flushRangeForRead).

 */

void ObjectState::fastRangeCheckOffset(ref<Expr> offset,
                                       unsigned *base_r,
                                       unsigned *size_r) const {
  *base_r = 0;
  *size_r = size;
}

void ObjectState::flushRangeForRead(unsigned rangeBase, 
                                    unsigned rangeSize) const {
  if (!flushMask) flushMask = new BitArray(size, true);
 
  for (unsigned offset=rangeBase; offset<rangeBase+rangeSize; offset++) {
    if (!isByteFlushed(offset)) {
      if (isByteConcrete(offset)) {
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       ConstantExpr::create(concreteStore[offset], Expr::Int8));
      } else {
        assert(isByteKnownSymbolic(offset) && "invalid bit set in flushMask");
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       knownSymbolics[offset]);
      }

      flushMask->unset(offset);
    }
  } 
}

void ObjectState::flushRangeForWrite(unsigned rangeBase, 
                                     unsigned rangeSize) {
  if (!flushMask) flushMask = new BitArray(size, true);

  for (unsigned offset=rangeBase; offset<rangeBase+rangeSize; offset++) {
    if (!isByteFlushed(offset)) {
      if (isByteConcrete(offset)) {
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       ConstantExpr::create(concreteStore[offset], Expr::Int8));
        markByteSymbolic(offset);
      } else {
        assert(isByteKnownSymbolic(offset) && "invalid bit set in flushMask");
        updates.extend(ConstantExpr::create(offset, Expr::Int32),
                       knownSymbolics[offset]);
        setKnownSymbolic(offset, 0);
      }

      flushMask->unset(offset);
    } else {
      // flushed bytes that are written over still need
      // to be marked out
      if (isByteConcrete(offset)) {
        markByteSymbolic(offset);
      } else if (isByteKnownSymbolic(offset)) {
        setKnownSymbolic(offset, 0);
      }
    }
  } 
}

bool ObjectState::isByteConcrete(unsigned offset) const {
  return !concreteMask || concreteMask->get(offset);
}

bool ObjectState::isByteFlushed(unsigned offset) const {
  return flushMask && !flushMask->get(offset);
}

bool ObjectState::isByteKnownSymbolic(unsigned offset) const {
  return knownSymbolics && knownSymbolics[offset].get();
}

void ObjectState::markByteConcrete(unsigned offset) {
  if (concreteMask)
    concreteMask->set(offset);
}

void ObjectState::markByteSymbolic(unsigned offset) {
  if (!concreteMask)
    concreteMask = new BitArray(size, true);
  concreteMask->unset(offset);
}

void ObjectState::markByteUnflushed(unsigned offset) {
  if (flushMask)
    flushMask->set(offset);
}

void ObjectState::markByteFlushed(unsigned offset) {
  if (!flushMask) {
    flushMask = new BitArray(size, false);
  } else {
    flushMask->unset(offset);
  }
}

void ObjectState::setKnownSymbolic(unsigned offset, 
                                   Expr *value /* can be null */) {
  if (knownSymbolics) {
    knownSymbolics[offset] = value;
  } else {
    if (value) {
      knownSymbolics = new ref<Expr>[size];
      knownSymbolics[offset] = value;
    }
  }
}

/***/

ref<Expr> ObjectState::read8(unsigned offset) const {
  if (isByteConcrete(offset)) {
    return ConstantExpr::create(concreteStore[offset], Expr::Int8);
  } else if (isByteKnownSymbolic(offset)) {
    return knownSymbolics[offset];
  } else {
    assert(isByteFlushed(offset) && "unflushed byte without cache value");
    
    return ReadExpr::create(getUpdates(), 
                            ConstantExpr::create(offset, Expr::Int32));
  }    
}

ref<Expr> ObjectState::read8(ref<Expr> offset) const {
  assert(!isa<ConstantExpr>(offset) && "constant offset passed to symbolic read8");
  unsigned base, size;
  fastRangeCheckOffset(offset, &base, &size);
  flushRangeForRead(base, size);

  if (size>4096) {
    std::string allocInfo;
    object->getAllocInfo(allocInfo);
    klee_warning_once(0, "flushing %d bytes on read, may be slow and/or crash: %s", 
                      size,
                      allocInfo.c_str());
  }
  
  return ReadExpr::create(getUpdates(), ZExtExpr::create(offset, Expr::Int32));
}

void ObjectState::write8(unsigned offset, uint8_t value) {
  //assert(read_only == false && "writing to read-only object!");
 
  concreteStore[offset] = value;
  setKnownSymbolic(offset, 0);

  markByteConcrete(offset);
  markByteUnflushed(offset);
}

void ObjectState::write8AsExpr(unsigned offset, ref<Expr> value) {

  setKnownSymbolic(offset, value.get());
      
  markByteSymbolic(offset);
  markByteUnflushed(offset);
  
}

void ObjectState::write8(unsigned offset, ref<Expr> value) {
  // can happen when ExtractExpr special cases
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(value)) {
    write8(offset, (uint8_t) CE->getZExtValue(8));
  } else {
    setKnownSymbolic(offset, value.get());
      
    markByteSymbolic(offset);
    markByteUnflushed(offset);
  }
}

void ObjectState::write8(ref<Expr> offset, ref<Expr> value) {
  assert(!isa<ConstantExpr>(offset) && "constant offset passed to symbolic write8");
  unsigned base, size;
  fastRangeCheckOffset(offset, &base, &size);
  flushRangeForWrite(base, size);

  if (size>4096) {
    std::string allocInfo;
    object->getAllocInfo(allocInfo);
    klee_warning_once(0, "flushing %d bytes on read, may be slow and/or crash: %s", 
                      size,
                      allocInfo.c_str());
  }
  
  updates.extend(ZExtExpr::create(offset, Expr::Int32), value);
}

/***/

ref<Expr> ObjectState::read(ref<Expr> offset, Expr::Width width) const {
  // Truncate offset to 32-bits.
  offset = ZExtExpr::create(offset, Expr::Int32);

  // Check for reads at constant offsets.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(offset))
    return read(CE->getZExtValue(32), width);

  // Treat bool specially, it is the only non-byte sized write we allow.
  if (width == Expr::Bool)
    return ExtractExpr::create(read8(offset), 0, Expr::Bool);

  // Otherwise, follow the slow general case.
  unsigned NumBytes = width / 8;
  assert(width == NumBytes * 8 && "Invalid read size!");
  ref<Expr> Res(0);
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    ref<Expr> Byte = read8(AddExpr::create(offset, 
                                           ConstantExpr::create(idx, 
                                                                Expr::Int32)));
    Res = i ? ConcatExpr::create(Byte, Res) : Byte;
  }

  return Res;
}

ref<Expr> ObjectState::read(unsigned offset, Expr::Width width) const {
  // Treat bool specially, it is the only non-byte sized write we allow.
  if (width == Expr::Bool)
    return ExtractExpr::create(read8(offset), 0, Expr::Bool);

  // Otherwise, follow the slow general case.
  unsigned NumBytes = width / 8;
  assert(width == NumBytes * 8 && "Invalid width for read size!");

  //Fast path for tase ---------------------------------
  //If we have an entirely concrete buffer backing the read, we just
  //read it and directly return the constant expr instead of the
  //roundabout method of alloc'ing byte-by-byte and concating the result.
  if ((NumBytes == 2 || NumBytes == 4 || NumBytes == 8) && (!tase_buf_has_taint((void *) (&(concreteStore[offset])), NumBytes)) ) {
    if (NumBytes == 2) {
      uint16_t val = * ((uint16_t *) &(concreteStore[offset]));
      return ConstantExpr::create(val, Expr::Int16);
    } else if (NumBytes == 4) {
      uint32_t val = * ((uint32_t *) &(concreteStore[offset])); 
      return ConstantExpr::create(val, Expr::Int32);
    }
    else if (NumBytes == 8) {
      uint64_t val = * ((uint64_t *) &(concreteStore[offset]));
      return ConstantExpr::create(val, Expr::Int64);
    }
  }
  //-------------------------------------------------------
  
  ref<Expr> Res(0);
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    ref<Expr> Byte = read8(offset + idx);
    Res = i ? ConcatExpr::create(Byte, Res) : Byte;
  }

  return Res;
}

void ObjectState::applyPsnOnRead(ref<Expr> offset) {

  if (ConstantExpr * CE = dyn_cast<ConstantExpr>(offset)) {
    //Nothing to do -- no state changes to isByteFlushed, isByteConcrete, isKnownSymbolic. 
  } else {
    //We have a read at a symbolic offset.
    //Need to ensure invariant and mark bytes as unflushed.

    //We can mark the bytes as unflushed by "safely" reading/writing the value for
    //each byte.
    for (unsigned i = 0; i < size; i++) {
      ref<Expr> byteVal = read8(i);
      write8AsExpr(i, byteVal);
    }
  }
}

void ObjectState::applyPsnOnWrite(ref<Expr> offset, ref<Expr> value) {

 // Truncate offset to 32-bits.
  offset = ZExtExpr::create(offset, Expr::Int32);

  // Check for writes at constant offsets.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(offset)) {
    unsigned firstOff = CE->getZExtValue(32);
    Expr::Width valWidth = value->getWidth();

    unsigned endOff;
    switch (valWidth) {
    default: assert(0 && "Invalid write size!");
    case  Expr::Bool:
    case  Expr::Int8:

      endOff = 1;
      break;
    case Expr::Int16: endOff = 2;
      break;
    case Expr::Int32: endOff = 4;
      break;
    case Expr::Int64: endOff = 8;
      break;
    }

    endOff = firstOff + endOff -1;
    
    uint64_t firstAddr = this->getObject()->address + (uint64_t) firstOff;
    uint64_t endAddr = this->getObject()->address + (uint64_t) endOff;
    bool concVal = false;    
    if(ConstantExpr * CE_Value = dyn_cast<ConstantExpr>(value)) {
      concVal = true;
    }

    bool twoByteAligned = false;

    if (firstAddr %2 == 0 && endAddr %2 == 1)
      twoByteAligned = true;

    //Deal with "middle"
    if (concVal == false) {
      if (valWidth == Expr::Int8 || valWidth == Expr::Bool) {
	//Fall through to "ends" case
      } else if (valWidth == Expr::Int16) {
	if (twoByteAligned) 
	  *(uint16_t *) firstAddr = poison_val;
	//If not twoByteAligned, fall through to the "ends" case
      } else if (valWidth == Expr::Int32) {
	if (twoByteAligned) {
	  ref <Expr> b1 = read8(firstOff);
	  ref <Expr> b2 = read8(firstOff + 1);
	  write8AsExpr(firstOff,b1);
	  write8AsExpr(firstOff+1 ,b2);
	  *(uint16_t *) firstAddr = poison_val;

	  ref <Expr> b3 = read8(firstOff + 2);
	  ref <Expr> b4 = read8(firstOff + 3);
	  write8AsExpr(firstOff + 2, b3);
	  write8AsExpr(firstOff + 3, b4);
	  *((uint16_t *) firstAddr +1) = poison_val;
	} else {
	  uint64_t tempAddr =  firstAddr +1;
	  ref <Expr> b2 = read8(firstOff +1);
	  ref <Expr> b3 = read8(firstOff +2);
	  write8AsExpr(firstOff + 1, b2);
	  write8AsExpr(firstOff + 2, b3);
	  *(uint16_t *) tempAddr = poison_val;
	}
      } else if (valWidth == Expr::Int64) {
      	if (twoByteAligned) {
	  uint16_t * tempAddr = (uint16_t *) firstAddr;
	  for (int i = 0; i < 4; i++) {
	    ref<Expr> b_first = read8(firstOff + 2*i);
	    ref<Expr> b_second = read8(firstOff + 2*i + 1);
	    write8AsExpr(firstOff + 2*i, b_first);
	    write8AsExpr(firstOff + 2*i + 1, b_second);
	    *(tempAddr +i) = poison_val;

	  }
	} else {
	  uint64_t tmp = (uint64_t) firstAddr +1;
	  uint16_t * tempAddr = (uint16_t *) tmp; 
	  for (int i = 0; i < 3; i++){
	    ref<Expr> b_first = read8(firstOff + 2*i + 1);
	    ref<Expr> b_second = read8(firstOff + 2*i + 2);
	    write8AsExpr(firstOff + 2*i + 1, b_first);
	    write8AsExpr(firstOff + 2*i + 2, b_second);
	    *(tempAddr +i) = poison_val;
	  }
	}
      } else {
	printf("ERROR: Unknown expression size in ApplyPsnOnWrite \n");
	std::cout.flush();
      }
    }
      
    //Deal with "ends"
    
    if ( firstAddr %2 == 1) {
      unsigned sibIdx = firstOff -1;
      if (! (isByteConcrete(firstOff) && !isByteFlushed(firstOff) && isByteConcrete(sibIdx) && !isByteFlushed(sibIdx))) {
	ref <Expr> byte1Val = read8(sibIdx);
	ref <Expr> byte2Val = read8(firstOff);
	write8AsExpr( sibIdx, byte1Val);
	write8AsExpr( firstOff, byte2Val);
	
	*(uint16_t *) (firstAddr -1) = poison_val;
	
      } else {
	//No action needed.  Both bytes are "nice"
      }	  
    }
    
    if (endAddr %2 == 0) {
      unsigned sibIdx = endOff +1;
      if (! ( isByteConcrete(endOff) && !isByteFlushed(endOff) && isByteConcrete(sibIdx) && !isByteFlushed(sibIdx) ) ) {
	ref <Expr> byte1Val = read8(endOff);
	ref <Expr> byte2Val = read8(sibIdx);
	write8AsExpr( endOff, byte1Val);
	write8AsExpr(sibIdx, byte2Val);
	*(uint16_t *) endAddr = poison_val;
	
      } else {
	//No action needed.  Both bytes are "nice"
      }
    }
  } else {
    //Write at symbolic offset
    //In this case, the whole objectstate will get flushed and all bytes marked symbolic
    // (See flushRangeForWrite)

    uint16_t * twoByteIterator = (uint16_t *) this->getObject()->address;
    if (size % 2 != 0 || (this->getObject()->address %2 != 0)) {
      printf("Memory Object unaligned or with odd number of bytes \n");
      std::cout.flush();
      std::exit(EXIT_FAILURE);
    }
    
    for (int i = 0; i < size/2; i++)
      *(twoByteIterator + i) = poison_val; 
  }


}

bool ObjectState::isObjectEntirelyConcrete() {

  for (int j = 0; j < this->size ; j++) {
    if (!isByteConcrete(j))
      return false;
  }
  return true;

}


void ObjectState::write(ref<Expr> offset, ref<Expr> value) {
  // Truncate offset to 32-bits.
  offset = ZExtExpr::create(offset, Expr::Int32);

  
  // Check for writes at constant offsets.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(offset)) {
    write(CE->getZExtValue(32), value);
    
    return;
  }

  // Treat bool specially, it is the only non-byte sized write we allow.
  Expr::Width w = value->getWidth();
  if (w == Expr::Bool) {
    write8(offset, ZExtExpr::create(value, Expr::Int8));
    return;
  }

  // Otherwise, follow the slow general case.
  unsigned NumBytes = w / 8;
  assert(w == NumBytes * 8 && "Invalid write size!");
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(AddExpr::create(offset, ConstantExpr::create(idx, Expr::Int32)),
           ExtractExpr::create(value, 8 * i, Expr::Int8));
  }
}

void ObjectState::write(unsigned offset, ref<Expr> value) {
  // Check for writes of constant values.

  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(value)) {
    Expr::Width w = CE->getWidth();
    if (w <= 64 && klee::bits64::isPowerOfTwo(w)) {

      uint64_t val = CE->getZExtValue();
      switch (w) {
      default:  printf("Invalid write size \n"); fflush(stdout); assert(0 && "Invalid write size!");
      case  Expr::Bool:
      case  Expr::Int8:  write8(offset, val); return;
      case Expr::Int16: write16(offset, val); return;
      case Expr::Int32: write32(offset, val); return;
      case Expr::Int64: write64(offset, val); return;
      }
    }
  }

  // Treat bool specially, it is the only non-byte sized write we allow.
  Expr::Width w = value->getWidth();
  if (w == Expr::Bool) {
    write8(offset, ZExtExpr::create(value, Expr::Int8));
    return;
  }

  // Otherwise, follow the slow general case.
  unsigned NumBytes = w / 8;
  assert(w == NumBytes * 8 && "Invalid write size!");
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx, ExtractExpr::create(value, 8 * i, Expr::Int8));
  }
} 

void ObjectState::write16(unsigned offset, uint16_t value) {
  unsigned NumBytes = 2;
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx, (uint8_t) (value >> (8 * i)));
  }
}

void ObjectState::write32(unsigned offset, uint32_t value) {
  unsigned NumBytes = 4;
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx, (uint8_t) (value >> (8 * i)));
  }
}

void ObjectState::write64(unsigned offset, uint64_t value) {
  unsigned NumBytes = 8;
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned idx = Context::get().isLittleEndian() ? i : (NumBytes - i - 1);
    write8(offset + idx, (uint8_t) (value >> (8 * i)));
  }
}

void ObjectState::print() const {
  llvm::errs() << "-- ObjectState --\n";
  llvm::errs() << "\tMemoryObject ID: " << object->id << "\n";
  llvm::errs() << "\tRoot Object: " << updates.root << "\n";
  llvm::errs() << "\tSize: " << size << "\n";

  llvm::errs() << "\tBytes:\n";
  for (unsigned i=0; i<size; i++) {
    llvm::errs() << "\t\t["<<i<<"]"
               << " concrete? " << isByteConcrete(i)
               << " known-sym? " << isByteKnownSymbolic(i)
               << " flushed? " << isByteFlushed(i) << " = ";
    ref<Expr> e = read8(i);
    llvm::errs() << e << "\n";
  }

  llvm::errs() << "\tUpdates:\n";
  for (const UpdateNode *un=updates.head; un; un=un->next) {
    llvm::errs() << "\t\t[" << un->index << "] = " << un->value << "\n";
  }
}
