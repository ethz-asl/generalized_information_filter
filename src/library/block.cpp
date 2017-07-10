
#include "filter_test/block.h"

namespace tsif {

namespace block_helper {
BlockBase* createBlockByType(BlockTypeId block_type) {
  switch (block_type) {
    case kVector1:
      return new VectorBlock<1>();
    case kVector2:
      return new VectorBlock<2>();
    case kVector3:
      return new VectorBlock<3>();
    case kVector4:
      return new VectorBlock<4>();
    case kVector5:
      return new VectorBlock<5>();
    case kVector6:
      return new VectorBlock<6>();
    case kSO3:
      CHECK(false);  // TODO(burrimi): Implement this!
      break;
    default:
      break;
  }
  return NULL;
}
}

}  // namespace tsif
