
#include "filter_test/block_impl.h"

namespace tsif {

namespace block_helper {
BlockBase::Ptr createBlockByType(BlockTypeId block_type) {
  switch (block_type) {
    case kVector1:
      return std::make_shared<VectorBlock<1>>();
    case kVector2:
      return std::make_shared<VectorBlock<2>>();
    case kVector3:
      return std::make_shared<VectorBlock<3>>();
    case kVector4:
      return std::make_shared<VectorBlock<4>>();
    case kVector5:
      return std::make_shared<VectorBlock<5>>();
    case kVector6:
      return std::make_shared<VectorBlock<6>>();
    case kSO3:
      return std::make_shared<QuaternionBlock>();
      break;
    default:
      break;
  }
  return NULL;
}
}

}  // namespace tsif
