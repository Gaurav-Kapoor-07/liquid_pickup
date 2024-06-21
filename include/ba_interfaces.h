#ifndef BA_INITIALIZABLE_NODE
#define BA_INITIALIZABLE_NODE

#include "manipulator.h"

namespace BT 
{
  using NodeConfiguration = NodeConfig;
  using AsyncActionNode = ThreadedAction;
  template <typename T>
  using Optional = Expected<T>;
}

#endif