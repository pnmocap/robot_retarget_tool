#pragma once

#include <atomic>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

#include "third_party/MocapApi/include/MocapApi.h"

namespace base {

template <typename T, typename Handle>
class handle_manager {
 protected:
  Handle insert(const std::shared_ptr<T>& object) {
    const Handle handle = ++_nextHandle;
    _objects[handle] = object;
    return handle;
  }

  template <typename Fn>
  auto run(Handle handle, Fn&& fn) -> decltype(fn(std::shared_ptr<T>{})) {
    auto iter = _objects.find(handle);
    using Ret = decltype(fn(std::shared_ptr<T>{}));

    if (iter == _objects.end()) {
      if constexpr (std::is_same_v<Ret, MocapApi::EMCPError>) {
        return MocapApi::Error_InvalidHandle;
      } else {
        throw std::runtime_error("invalid handle");
      }
    }

    auto object = iter->second.lock();
    if (!object) {
      _objects.erase(iter);
      if constexpr (std::is_same_v<Ret, MocapApi::EMCPError>) {
        return MocapApi::Error_InvalidObject;
      } else {
        throw std::runtime_error("expired handle");
      }
    }

    return fn(object);
  }

  void remove(Handle handle) { _objects.erase(handle); }

 private:
  std::unordered_map<Handle, std::weak_ptr<T>> _objects;
  std::atomic<Handle> _nextHandle{0};
};

}  // namespace base
