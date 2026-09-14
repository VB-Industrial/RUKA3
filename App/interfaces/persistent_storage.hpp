#pragma once

#include <cstddef>

namespace ruka3 {

class PersistentStorage {
public:
    virtual ~PersistentStorage() = default;
    virtual bool read(std::size_t address, void* output, std::size_t size) = 0;
    virtual bool write(std::size_t address, const void* data, std::size_t size) = 0;
};

}  // namespace ruka3
