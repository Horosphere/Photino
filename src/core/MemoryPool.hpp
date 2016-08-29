#ifndef PHOTINO_CORE_MEMORYPOOL_HPP_
#define PHOTINO_CORE_MEMORYPOOL_HPP_

#include <queue>
#include <cstdint>

extern "C"
{
#include "memory.h"
}
#include "photino.hpp"

namespace photino
{

class MemoryPool
{
public:
	MemoryPool(std::size_t blockSize = 0x10000);
	~MemoryPool();

	/**
	 * @warning This function is not responsible for in-place constructing the
	 *  array.
	 * @brief Allocates a continuous piece of memory capable of holding size T s.
	 */
	template <typename T = uint8_t> T* alloc(std::size_t size);
	/**
	 * @brief See \ref alloc with default constructor calls
	 */
	template <typename T = uint8_t> T* alloc_ctor(std::size_t size);

	/**
	 * @warning This function is not responsible for in-place constructing the
	 *  array.
	 * @brief Allocates a continuous piece of memory capable of a T.
	 */
	template <typename T> T* alloc();
	/**
	 * @brief See \ref alloc with default constructor calls
	 */
	template <typename T> T* alloc_ctor();

	void freeAll();

private:
	std::size_t blockSize;
	std::size_t index;

	uint8_t* block;
	std::vector<uint8_t*> blocksFull;
	std::vector<uint8_t*> blocksEmpty;

};


// Implementations

inline MemoryPool::MemoryPool(std::size_t blockSize):
	blockSize(blockSize), index(0),
	block((uint8_t*) alloc_aligned(blockSize, PHOTINO_MEMALIGN))
{
}
inline MemoryPool::~MemoryPool()
{
	for (uint8_t* block : blocksFull)
		free_aligned(block);
	for (uint8_t* block : blocksEmpty)
		free_aligned(block);
}

template <typename T> inline T*
MemoryPool::alloc(std::size_t size)
{
	return reinterpret_cast<T*>(alloc<uint8_t>(size * sizeof(T)));
}
template <typename T> inline T*
MemoryPool::alloc_ctor(std::size_t size)
{
	T* const ptr = alloc<T>(size);
	for (std::size_t i = 0; i < size; ++i)
		new (ptr + i) T();
	return ptr;
}

template <> inline uint8_t*
MemoryPool::alloc(std::size_t size)
{
	size = ((size + 0xF) & (~0xF)); // Align to 0x10
	if (index + size > blockSize) // Block full. Needs new block
	{
		blocksFull.push_back(block);

		if (blocksEmpty.size() && size <= blockSize)
		{
			block = blocksEmpty.back();
			blocksEmpty.pop_back();
		}
		else
			block = (uint8_t*) alloc_aligned(size > blockSize ? size : blockSize,
			                                 PHOTINO_MEMALIGN);
		index = 0;
	}
	uint8_t* result = block + index;
	index += size;
	return result;
}
template <typename T> inline T*
MemoryPool::alloc()
{
	return alloc<T>(1);
}
template <typename T> inline T*
MemoryPool::alloc_ctor()
{
	T* ptr = alloc<T>();
	new (ptr) T();
	return ptr;
}
inline void MemoryPool::freeAll()
{
	index = 0;
	blocksEmpty.insert(blocksEmpty.end(), blocksFull.begin(), blocksFull.end());
	blocksFull.clear();
}

} // namespace photino

#endif // !PHOTINO_CORE_MEMORYPOOL_HPP_
