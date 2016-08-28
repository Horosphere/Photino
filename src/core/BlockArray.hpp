#ifndef PHOTINO_CORE_BLOCKARRAY_HPP_
#define PHOTINO_CORE_BLOCKARRAY_HPP_

#include <cstdint>

extern "C"
{
#include "memory.h"
}
#include "photino.hpp"
#include "../math/integers.hpp"

namespace photino
{

template <typename T, int logBlockSize>
class BlockArray
{
public:
	static constexpr std::size_t const blockSize = 1 << logBlockSize;

	/**
	 * @warning Function not responsible for in-place constructing
	 * @brief Allocates a two-dimensional block array of dimensions m * n
	 */
	BlockArray(std::size_t m, std::size_t n);
	~BlockArray();

	std::size_t width() const;
	std::size_t height() const;

	T operator()(std::size_t j, std::size_t k) const;
	T& operator()(std::size_t j, std::size_t k);

//private:
	/**
	 * @brief Evaluates i / blockSize using bitwise operations
	 */
	std::size_t blockIndex(std::size_t i) const;
	/**
	 * @brief Evaluates i % blockSize using bitwise operations
	 */
	std::size_t blockOffset(std::size_t i) const;

	std::size_t arraySize() const;

	std::size_t m, n;
	std::size_t rowBlocks;
	T* const data;
};


// Implementations
template <typename T, int logBlockSize> inline
BlockArray<T, logBlockSize>::BlockArray(std::size_t m, std::size_t n):
	m(m), n(n), rowBlocks(roundUpModulo(m, blockSize) >> logBlockSize),
	data((T* const) alloc_aligned(arraySize() * sizeof(T), PHOTINO_MEMALIGN))
{
}
template <typename T, int logBlockSize> inline
BlockArray<T, logBlockSize>::~BlockArray()
{
	free_aligned(data);
}

template <typename T, int logBlockSize> inline std::size_t
BlockArray<T, logBlockSize>::width() const
{
	return m;
}
template <typename T, int logBlockSize> inline std::size_t
BlockArray<T, logBlockSize>::height() const
{
	return n;
}
template <typename T, int logBlockSize> inline T
BlockArray<T, logBlockSize>::operator()(std::size_t j, std::size_t k) const
{
	std::size_t jBlock = blockIndex(j);
	std::size_t kBlock = blockIndex(k);
	std::size_t jOffset = blockOffset(j);
	std::size_t kOffset = blockOffset(k);
	std::size_t bOffset = blockSize * blockSize * (jBlock * rowBlocks + kBlock);
	return data[bOffset + jOffset * blockSize + kOffset];
}
template <typename T, int logBlockSize> inline T&
BlockArray<T, logBlockSize>::operator()(std::size_t j, std::size_t k)
{
	std::size_t jBlock = blockIndex(j);
	std::size_t kBlock = blockIndex(k);
	std::size_t jOffset = blockOffset(j);
	std::size_t kOffset = blockOffset(k);
	std::size_t bOffset = blockSize * blockSize * (jBlock * rowBlocks + kBlock);
	return data[bOffset + jOffset * blockSize + kOffset];
}


template <typename T, int logBlockSize> inline std::size_t
BlockArray<T, logBlockSize>::blockIndex(std::size_t i) const
{
	return i >> logBlockSize;
}
template <typename T, int logBlockSize> inline std::size_t
BlockArray<T, logBlockSize>::blockOffset(std::size_t i) const
{
	return i & (blockSize - 1);
}
template <typename T, int logBlockSize> inline std::size_t
BlockArray<T, logBlockSize>::arraySize() const
{
	return roundUpModulo(m, blockSize) * roundUpModulo(n, blockSize);
}
} // namespace photino

#endif // !PHOTINO_CORE_BLOCKARRAY_HPP_
