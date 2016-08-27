#ifndef PHOTINO_CORE_MEMORY_H_
#define PHOTINO_CORE_MEMORY_H_

#include <stddef.h>
#include <stdlib.h>

static void* alloc_aligned(size_t size, size_t alignment);
static void alloc_free(void*);

static inline void* alloc_aligned(size_t size, size_t alignment)
{
#ifdef _MSC_VER
	return _aligned_malloc(size, alignment);
#else
	void* result;
	if(posix_memalign(&result, alignment, size)) result = NULL;
	return result;
#endif
}
static inline void free_aligned(void* ptr)
{
#ifdef _MSC_VER
	_aligned_free(ptr);
#else
	free(ptr);
#endif
}

#endif // !PHOTINO_CORE_MEMORY_H_
