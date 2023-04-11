#include <stdlib.h>
#include <kernel/dpl/HeapP.h>

#ifndef far 
#define far /* Empty */
#endif

extern HeapP_Object gHeapObj;

far unsigned int memoryBytesUsed = 0;

void *gtrack_alloc(unsigned int numElements, unsigned int sizeInBytes)
{
	memoryBytesUsed += numElements*sizeInBytes;
    // return MemoryP_ctrlAlloc(numElements*sizeInBytes, 0);
	return HeapP_alloc(&gHeapObj, (numElements*sizeInBytes));
}
void gtrack_free(void *pFree, unsigned int sizeInBytes)
{
	memoryBytesUsed -= sizeInBytes;
	// MemoryP_ctrlFree(pFree,sizeInBytes);
    HeapP_free(&gHeapObj, pFree);
}
