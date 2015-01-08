#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>

#include "mtmm.h"

#if 1
#define DBG_MSG printf("\n[%d]: %s): ", __LINE__, __FUNCTION__);printf
#define DBG_ENTRY printf("\n[%d]: --> %s", __LINE__,__FUNCTION__);
#define DBG_EXIT printf("\n[%d]: <-- %s", __LINE__,__FUNCTION__);
#else
#define DBG_MSG
#define DBG_ENTRY
#define DBG_EXIT
#endif

  
#define NUM_HEAPS			3
#define GLOBAL_HEAP			0
/* log2(SUPERBLOCK_SIZE) */
#define NUM_SIZE_CLASSES	17 /* 16 real size classes, +1 for 'any size' i.e. recycling completely empty superblocks*/	
#define RECYCLED_CLASS		16 /* the 17th slot is for completely empty, recycled superblocks that don't yet belong to any size class */

/* threshold for using hoard. If memory requested is more than this, then just mmap and return pointer to user */
#define HOARD_THRESHOLD_MEM_SIZE	(SUPERBLOCK_SIZE/2)		

/* fullness threshold : actually defines 'how empty' a heap needs to be before one of its empty enough superblocks is moved to the global heap.
The criteria for moving an empty enough superblock to the global heap is: u(i) < a(i) − K ∗ S and u(i) < (1 − f)a(i)) 
where a(i) is the amount of memory in use by heap i, a(i) is the amount of memory originally allocated for heap i from the OS, f is fullness and K 
is number of empty enough superblocks.
E.g. when f=1/4 and K=0, heap will move one of its superblocks that is less than 75% full if the heap's total memory in use is
less than 75% of the total memory allocated to that heap. */
#define FULLNESS_THRESHOLD_F			0.25
#define SUPERBLOCK_EMPTY_THRESHHOLD_K	0						
	

/* header of memory block */ 
typedef struct sBlockHeader
{
	unsigned int		inUse;							/* 1 if block is allocated to user, otherwise 0 */
	size_t				size;							/* size of allocated memory as available for user */
	struct sBlockHeader	*pNextFree;						/* pointer to next free block in linked list of free blocks. Only relevant if not in use. */
	struct sSuperblock	*pMySuperblock;					/* pointer back to superblock that contains this block */
} tBlockHeader;

typedef struct sSuperblock
{
	struct sSuperblock	*pPrev;
	struct sSuperblock	*pNext;							
	unsigned int		sizeClass; 						/* 0-16 - size class 16 is the recycled class - can be any size */
	size_t				blockSize;						/* calculate only once on initialization */
	unsigned int		ownerHeap;						/* index into the heap array to the heap this superblock belongs to */
	unsigned int 		numBlocks; 						/* up to SUPERBLOCK_SIZE/blockSize. Calculated upon creation. */
	tBlockHeader		*pBlockArray;					/* mmap space needed according to num blocks - depends on class size */
	unsigned int		numFreeBlocks;					/* keep track of number of free blocks	*/
	tBlockHeader		*pFreeBlocksHead;				/* pointer to LIFO linked list of free blocks. */
}tSuperblock;

/* A collection of superblocks. Each superblock is divided into blocks of equal size, each equalling this class's size */
typedef struct sSizeClass
{
	unsigned int		size;							/* class size: 0 if superblock completely empty and not yet classified. otherwise ranges from 2^0 to 2^15 */
	tSuperblock			*pHead;							/* superblocks ordered from most full to least full */
	tSuperblock			*pTail;	
}tSizeClass;

typedef struct sHeap
{
	unsigned int		processorId; 					/* one heap per CPU */
	size_t				statMemoryInUse;				/* The amount of memory in use by this heap */
	size_t				statMemoryHeld;					/* The amount of memory held in this heap that was allocated from the operating system */
	tSizeClass			sizeClasses[NUM_SIZE_CLASSES]; 	/* hold size classes for all sizes from 1 to log2(SUPERBLOCK_SIZE) plus one for completely empty s.blocks */	
} tHeap;


typedef struct sHoard
{
	tHeap				heapArray[NUM_HEAPS];
}tHoard;

/* Heaps are defined as a static array in the heap - reside in the data segment */
static tHoard		s_hoard;	

/* Function to print out contents of hoard heaps */
static void dumpHoard(char *title);

/* Allocate memory straight from OS  */
static void *	allocateLargeMemoryChunk(size_t	sz);

/* Deallocate memory that was previously allocated from OS  */
static void		deallocateLargeMemoryChunk(void * ptr, size_t sz);

/* Gets an index into the heap array based on the current thread's processor id */
static int		getHeapNumber(unsigned int *pHeapNumber);

/* Get size class by rounding up requested size to next highest power of 2, return that power. */
static int		getSizeClass    (size_t	requestedSize, unsigned int *pNextPowerOfTwo);

/* Allocate memory (memmap) for the superblock */
static tSuperblock	*	createSuperblock(unsigned int heapNum, unsigned int sizeClass);

/* Initialize the superblock for a given size class and heap */
static void initSuperblock(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock);
 
/* internal malloc function */ 
static void * allocMem(unsigned int heapNum, unsigned int sizeClass);

/* search for free block in given size class, return pointer to block if found, otherwise NULL. Update heap statistics */
static void * allocFromFreeBlockInHeap(unsigned int heapNum, unsigned int sizeClass);

/* create a new superblock, add to the given heap and size class, check emptiness invariant, update heap statistics, return pointer to requested block of memory */
static void *allocFromFreeBlockInNewSuperblock(unsigned int heapNum, unsigned int sizeClass);

/* memory to add to 'memory held' statistic in given heap. Memory to add may be negative. */
static void updateMemoryHeld(unsigned int heapNum, int memoryToAdd);

/* memory to add to 'memory used' statistic in given heap. Memory to add may be negative. */
static void updateMemoryUsed(unsigned int heapNum, int memoryToAdd);

/* return 1 if heap is under the fullness threshold, otherwise if heap is still full enough, return 0 */
static int isEmptyEnough(unsigned int heapNum);

/* return the superblock that is empty enough to be moved to the global heap  */
static tSuperblock *findEmptyEnoughSuperblock(unsigned int heapNum);

/* move superblock from one heap to the other, update statistics ...*/
static void moveSuperblockFromTo(unsigned int fromHeap, unsigned int toHeap, tSuperblock *pSuperblock);

/* add superblock to the sorted-from-fullest-to-emptiest list of superblocks for the given size class and heap */
static void addSuperblockToClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock);

/* remove superblock from the sorted-from-fullest-to-emptiest list of superblocks for the given size class and heap */
static void removeSuperblockFromClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock);

/* Keep sorted-from-fullest-to-emptiest list of superblocks sorted upon malloc or free */
static void reorderSuperblockInClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock);

/* allocate one block of memory from the given superblock */
void * allocBlock(tSuperblock *pSuperblock, unsigned int requestedSizeClass);

/* Recycle completely empty superblocks to be used by any size class */
static void recycleSuperblock(unsigned int heapNum, unsigned int newSizeClass, tSuperblock *pSuperblock);

/* check if the heap is too empty and move f-empty superblocks out to global heap */
static void checkInvariantAndMoveSuperblocks(unsigned int heapNum);

/*
The malloc() function allocates size bytes and returns a pointer to the allocated memory. 
The memory is not initialized. If size is 0, then malloc() returns either NULL, or a unique 
pointer value that can later be successfully passed to free(). 


malloc (sz)
1. If sz > S/2, allocate the superblock from the OS and return it.
2. i ← hash(the current thread).
3. Lock heap i.
4. Scan heap i’s list of superblocks from most full to least (for the size class corresponding to sz).
5. If there is no superblock with free space,
6. Check heap 0 (the global heap) for a superblock.
7. If there is none,
8. Allocate S bytes as superblock s and set the owner to heap i.
9. Else,
10. Transfer the superblock s to heap i.
11. u 0 ← u 0 − s.u
12. u i ← u i + s.u
13. a 0 ← a 0 − S
14. a i ← a i + S
15. u i ← u i + sz.
16. s.u ← s.u + sz.
17. Unlock heap i.
18. Return a block from the superblock.
*/

void * malloc (size_t sz)
{	
	void			*p = 0;
	unsigned int 	heapNum = GLOBAL_HEAP;
	unsigned int	sizeClassIndex = 0U;
		
	//DBG_MSG("requested size: %d\n", sz);
	printf("malloc requested size: %d\n", sz);
	
	dumpHoard("begin malloc");

	if (sz >= HOARD_THRESHOLD_MEM_SIZE)
	{
		p = allocateLargeMemoryChunk(sz);
		if (!p)
		{
			return 0;
		}
		return p;
	}
	
	/* hash to the correct heap */
	if (!getHeapNumber(&heapNum))
	{
		return 0;
	}
	
	DBG_MSG("heap =  %d\n", heapNum);
	
	if (!getSizeClass (sz, &sizeClassIndex))
	{
		return 0;
	}
	DBG_MSG("sizeClassIndex =  %d\n", sizeClassIndex);
	
	p = allocMem(heapNum, sizeClassIndex);
	DBG_MSG("after allocMem p=0x%x\n", (unsigned int)p);
	if (!p)
	{
		return 0;
	}
	
	/* we found free memory! */
	printf("malloc returns p=0x%x\n", (unsigned int)p);
	return p;
}

/*

The free() function frees the memory space pointed to by ptr, which must have been returned 
by a previous call to malloc(), calloc() or realloc(). Otherwise, or if free(ptr) has already 
been called before, undefined behavior occurs. If ptr is NULL, no operation is performed.


free (ptr)
1. If the block is “large”,
2. Free the superblock to the operating system and return.
3. Find the superblock s this block comes from and lock it.
4. Lock heap i, the superblock’s owner.
5. Deallocate the block from the superblock.
6. u i ← u i − block size.
7. s.u ← s.u − block size.
8. If i = 0, unlock heap i and the superblock and return.
9. If u i < a i − K ∗ S and u i < (1 − f) ∗ a i,
10. Transfer a mostly-empty superblock s1
to heap 0 (the global heap).
11. u 0 ← u 0 + s1.u, u i ← u i − s1.u
12. a 0 ← a 0 + S, a i ← a i − S
13. Unlock heap i and the superblock.
*/
void free (void * ptr) 
{  
	tBlockHeader	*pBlockHeader;
	tSuperblock		*pMySuperblock;
	size_t			size;
	unsigned int	heapNum;
	
	if (!ptr)
	{
		return;
	}
	
	pBlockHeader = (tBlockHeader *)(ptr - sizeof(tBlockHeader));
	
	size = pBlockHeader->size;
	 
	if (size >= HOARD_THRESHOLD_MEM_SIZE)
	{
		deallocateLargeMemoryChunk(ptr, size + sizeof(tBlockHeader));
	}

	/* find where this block came from */
	pMySuperblock = pBlockHeader->pMySuperblock;
	heapNum = pMySuperblock->ownerHeap;
	
	pBlockHeader->inUse = 0;
	/* attach this block to the head of the free list */	
	pBlockHeader->pNextFree = pMySuperblock->pFreeBlocksHead;
	pMySuperblock->pFreeBlocksHead = pBlockHeader;
	pMySuperblock->numFreeBlocks++;
	
	/* keep superblocks ordered by fullness */
	reorderSuperblockInClass(heapNum, pMySuperblock->sizeClass, pMySuperblock);
	
	if (pMySuperblock->numFreeBlocks == pMySuperblock->numBlocks)
	{
		/* An empty superblock container, recycle it! */
		recycleSuperblock(heapNum, RECYCLED_CLASS, pMySuperblock);
	}
	
	updateMemoryUsed(heapNum, (-1)*pMySuperblock->blockSize);
	/* Check heap invariants, if necessary move superblock to global heap */		
	checkInvariantAndMoveSuperblocks(heapNum);	
}

/*

The realloc() function changes the size of the memory block pointed to by ptr to size bytes. 
The contents will be unchanged in the range from the start of the region up to the minimum 
of the old and new sizes. If the new size is larger than the old size, the added memory 
will not be initialized. If ptr is NULL, then the call is equivalent to malloc(size), 
for all values of size; if size is equal to zero, and ptr is not NULL, then the call 
is equivalent to free(ptr). Unless ptr is NULL, it must have been returned by an earlier 
call to malloc(), calloc() or realloc(). If the area pointed to was moved, a free(ptr) is done. 


1. allocate sz bytes
2. copy from old location to a new one
3. free old allocation
*/
void * realloc (void * ptr, size_t sz) 
{

	printf("myrealloc\n");
	return malloc(sz);
}

static void *	allocateLargeMemoryChunk(size_t	sz)
{
	int fd;
	void *p;

	DBG_ENTRY;	
	fd = open("/dev/zero", O_RDWR);
	
	if (fd == -1){
		return 0;
	}
	
	p = mmap(0, sz + sizeof(tBlockHeader), PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);
	close(fd);

	if (p == MAP_FAILED){
		return 0;
	}

	((tBlockHeader *) p) -> size = sz;

	DBG_EXIT;
	return (p + sizeof(tBlockHeader));	
}

static void deallocateLargeMemoryChunk(void * ptr, size_t sz)
{
	DBG_ENTRY;
	
	if (munmap(ptr - sizeof(tBlockHeader), sz) < 0)
	{
		perror(NULL);
	}

	DBG_EXIT;
}

static int		getHeapNumber(unsigned int *pHeapNumber)
{
	pthread_t         self;
	self = pthread_self();
	
	DBG_MSG("self =  0x%.8x\n", (unsigned)self);
	
	/* trying to reduce the probability that two threads will use the same heap */
	*pHeapNumber = ((self >> 12) % (NUM_HEAPS-1)) + 1;
	return 1;	
}

/* Get size class by rounding up requested size to next highest power of 2, return that power. */
static int		getSizeClass    (size_t	requestedSize, unsigned int *pNextPowerOfTwo)
{
	int		count = 0;
	int		dividedByTwo = requestedSize -1; /*subtracting 1 so that if the requested size is exact power, we won't round up */
	
	while (dividedByTwo)
	{
		dividedByTwo = dividedByTwo/2;
		count++;
	}

	*pNextPowerOfTwo = count;
	return 1;
}

static tSuperblock	*	createSuperblock(unsigned int heapNum, unsigned int sizeClass)
{
	int 		fd;
	tSuperblock *pNewSuperblock	= 0;
	void		*p				= 0;
  
	DBG_ENTRY;	
		
	fd = open("/dev/zero", O_RDWR);
	
	if (fd == -1){
		return 0;
	}
	
	pNewSuperblock = mmap(0, sizeof(tSuperblock), PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);	
	
	if (pNewSuperblock == MAP_FAILED){
		close(fd);
		return 0;
	}
	
	p = mmap(0, SUPERBLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);	
	close(fd);

	if (p == MAP_FAILED){
		return 0;
	}
	DBG_MSG("p 0x%X\n", (unsigned int)p);
	
	/* Initialize the superblock structure */
	pNewSuperblock->pBlockArray = (tBlockHeader *)p;	
	
	initSuperblock(heapNum, sizeClass, pNewSuperblock);
	
	DBG_EXIT;
	return pNewSuperblock;	
}

/* Initialize the superblock for a given size class and heap */
static void initSuperblock(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock)
{

	void			*pNewBlock;				/* first block in superblock */
	void			*pEndOfSuperblock;		/* end of last block in superblock */
	size_t			blockSize, blockSizeWithHeader;	/* user memory chunk + header */
	unsigned int	numBlocks = 0;			/* final count depends on block size */
	
	DBG_ENTRY;
	pNewBlock			= (void*) (pSuperblock -> pBlockArray);
	/* The actual block size is a power of two */
	blockSize = 1 << sizeClass;
	pEndOfSuperblock	= pNewBlock + SUPERBLOCK_SIZE;
	blockSizeWithHeader = blockSize + sizeof(tBlockHeader);
	
	DBG_MSG("1st block 0x%X end 0x%X blockSize %d blockSizeWithHeader %d\n",
			(unsigned int)pNewBlock, (unsigned int)pEndOfSuperblock, blockSize, blockSizeWithHeader);
			
	/* fit in as many blocks as possible into the superblock */
	while (pNewBlock + blockSizeWithHeader < pEndOfSuperblock)
	{		
		/* init each block */				
		((tBlockHeader *)pNewBlock) -> inUse = 0U; 
		((tBlockHeader *)pNewBlock) -> size = blockSize;
		((tBlockHeader *)pNewBlock) -> pMySuperblock = pSuperblock;
		((tBlockHeader *)pNewBlock) -> pNextFree = pNewBlock + blockSizeWithHeader;
		/* advance to next block */
		pNewBlock += blockSizeWithHeader;
		numBlocks++;
	}
	
	/* set the last block as the tail */
	((tBlockHeader *)pNewBlock) -> pNextFree = 0;
	
	DBG_MSG("numBlocks in superblock with class size %d:  %d\n",blockSize, numBlocks);
	
	/* init the superblock */
	pSuperblock->pPrev = 0;
	pSuperblock->pNext = 0;						
	pSuperblock->sizeClass = sizeClass;
	pSuperblock->blockSize = blockSize;
	pSuperblock->ownerHeap = heapNum;
	pSuperblock->numBlocks = numBlocks;
	pSuperblock->numFreeBlocks = numBlocks;	
	pSuperblock->pFreeBlocksHead = pSuperblock->pBlockArray;
	
	/* now update how much memory is held. Note that this is not the real amount of memory - but
	rather the amount of memory held that will be taken in to account when calculating how 'empty' a superblock is */
	updateMemoryHeld(heapNum, (pSuperblock->numBlocks * pSuperblock->blockSize));
	
	DBG_EXIT;
}

/* internal malloc function */
static void * allocMem(unsigned int heapNum, unsigned int sizeClass)
{
	void *p;
	DBG_ENTRY;
	/* Is there a free block in this heap (in the appropriate size class) */
	p = allocFromFreeBlockInHeap(heapNum, sizeClass);
	if (p)
	{
		DBG_EXIT;
		return p;
	}
	
	/* So check to see if we can use a recycled superblock */
	p = allocFromFreeBlockInHeap(heapNum, RECYCLED_CLASS);
	if (p)
	{
		DBG_EXIT;
		return p;
	}
	
	/* So try the global heap */	
	p = allocFromFreeBlockInHeap(GLOBAL_HEAP, sizeClass);
	if (p)
	{
		/* move superblock to appropriate size class in regular heap */
		moveSuperblockFromTo(GLOBAL_HEAP, heapNum, ((tBlockHeader *)(p - sizeof(tBlockHeader)))->pMySuperblock);
		DBG_EXIT;
		return p;
	}
	/* No free chunk in global heap either. So try to allocate from a new superblock allocated from OS */ 
	/* get a new superblock for this heap and add to appropriate size class */
	p = allocFromFreeBlockInNewSuperblock(heapNum, sizeClass);
		
	/* whether we failed or succeeded, return pointer - will either be NULL or point to allocated block */
	DBG_EXIT;
	return p;
	
}


/* search for free block in given size class, return pointer to block if found, otherwise NULL. Update heap statistics */
static void * allocFromFreeBlockInHeap(unsigned int heapNum, unsigned int sizeClass)
{
	tSizeClass		*pSizeClass;
	tSuperblock		*pSuperblock;
	void			*p;
	
	DBG_ENTRY;
	/* Check each superblock of this heap and size class and see if it has any free memory */
	pSizeClass = &s_hoard.heapArray[heapNum].sizeClasses[sizeClass];
	/* start searching from the most full */
	pSuperblock = pSizeClass->pHead;
	
	while (pSuperblock)
	{
		p = allocBlock(pSuperblock, sizeClass);
		if (p)
		{
			/* found a free block! */
			updateMemoryUsed(pSuperblock->ownerHeap, pSuperblock->blockSize);
	
			/* This block's superblock might need to change its place in the ordered list */
			reorderSuperblockInClass(pSuperblock->ownerHeap, sizeClass, pSuperblock);
			
			DBG_EXIT;
			return p;
		}
		pSuperblock = pSuperblock->pNext;
	}
			
	/* No free memory found in this class */
	DBG_EXIT;
	return 0;
}

/* create a new superblock, add to the given heap and size class, check emptiness invariant, update heap statistics, return pointer to requested block of memory */
static void *allocFromFreeBlockInNewSuperblock(unsigned int heapNum, unsigned int sizeClass)
{
	tSuperblock		*pNewSuperblock;
	void			*p;
		
	DBG_ENTRY;
	
	pNewSuperblock = createSuperblock(heapNum, sizeClass);
	if (!pNewSuperblock)
	{
		DBG_EXIT;
		return 0;
	}
		
	p = allocBlock(pNewSuperblock, sizeClass);
	
	if (!p)
	{
		/* shouldn't get to this, since it's a brand new empty superblock ... maybe should assert?*/
		DBG_EXIT;
		return 0;
	}

	updateMemoryUsed(heapNum, pNewSuperblock->blockSize);
	
	/* Now attach the new superblock to the correct size class */
	addSuperblockToClass(heapNum, sizeClass, pNewSuperblock);

	/* Check heap invariants, if necessary move superblock to global heap */		
	checkInvariantAndMoveSuperblocks(heapNum);		
			
	DBG_EXIT;
	return p;
}

/* memory to add to 'memory held' statistic in given heap. Memory to add may be negative. */
static void updateMemoryHeld(unsigned int heapNum, int memoryToAdd)
{
	s_hoard.heapArray[heapNum].statMemoryHeld += memoryToAdd;
}

/* memory to add to 'memory used' statistic in given heap. Memory to add may be negative. */
static void updateMemoryUsed(unsigned int heapNum, int memoryToAdd)
{
	s_hoard.heapArray[heapNum].statMemoryInUse += memoryToAdd;
}


/* return 1 if heap is under the fullness threshold, otherwise if heap is still full enough, return 0 */
static int isEmptyEnough(unsigned int heapNum)
{
	size_t	memHeld, memInUse;
	
	memHeld = s_hoard.heapArray[heapNum].statMemoryHeld;
	memInUse = s_hoard.heapArray[heapNum].statMemoryInUse;
	
	/* fortunately k=0 - otherwise there would be a bug here.. because elsewhere memory in use is numblocks*blocksize... */
	if (memInUse >= memHeld - (SUPERBLOCK_EMPTY_THRESHHOLD_K * SUPERBLOCK_SIZE))
	{
		return 0;
	}
	
	if (memInUse >= (1 - FULLNESS_THRESHOLD_F) * memHeld)
	{
		return 0;
	}
	/* The emptiness invariant holds */	
	return 1;
}

/* return the superblock that is empty enough to be moved to the global heap */
static tSuperblock *findEmptyEnoughSuperblock(unsigned int heapNum)
{
	int				sizeClassIdx;
	tSizeClass		*pSizeClass;
	tHeap			*pHeap;
	tSuperblock		*pSuperblock;
	
	DBG_ENTRY;
	/* search through this heap's size classes for a superblock that maintains the emptiness invariant 
	'at least f empty'  - this is using a slow algorithm .. another point that can be optimized */
	pHeap = &s_hoard.heapArray[heapNum];
	
	for (sizeClassIdx = 0; sizeClassIdx < NUM_SIZE_CLASSES; sizeClassIdx++)
	{
		pSizeClass = &pHeap->sizeClasses[sizeClassIdx];
		/* just check the tail since the list is ordered from full to least full */
		pSuperblock = pSizeClass->pTail;
		//DBG_MSG("sizeClassIdx=%d pSuperblock=0x%x\n",sizeClassIdx, (unsigned int)pSuperblock);
		if (pSuperblock && ((pSuperblock->numFreeBlocks/pSuperblock->numBlocks) > FULLNESS_THRESHOLD_F))
		{
			DBG_EXIT;
			return pSuperblock;
		}
	}
	DBG_EXIT;
	return 0;	
}

/* move superblock from one heap to the other, update statistics ...*/
static void moveSuperblockFromTo(unsigned int fromHeap, unsigned int toHeap, tSuperblock *pSuperblock)
{
	unsigned int	sizeClass, numBlocks;
	size_t			heldMemorySize, usedMemorySize, blockSize;

	DBG_ENTRY;
	
	numBlocks = pSuperblock->numBlocks;
	blockSize = pSuperblock->blockSize;
	
	heldMemorySize = numBlocks * blockSize;
	usedMemorySize = (numBlocks - pSuperblock->numFreeBlocks)*blockSize;
		
	/* stay in same size class */
	sizeClass = pSuperblock->sizeClass;
	
	removeSuperblockFromClass(fromHeap, sizeClass, pSuperblock);	
	updateMemoryHeld(fromHeap, (-1)*heldMemorySize);
	updateMemoryUsed(fromHeap, (-1)*usedMemorySize);
	
	addSuperblockToClass(toHeap, sizeClass, pSuperblock);
	updateMemoryHeld(toHeap, heldMemorySize);
	updateMemoryUsed(toHeap, usedMemorySize);
	
	pSuperblock->ownerHeap = toHeap;

}

/* add superblock to the sorted-from-fullest-to-emptiest list of superblocks for the given size class and heap */
static void addSuperblockToClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock)
{
	float			emptyFactor, tempEmptyFactor;
	tSuperblock		*pTempSb;
	tSizeClass		*pSizeClass;
	DBG_ENTRY;
	
	DBG_MSG("heap %d size %d pSuperblock=0x%x\n", heapNum, sizeClass, (unsigned int)pSuperblock);
	pSizeClass = &s_hoard.heapArray[heapNum].sizeClasses[sizeClass];
	emptyFactor = (pSuperblock->numFreeBlocks/pSuperblock->numBlocks);
	/* make sure the superblock to be attached isn't still chained to its previous size class linked list */
	pSuperblock->pNext = NULL;
	pSuperblock->pPrev = NULL;	
	
	/* we'll start the search from the tail because usually we're adding empty superblocks. This algo. can be improved */
	pTempSb = pSizeClass->pTail;
	DBG_MSG("start search from tail=0x%x\n",(unsigned int)pSizeClass->pTail);
		
	if (!pTempSb)
	{
		/* this is the first superblock in the chain */
		pSizeClass->pHead = pSizeClass->pTail = pSuperblock;
		DBG_MSG("first superblock in chain head=0x%x tail=0x%x\n", (unsigned int)pSizeClass->pHead, (unsigned int)pSizeClass->pTail);
		DBG_EXIT;
		return;
	}
	
	/* not the first - need to find its place in the order */
	tempEmptyFactor = pTempSb->numFreeBlocks/pTempSb->numBlocks;
	while (pTempSb && emptyFactor < tempEmptyFactor)
	{
		pTempSb = pTempSb->pPrev;
		tempEmptyFactor = pTempSb->numFreeBlocks/pTempSb->numBlocks;
	}
	
	if (pTempSb)
	{
		/* not inserting at top, so attach to previous */
		pSuperblock->pNext = pTempSb->pNext;
	}
	else
	{
		/* inserting at top */
		pSuperblock->pNext = pSizeClass->pHead;
		pSizeClass->pHead->pPrev = pSuperblock;
		pSizeClass->pHead = pSuperblock;
		DBG_EXIT;
		return;
	}
	
	if(pTempSb->pNext)
	{
		/* we're not inserting at end */
		pTempSb->pNext->pPrev = pSuperblock;
	}
	else
	{
		/* We are inserting at end */
		pSizeClass->pTail = pSuperblock;
	}
	
	pTempSb->pNext = pSuperblock;
	pSuperblock->pPrev = pTempSb;
	
	DBG_EXIT;
}

/* remove superblock from the sorted-from-fullest-to-emptiest list of superblocks for the given size class and heap */
static void removeSuperblockFromClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock)
{

	tSizeClass		*pSizeClass;
	DBG_ENTRY;
	
	pSizeClass = &s_hoard.heapArray[heapNum].sizeClasses[sizeClass];
	
	if (pSuperblock->pNext)
	{
		pSuperblock->pNext->pPrev = pSuperblock->pPrev;	
	}
	
	if (pSuperblock->pPrev)
	{
		pSuperblock->pPrev->pNext = pSuperblock->pNext;	
	}
	
	if (!pSuperblock->pPrev)
	{
		pSizeClass->pHead = pSuperblock->pNext;	
	}
	
	if (!pSuperblock->pNext)
	{
		pSizeClass->pTail = pSuperblock->pPrev;	
	}	
	DBG_EXIT;
}

/* Keep sorted-from-fullest-to-emptiest list of superblocks sorted upon malloc or free */
static void reorderSuperblockInClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock)
{
	DBG_ENTRY;
	/* just remove from linked list and put back in to keep it ordered. */
	removeSuperblockFromClass(heapNum, sizeClass, pSuperblock);
	addSuperblockToClass(heapNum, sizeClass, pSuperblock);
	DBG_EXIT;
}
/* allocate one block of memory from the given superblock */
void * allocBlock(tSuperblock *pSuperblock, unsigned int requestedSizeClass)
{
	tBlockHeader		*pBlock;
	void				*p = 0;
	
	DBG_ENTRY;
	pBlock = pSuperblock->pFreeBlocksHead;
	
	if (!pBlock)
	{
		DBG_EXIT;
		return 0;
	}
	
	if (RECYCLED_CLASS == pSuperblock->sizeClass)
	{
		recycleSuperblock(pSuperblock->ownerHeap, requestedSizeClass, pSuperblock);
	}
	
	/* user memory block has a header right before it. This is the 'trick' for finding block info on free*/
	p = ((void *)pBlock) + sizeof(tBlockHeader);
	
	/* unlink block from head of free chain */
	pSuperblock->pFreeBlocksHead = pBlock->pNextFree;
	pBlock->pNextFree = NULL;
	
	/* update flags and free block counter*/
	pBlock->inUse = 1;	
	pSuperblock->numFreeBlocks--;
	
	DBG_EXIT;
	return p;
}

static void recycleSuperblock(unsigned int heapNum, unsigned int newSizeClass, tSuperblock *pSuperblock)
{	
	DBG_ENTRY;
	
	if (RECYCLED_CLASS == newSizeClass)
	{
		/* just move superblock as is to the 'all empty' class.*/
		removeSuperblockFromClass(heapNum, pSuperblock->sizeClass, pSuperblock);
		pSuperblock->sizeClass = RECYCLED_CLASS;
		addSuperblockToClass(heapNum, RECYCLED_CLASS, pSuperblock);
		/* not updating heap statistics. Superblock will carry old numBlocks and blockSize until recycled into new size class */
		return;
	}
	
	/* We are recycling into a new size class. Need to be careful here with the statistics, because the memory
		held by superblock depended on the previous size class, and on how many blocks we managed to fit
		in to this superblock while taking the block header size into consideration.. */
	updateMemoryHeld(heapNum, (-1)*(pSuperblock->numBlocks * pSuperblock->blockSize));
	
	/* Now overwrite numBlocks, blockSize and heap stats */
	initSuperblock(heapNum, newSizeClass, pSuperblock);
		
	DBG_EXIT;
}

static void checkInvariantAndMoveSuperblocks(unsigned int heapNum)
{	
	tSuperblock *pEmptyEnoughSuperblock;
	
	pEmptyEnoughSuperblock = findEmptyEnoughSuperblock(heapNum);
	DBG_MSG("pEmptyEnoughSuperblock=0x%x\n", (unsigned int)pEmptyEnoughSuperblock);
	
	while (pEmptyEnoughSuperblock  && isEmptyEnough(heapNum))
	{
		/* move superblock to global heap */
		moveSuperblockFromTo(heapNum, GLOBAL_HEAP, pEmptyEnoughSuperblock);
		pEmptyEnoughSuperblock = findEmptyEnoughSuperblock(heapNum);		
	}
}		

static void dumpHoard(char *title)
{
	int				heap, class;
	tHeap			*pHeap;
	tSizeClass		*pClass;
	tSuperblock		*pSb;
	
	printf("-----------------------------------------------------------------\n");
	printf("\nDUMP HOARD:");
	
	if (title) {printf("%s\n", title);} else {printf("\n");}
	
	for (heap = 0; heap < NUM_HEAPS; heap++)
	{
		pHeap = &s_hoard.heapArray[heap];
		printf("-----------------------------------------------------------------\n");
		printf("heap #%d: procId=%d inUse=%d held=%d\n",heap, pHeap->processorId, pHeap->statMemoryInUse, pHeap->statMemoryHeld);
		printf("-----------------------------------------------------------------\n");
		for (class = 0; class < NUM_SIZE_CLASSES; class++)
		{
			pClass = &pHeap->sizeClasses[class];
			printf("class #%d: size=%d head=0x%x tail=0x%x\n",
						class, pClass->size, (unsigned int)pClass->pHead, (unsigned int)pClass->pTail);
			pSb = pClass->pHead;
			while (pSb)
			{
				printf("Superblock: pNext=0x%x sizeClass=%d blockSize=%d ownerHeap=%d numBlocks=%d numFreeBlocks=%d pBlockArray=0x%x pFreeBlocksHead=0x%x\n",
						(unsigned int)pSb->pNext, pSb->sizeClass,
						pSb->blockSize, pSb->ownerHeap, pSb->numBlocks,
						pSb->numFreeBlocks, (unsigned int)pSb->pBlockArray, (unsigned int)pSb->pFreeBlocksHead);
				pSb = pSb->pNext;				
			}						
		}	
	}
	printf("--------------------END HOARD DUMP ------------------------------------\n");
}
