#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>


#include <stdint.h>
typedef uint64_t pthread_id_np_t;

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

//Our version of the allocator will  have P + 1 heaps

//each heap (both global and per-processor) is a collection of superblocks

//Each superblock is an array of some number of blocks (objects) and contains a free list of its available blocks maintained in LIFO order to improve locality.
//All superblocks are the same size (S).

//All of the blocks in a superblock are in the same size class

/*
When there is no memory available in any superblock on a thread’s heap,
 the allocator obtains a superblock from the global heap if one is available.
 If the global heap is also empty, Hoard creates a new superblock
 by requesting virtual memory from the operating system and adds it to the thread’s heap. 
*/

/*
Hoard ﬁnds f-empty superblocks in constant time by dividing superblocks into a number of bins that are calld “fullness groups”.
Each bin contains a doubly-linked list of superblocks that are in a given fullness range
 (e.g., all superblocks that are between 3/4 and completely empty are in the same bin).
 Hoard moves superblocks from one group to another when appropriate, and always allocates from nearly-full superblocks. 
In our assignment, we advise to implement fullness ranges as following:
 all superblocks that are between 3/4 and completely empty are in the same bin.
 Others are in another bin. 
 */
  
  /*
  Heap number to be used by thread is thread ID modulo number of heaps.
  hash(thread) = thread.id % heaps.count
  */
  /*
  את הערמות מגדירים מראש על ה data segment
  את הערמות מגדירים מראש על ה data segment. זאת הקלה של הממ"ן. שמקצים רק 3 ערמות. 2 ארמות אחת לכל CPU ואחת גובאלית. 

לרשימה דו מקורשת ממפים איזור זיכרון גדול שגודלו כפולה של גודל הצמתים של רשימות משורשרת של סופרבלוקים. זה מוגדת באחד ה tasks.

  To allocate nodes for the linked list, first allocated some big pool of nodes using the following code snippet:
 
int  = open("/dev/zero", O_RDWR);
p = mmap(0, sizeof(ListNode) * MAX_NUMBER_OF_NODES, PROT_READ|PROT_WRITE, MAP_PRIVATE, fd, 0);
close(fd);

Note 1: You probably should add to the list node structure the indication whether this part of memory is allocated for the list mode or not. In this way you are actually implementing an auxiliary malloc for list nodes. For simplicity, this auxiliary malloc can take O(n) time. 
Note 2: Define MAX_NUMBER_OF_NODES = 2^16. After allocating MAX_NUMBER_OF_NODES auxiliary malloc will simply fail causing to library malloc fail as well.

  */
  
  /*
  Consider using the mmap to allocate virtual memory for superblocks and large (more then S/2) memory chunks
  int fd = open("/dev/zero", O_RDWR);
p = mmap(0, size, PROT_READ|PROT_WRITE, MAP_PRIVATE, fd, 0);
close(fd);

Note: don't forget to check return values!

  */
  
#define NUM_HEAPS			2
#define GLOBAL_HEAP			0
/* log2(SUPERBLOCK_SIZE) */
#define NUM_SIZE_CLASSES	17 /* 16 real size classes, +1 for 'any size' i.e. recycling completely empty superblocks*/	
#define RECYCLED_CLASS		16 /* the 17th slot is for completely empty, recycled superblocks that don't yet belong to any size class */

/* threshold for using hoard. If memory requested is more than this, then just mmap and return pointer to user */
#define HOARD_THRESHOLD_MEM_SIZE	SUPERBLOCK_SIZE/2		

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
	size_t				blockSize; 						/* all blocks are same size : 2^classIndex where classIndex is 0-15 */
	unsigned int		heapNum;						/* index into the heap array to the heap this superblock belongs to */
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

static void *	allocateLargeMemoryChunk(size_t	sz);
static void		deallocateLargeMemoryChunk(void * ptr, size_t sz);

/* Gets an index into the heap array based on the current thread's processor id */
static int		getHeapNumber(unsigned int *pHeapNumber);

/* Get size class by rounding up requested size to next highest power of 2, return that power. */
static int		getSizeClass    (size_t	requestedSize, int *pNextPowerOfTwo);

/* Allocate memory (memmap) for the superblock */
static tSuperblock	*	createSuperblock(size_t blockSize, unsigned int heapNum);

/* Initialize the superblock for a given size class and heap */
static void initSuperblock(tSuperblock *pSuperblock, size_t blockSize, unsigned int heapNum);

/* Initialize the statically allocated hoard structures - a better implementation is dynamic alloc. for non predetermined number of heaps */
//static void initHeaps();

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

/* return 1 if heap is under the fullness threshold, otherwise if heap is still full enough, return 0 */
static tSuperblock *findEmptyEnoughSuperblock(unsigned int heapNum);

/* move superblock from one heap to the other, update statistics ...*/
static void moveSuperblockFromTo(unsigned int fromHeap, unsigned int toHeap, tSuperblock *pSuperblock);

/* add superblock to the sorted-from-fullest-to-emptiest list of superblocks for the given size class and heap */
static void addSuperblockToClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock);

/* remove superblock from the sorted-from-fullest-to-emptiest list of superblocks for the given size class and heap */
static void removeSuperblockFromClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock);



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
	void			*p;
	unsigned int 	heapNum = GLOBAL_HEAP;
	unsigned int	requestedBlockSize = 0;
	int				sizeClassIndex;
	
	DBG_EXIT;
	
	DBG_MSG("requested size: %d\n", sz);

	if (sz >= HOARD_THRESHOLD_MEM_SIZE)
	{
		p = allocateLargeMemoryChunk(sz);
		if (!p)
		{
			perror(NULL);
			return 0;
		}
		return p;
	}
	
	initHeaps();
	/* hash to the correct heap */
	if (!getHeapNumber(&heapNum))
	{
		perror(NULL);
		return 0;
	}
	
	DBG_MSG("heap =  %d\n", heapNum);
	
	if (!getSizeClass (sz, &sizeClassIndex))
	{
		perror(NULL);
		return 0;
	}
	DBG_MSG("sizeClassIndex =  %d\n", sizeClassIndex);
	
	
	requestedBlockSize = 1 << sizeClassIndex;
	DBG_MSG("requestedBlockSize =  %d\n", requestedBlockSize);
	
	if (!allocMem(heapNum, sizeClassIndex))
	{
		perror(NULL);
		return 0
	}
	
	/* we found free memory! */
	
	/* below is just placeholder... */
	
	
	p = allocateLargeMemoryChunk(sz);
		if (!p)
		{
			perror(NULL);
			return 0;
		}
	return p;
	DBG_EXIT;
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
	if (ptr != NULL)
    	{
		size_t size = ((tBlockHeader *)(ptr - sizeof(tBlockHeader))) -> size + sizeof(tBlockHeader);
		
		if (size >= HOARD_THRESHOLD_MEM_SIZE)
		{
			deallocateLargeMemoryChunk(ptr, size);
		}
	}		
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
	*pHeapNumber = ((self >> 12) % NUM_HEAPS) + 1;
	return 1;	
}

/* Get size class by rounding up requested size to next highest power of 2, return that power. */
static int		getSizeClass    (size_t	requestedSize, int *pNextPowerOfTwo)
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

static tSuperblock	*	createSuperblock(size_t blockSize, int heapNum)
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
	initSuperblock(pNewSuperblock, blockSize, heapNum);
	
	DBG_EXIT;
	return pNewSuperblock;	
}

/* Initialize the superblock for a given size class and heap */
static void initSuperblock(tSuperblock *pSuperblock, size_t blockSize, int heapNum)
{

	void			*pNewBlock;				/* first block in superblock */
	void			*pEndOfSuperblock;		/* end of last block in superblock */
	size_t			blockSizeWithHeader;	/* user memory chunk + header */
	unsigned int	numBlocks = 0;			/* final count depends on block size */
	
	DBG_ENTRY;
	pNewBlock			= (void*) (pSuperblock -> pBlockArray);
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
		/* advance to next block */
		pNewBlock += blockSizeWithHeader;
		numBlocks++;
	}
	
	DBG_MSG("numBlocks in superblock with class size %d:  %d\n",blockSize, numBlocks);
	
	/* init the superblock */
	pSuperblock->pPrev = 0;
	pSuperblock->pNext = 0;						
	pSuperblock->blockSize = blockSize;
	pSuperblock->heapNum = heapNum;
	pSuperblock->numBlocks = numBlocks;
	pSuperblock->numFreeBlocks = numBlocks;	
	
	DBG_EXIT;
}


static void * allocMem(int heapNum, int sizeClass)
{
	/* Is there a free block in this heap (in the appropriate size class) */
	p = allocFromFreeBlockInHeap(heapNum, sizeClass);
	if (p)
	{
		return p;
	}
	
	/* So try the global heap */	
	p = allocFromFreeBlockInHeap(GLOBAL_HEAP, sizeClass);
	if (p)
	{
		return p;
	}
	/* No free chunk in global heap either. So try to allocate from a new superblock allocated from OS */ 
	/* get a new superblock for this heap and add to appropriate size class */
	p = allocFromFreeBlockInNewSuperblock(heapNum, sizeClass);
		
	/* whether we failed or succeeded return pointer  -will either be NULL or point to allocated block */
	return p;	
	
}



/* search for free block in given size class, return pointer to block if found, otherwise NULL. Update heap statistics */
static void * allocFromFreeBlockInHeap(unsigned int heapNum, unsigned int sizeClass)
{
	
}

/* create a new superblock, add to the given heap and size class, check emptiness invariant, update heap statistics, return pointer to requested block of memory */
static void *allocFromFreeBlockInNewSuperblock(unsigned int heapNum, unsigned int sizeClass)
{
	unsigned int	requestedBlockSize;
	tSuperblock		*pNewSuperblock;
	
	DBG_ENTRY;
	requestedBlockSize = 1 << sizeClassIndex;
	pNewSuperblock = createSuperblock(requestedBlockSize, heapNum);
	if (!pNewSuperblock)
	{
		DBG_EXIT;
		return 0;
	}
	
	updateMemoryInUse(heapNum, requestedBlockSize);
	
	/* Now attach the new superblock to the correct size class */
	
	/* Update the heap statistics */
	
	/* Check heap invariants, if necessary move superblock to global heap */
	updateMemoryHeld(heapNum, (pNewSuperblock -> numBlocks * blockSize));
	updateMemoryInUse(heapNum, blockSize);
	
	if (isTooEmpty(heapNum))
	{
		/* move superblock to global heap */
		moveSuperblockFromTo(heapNum, GLOBAL_HEAP);
		/* update statistics */
		updateMemoryHeld(GLOBAL_HEAP, (pNewSuperblock -> numBlocks * blockSize));
		updateMemoryInUse(GLOBAL_HEAP, blockSize);
		updateMemoryHeld(heapNum, (-1)*(pNewSuperblock -> numBlocks * blockSize));
		updateMemoryInUse(heapNum, (-1)*blockSize);
	}
		
	
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
	'at least f empty' */
	pHeap = &s_hoard.heapArray[heapNum];
	
	for (sizeClassIdx = 0; sizeClassIdx < NUM_SIZE_CLASSES; sizeClassIdx++)
	{
		pSizeClass = &pHeap->sizeClasses[sizeClassIdx];
		/* just check the tail since the list is ordered from full to least full */
		pSuperblock = &pSizeClass->pTail;
		if ((pSuperblock->numFreeBlocks/pSuperblock->numBlocks) > FULLNESS_THRESHOLD_F)
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
	
}

/* add superblock to the sorted-from-fullest-to-emptiest list of superblocks for the given size class and heap */
static void addSuperblockToClass(unsigned int heapNum, unsigned int sizeClass, tSuperblock *pSuperblock)
{
	float			emptyFactor, tempEmptyFactor;
	tSuperblock		*pPrevSb;
	tSuperblock		*pTempSb;
	tSizeClass		*pSizeClass;
	DBG_ENTRY;
	
	pSizeClass = &s_hoard.heapArray[heapNum].sizeClasses[sizeClass];
	emptyFactor = (pSuperblock->numFreeBlocks/pSuperblock->numBlocks);
	/* make sure the superblock to be attached isn't still chained to its previous size class linked list */
	pSuperblock->pNext = NULL;
	pSuperblock->pPrev = NULL;	
	
	/* we'll start the search from the tail because usually we're adding empty superblocks. This algo. can be improved */
	pTempSb = pSizeClass->pTail;
	if (!pTempSb)
	{
		/* this is the first superblock in the chain */
		pSizeClass->pHead = pSizeClass->pTail = pSuperblock;	
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
	
}


