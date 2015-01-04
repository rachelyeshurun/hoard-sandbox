#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#if 1
#define DBG_MSG printf("\n%s (%d): ", __LINE__, __FUNCTION__);printf
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
 
int fd = open("/dev/zero", O_RDWR);
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
  
#define MAX_NUM_HEAPS		2
/* log2(SUPERBLOCK_SIZE) */
#define NUM_SIZE_CLASSES	16	

/* threshold for using hoard. If memory requested is less than this, then just mmap and return pointer to user */
#define HOARD_THRESHOLD_MEM_SIZE	SUPERBLOCK_SIZE/2			

/* header of memory block */ 
typedef struct sBlockNode
{
	unsigned int		isFree;							/* 1 if block is free memory, otherwise 0 */
	struct sBlockNode	*pNext;							/* pointer to next block node in the LIFO list of free blocks (only relevant if free = 1)*/
	unsigned int		size;							/* size of allocated memory as available for user */
	struct sSuperblock	*pMySuperblock;					/* pointer back to superblock that contains this block */
} tBlockNode;

typedef struct sSuperblock
{
	struct sSuperblock	*pPrev;
	struct sSuperblock	*pNext;							
	unsigned int		blockSize; 						/* all blocks are same size : 2^classIndex where classIndex is 0-15 */
	unsigned int		heapNum;						/* index into the heap array to the heap this superblock belongs to */
	unsigned int 		numBlocks; 						/* SUPERBLOCK_SIZE/blockSize */
	tBlockNode			*pBlockArray;					/* mmap space needed according to num blocks - depends on class size */
	unsigned int		numFreeBlocks;					/* keep track of number of free blocks	*/				
	tBlockNode			*pFreeBlocksTail; 				/* LIFO linked list of free block nodes */
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
	unsigned long		statMemoryInUse;				/* The amount of memory in use by this heap */
	unsigned long		statMemoryHeld;					/* The amount of memory held in this heap that was allocated from the operating system */
	tSizeClass			sizeClasses[NUM_SIZE_CLASSES]; 	/* hold size classed for all sizes from 1 to log2(SUPERBLOCK_SIZE) */	
	tSizeClass			emptySuperblocks;				/* completely empty superblocks are recycled for use by any size class */	
} tHeap;


typedef struct sHoard
{
	unsigned int		numHeaps;
	tHeap				heapArray[MAX_NUM_HEAPS];
}tHoard;

/* Heaps are defined as a static array in the heap - reside in the data segment */
static tHoard		s_hoard;	

typedef struct sHeader
{
   /* 
    * @breaf - size of allocated memory as available for user
    */
   int mSize; 


} Header;	

static void *	allocateLargeMemoryChunk(size_t	sz);
static void		deallocateLargeMemoryChunk(void * ptr, size_t sz);
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
	void *p;
	
	DBG_EXIT;
	
	printf("start malloc 0x%x\n", &s_hoard);
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
	
	return 0;
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
		int size = ((tBlockNode *)(ptr - sizeof(tBlockNode))) -> size + sizeof(tBlockNode);
		
		if (size >= HOARD_THRESHOLD_MEM_SIZE)
		{
			    deallocateLargeMemoryChunk(ptr, size);
			if (!p)
			{
				perror(NULL);
				return 0;
			}
			return p;
		}
	}	
	printf("myfree\n");
	
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
	DBG_ENTRY;	
	fd = open("/dev/zero", O_RDWR);
	
	if (fd == -1){
		return 0;
	}
	
	p = mmap(0, sz + sizeof(tBlockNode), PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);
	close(fd);

	if (p == MAP_FAILED){
		return 0;
	}

	((tBlockNode *) p) -> size = sz;

	DBG_EXIT;
	return (p + sizeof(tBlockNode));	
}

static void deallocateLargeMemoryChunk(void * ptr, sz)
{
	int fd;
	void *p;
	
	DBG_ENTRY;
	
	if (munmap(ptr - sizeof(tBlockNode), sz) < 0)
	{
		 perror(NULL);
	}


	DBG_EXIT;
}
