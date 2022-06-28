#include <lx_emul.h>
#include <linux/slab.h>


#include <asm-generic/delay.h>

void __const_udelay(unsigned long xloops)
{
	lx_emul_time_udelay(xloops / 0x10C7UL);
}


#include <linux/dma-mapping.h>

int dma_supported(struct device *dev, u64 mask)
{
	return 1;
}


#include <linux/dmapool.h>

struct dma_pool { size_t size; };

struct dma_pool * dma_pool_create(const char * name,
                                  struct device * dev,
                                  size_t size,
                                  size_t align,
                                  size_t boundary)
{
	struct dma_pool * pool = kmalloc(sizeof(struct dma_pool), GFP_KERNEL);
	pool->size = size;
	return pool;
}


void * dma_pool_alloc(struct dma_pool * pool, gfp_t mem_flags, dma_addr_t * handle)
{
	void * ret =
		lx_emul_mem_alloc_aligned_uncached(pool->size, PAGE_SIZE);
	*handle = lx_emul_mem_dma_addr(ret);
	return ret;
}


void dma_pool_free(struct dma_pool * pool,void * vaddr,dma_addr_t dma)
{
	lx_emul_mem_free(vaddr);
}

