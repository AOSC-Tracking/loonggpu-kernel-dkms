#ifndef LOONGGPU_DMA_RESV_HELPER_H
#define LOONGGPU_DMA_RESV_HELPER_H

#include <conftest.h>

#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
#include <linux/dma-resv.h>
#else
#include <linux/reservation.h>
#endif

#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
typedef struct dma_resv lg_dma_resv_t;
typedef struct dma_resv_list lg_dma_resv_list_t;
#else
typedef struct reservation_object lg_dma_resv_t;
typedef struct reservation_object_list lg_dma_resv_list_t;
#endif

#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
#define lg_dma_resv_held(resv) dma_resv_held(resv)
#else
#define lg_dma_resv_held(resv) reservation_object_held(resv)
#endif

static inline lg_dma_resv_list_t *
lg_dma_resv_get_list(lg_dma_resv_t *resv)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT) && defined(LG_DMA_RESV_GET_LIST)
	return dma_resv_get_list(resv);
#elif defined(LG_LINUX_RESERVATION_H_PRESENT)
	return reservation_object_get_list(resv);
#else
	return NULL;
#endif
}

static inline struct dma_fence *
lg_dma_resv_get_excl(lg_dma_resv_t *obj)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT) && defined(LG_DMA_RESV_GET_EXCL_PRESENT)
	return dma_resv_get_excl(obj);
#elif defined(LG_LINUX_RESERVATION_H_PRESENT)
	return reservation_object_get_excl(obj);
#else
	return NULL;
#endif
}

static inline int
lg_dma_resv_reserve_shared(lg_dma_resv_t *obj, unsigned int num_fences)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
#if defined(LG_HAS_DMA_RESV_RESERVE_FENCES)
	return dma_resv_reserve_fences(obj, num_fences);
#else
	return dma_resv_reserve_shared(obj, num_fences);
#endif
#else
	return reservation_object_reserve_shared(obj);
#endif
}

static inline int lg_dma_resv_lock(lg_dma_resv_t *obj,
                                    struct ww_acquire_ctx *ctx)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
    return dma_resv_lock(obj, ctx);
#else
    return ww_mutex_lock(&obj->lock, ctx);
#endif
}

static inline void lg_dma_resv_unlock(lg_dma_resv_t *obj)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
    dma_resv_unlock(obj);
#else
    ww_mutex_unlock(&obj->lock);
#endif
}

static inline bool __must_check
lg_dma_resv_trylock(lg_dma_resv_t *obj)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
    return dma_resv_trylock(obj);
#else
    return ww_mutex_trylock(&obj->lock);
#endif
}

static inline long
lg_dma_resv_wait_timeout_rcu(lg_dma_resv_t *obj, int wait_flag, bool wait_all,
			     bool intr, unsigned long timeout)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
#if defined(LG_DMA_RESV_WAIT_TIMEOUT)
	return dma_resv_wait_timeout(obj, (enum dma_resv_usage)wait_flag, intr, timeout);
#else
	return dma_resv_wait_timeout_rcu(obj, wait_all, intr, timeout);
#endif
#else
	return reservation_object_wait_timeout_rcu(obj, wait_all, intr, timeout);
#endif
}

static inline int
lg_dma_resv_get_fences_rcu(lg_dma_resv_t *obj, struct dma_fence **pfence_excl,
			   unsigned *pshared_count, struct dma_fence ***pshared,
			   int usage)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
#if defined(LG_DMA_RESV_WAIT_TIMEOUT)
	return dma_resv_get_fences(obj, (enum dma_resv_usage)usage,
					pshared_count, pshared);
#else
	return dma_resv_get_fences_rcu(obj, pfence_excl, pshared_count, pshared);
#endif
#else
	return reservation_object_get_fences_rcu(obj, pfence_excl,
						 pshared_count, pshared);
#endif
}

static inline void
lg_dma_resv_add_shared_fence(lg_dma_resv_t *obj, struct dma_fence *fence)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
#if defined(LG_DMA_RESV_ADD_FENCE)
	dma_resv_add_fence(obj, fence, DMA_RESV_USAGE_READ);
#else
	dma_resv_add_shared_fence(obj, fence);
#endif
#else
	reservation_object_add_shared_fence(obj, fence);
#endif
}

static inline void
lg_dma_resv_add_excl_fence(lg_dma_resv_t *obj, struct dma_fence *fence)
{
#if defined(LG_LINUX_DMA_RESV_H_PRESENT)
#if defined(LG_DMA_RESV_ADD_FENCE)
	dma_resv_add_fence(obj, fence, DMA_RESV_USAGE_WRITE);
#else
	dma_resv_add_excl_fence(obj, fence);
#endif
#else
	reservation_object_add_excl_fence(obj, fence);
#endif
}

#endif
