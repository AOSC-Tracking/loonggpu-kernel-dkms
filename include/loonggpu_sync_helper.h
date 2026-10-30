#if !defined(LG_SYNC_HELPER_H)
#define LG_SYNC_HELPER_H

void *loonggpu_sync_get_owner(struct dma_fence *f);
bool loonggpu_sync_same_dev(struct loonggpu_device *adev, struct dma_fence *f);

static bool loonggpu_sync_test_fence(struct loonggpu_device *adev,
                                  enum loonggpu_sync_mode mode,
                                  void *owner, struct dma_fence *f)
{
        void *fence_owner = loonggpu_sync_get_owner(f);

        /* Always sync to moves, no matter what */
        if (fence_owner == LOONGGPU_FENCE_OWNER_UNDEFINED)
                return true;

        /* We only want to trigger KCD eviction fences on
         * evict or move jobs. Skip KCD fences otherwise.
         */
        if (fence_owner == LOONGGPU_FENCE_OWNER_KCD &&
            owner != LOONGGPU_FENCE_OWNER_UNDEFINED)
                return false;

        /* Never sync to VM updates either. */
        if (fence_owner == LOONGGPU_FENCE_OWNER_VM &&
            owner != LOONGGPU_FENCE_OWNER_UNDEFINED)
                return false;

        /* Ignore fences depending on the sync mode */
        switch (mode) {
        case LOONGGPU_SYNC_ALWAYS:
                return true;

        case LOONGGPU_SYNC_NE_OWNER:
                if (loonggpu_sync_same_dev(adev, f) &&
                    fence_owner == owner)
                        return false;
                break;

        case LOONGGPU_SYNC_EQ_OWNER:
                if (loonggpu_sync_same_dev(adev, f) &&
                    fence_owner != owner)
                        return false;
                break;

        case LOONGGPU_SYNC_EXPLICIT:
                return false;
        }

        return true;
}

#endif /* LG_SYNC_HELPER_H */
