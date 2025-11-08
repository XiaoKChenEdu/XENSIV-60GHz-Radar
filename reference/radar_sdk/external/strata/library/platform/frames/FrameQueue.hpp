/**
 * @copyright 2018 Infineon Technologies
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include <Definitions.hpp>
#include <platform/interfaces/IFrameListener.hpp>
#include <platform/interfaces/IFrameQueue.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>


class FrameQueue :
    public IFrameQueue
{
public:
    STRATA_API FrameQueue();
    STRATA_API virtual ~FrameQueue();

    ///
    /// Set the maximum number of entries in the queue.
    /// When there are too many entries queued, the oldest one will be deleted (circular buffer).
    /// @param count The maximum number, 0 means no limitation
    STRATA_API void setMaxCount(uint32_t count);

    ///
    /// Clear the queue and free all frames
    ///
    STRATA_API void clear() override;

    ///
    /// Enqueues a frame at the end of the queue
    /// \param frame Pointer to the frame to enqueue. Ownership is taken by this function.
    ///
    STRATA_API void enqueue(IFrame *frame);

    ///
    /// \return the next frame in the queue
    /// \retval nullptr if the queue is empty
    ///
    STRATA_API IFrame *dequeue();

    ///
    /// Blocks until there is a new frame available or the queue is destroyed
    /// \param timeoutMs Time to wait in milliseconds for a new frame, 0 means wait forever (or until destruction)
    /// \return the next frame in the queue
    /// \retval nullptr on exit
    ///
    STRATA_API IFrame *blockingDequeue(uint16_t timeoutMs = 0) override;

    ///start
    /// Start functionality in case it was stopped before
    ///
    STRATA_API void start() override;

    ///
    /// Stop functionality and release all blocking calls in the blockingDequeue function
    ///
    STRATA_API bool stop() override;

private:
    void trimQueue();

    std::mutex m_lock;
    std::deque<IFrame *> m_queue;
    std::condition_variable m_cv;

    std::atomic<bool> m_queueing;  //true as long as the queue works
    uint32_t m_maxCount;           //maximum number of elements in the queue
};
