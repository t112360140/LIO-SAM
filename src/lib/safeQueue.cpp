#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>

template <typename T>
class SafeQueue {
private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
    size_t max_size_; // 設定上限，避免記憶體爆掉 (Backpressure)

public:
    SafeQueue(size_t max_size = 100) : max_size_(max_size) {}

    // 生產者專用：把資料推進去
    void push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        // 如果佇列滿了，生產者就在這裡稍微等一下
        cond_var_.wait(lock, [this]() { return queue_.size() < max_size_; });
        
        queue_.push(std::move(item));
        
        // 叫醒正在睡覺的消費者：「有新資料囉！」
        lock.unlock();
        cond_var_.notify_one(); 
    }

    // 消費者專用：把資料拿出來
    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        // 如果佇列空的，消費者就在這裡睡覺，直到被叫醒
        cond_var_.wait(lock, [this]() { return !queue_.empty(); });
        
        item = std::move(queue_.front());
        queue_.pop();
        
        // 叫醒可能因為佇列滿了而在等待的生產者
        lock.unlock();
        cond_var_.notify_one();
        
        return true;
    }
};