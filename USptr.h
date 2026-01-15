#ifndef SMART_POINTERS_H
#define SMART_POINTERS_H

#include <utility>


template<typename T>
class UnqPtr {
private:
    T* ptr;
    std::size_t length;
    bool is_array;

public:
    UnqPtr() noexcept : ptr(nullptr), length(0), is_array(false) {}

    UnqPtr(T* p, bool arr = false, std::size_t len = 0) noexcept : ptr(p), length(len), is_array(arr) {}

    UnqPtr(const UnqPtr&) = delete;
    UnqPtr& operator=(const UnqPtr&) = delete;

    UnqPtr(UnqPtr&& other) noexcept
        : ptr(other.ptr), length(other.length), is_array(other.is_array) {
        other.ptr = nullptr; other.length = 0; other.is_array = false;
    }

    UnqPtr& operator=(UnqPtr&& other) noexcept {
        if (this != &other) {
            reset();
            ptr = other.ptr;
            length = other.length;
            is_array = other.is_array;
            other.ptr = nullptr; other.length = 0; other.is_array = false;
        }
        return *this;
    }

    ~UnqPtr() {
        reset();
    }

    static UnqPtr make_array(std::size_t n) {
        return UnqPtr(new T[n], true, n);
    }

    static UnqPtr from_array(T* arr, std::size_t n) {
        return UnqPtr(arr, true, n);
    }

    T* get() const noexcept { return ptr; }
    T& operator*() const noexcept { return *ptr; }
    T* operator->() const noexcept { return ptr; }
    explicit operator bool() const noexcept { return ptr != nullptr; }

    T& operator[](std::size_t i) const noexcept {
        return ptr[i];
    }

    std::size_t size() const noexcept { return is_array ? length : 0; }
    bool isArray() const noexcept { return is_array; }

    T* release() noexcept {
        T* tmp = ptr;
        ptr = nullptr;
        length = 0;
        is_array = false;
        return tmp;
    }

    void reset(T* p = nullptr, bool arr = false, std::size_t len = 0) noexcept {
        if (ptr) {
            if (is_array) delete[] ptr;
            else delete ptr;
        }
        ptr = p;
        is_array = arr;
        length = arr ? len : 0;
    }
};


template<typename T>
class ShrdPtr {
private:
    T* ptr;
    int* count;
    std::size_t length;
    bool is_array;

    void release() noexcept {
        if (count && --*count == 0) {
            if (is_array) delete[] ptr;
            else delete ptr;
            delete count;
        }
        ptr = nullptr; count = nullptr; length = 0; is_array = false;
    }

public:
    ShrdPtr() noexcept : ptr(nullptr), count(nullptr), length(0), is_array(false) {}

    ShrdPtr(T* p, bool arr = false, std::size_t len = 0): ptr(p), count(p ? new int(1) : nullptr), length(len), is_array(arr) {}

    ShrdPtr(UnqPtr<T>&& u) noexcept: ptr(u.release()), count(ptr ? new int(1) : nullptr), length(u.size()), is_array(u.isArray()) {}

    ShrdPtr(const ShrdPtr& other) noexcept
        : ptr(other.ptr), count(other.count), length(other.length), is_array(other.is_array) {
        if (count) ++*count;
    }

    ShrdPtr(ShrdPtr&& other) noexcept
        : ptr(other.ptr), count(other.count), length(other.length), is_array(other.is_array) {
        other.ptr = nullptr; other.count = nullptr; other.length = 0; other.is_array = false;
    }

    ShrdPtr& operator=(const ShrdPtr& other) noexcept {
        if (this != &other) {
            release();
            ptr = other.ptr; count = other.count;
            length = other.length; is_array = other.is_array;
            if (count) ++*count;
        }
        return *this;
    }

    ShrdPtr& operator=(ShrdPtr&& other) noexcept {
        if (this != &other) {
            release();
            ptr = other.ptr; count = other.count;
            length = other.length; is_array = other.is_array;
            other.ptr = nullptr; other.count = nullptr; other.length = 0; other.is_array = false;
        }
        return *this;
    }

    ~ShrdPtr() {
        release();
    }

    static ShrdPtr make_shared(T* p) {
        return ShrdPtr(p, false, 0);
    }

    static ShrdPtr make_array(std::size_t n) {
        return ShrdPtr(new T[n], true, n);
    }

    static ShrdPtr from_array(T* arr, std::size_t n) {
        return ShrdPtr(arr, true, n);
    }

    T* get() const noexcept { return ptr; }
    T& operator*() const noexcept { return *ptr; }
    T* operator->() const noexcept { return ptr; }
    explicit operator bool() const noexcept { return ptr != nullptr; }

    T& operator[](std::size_t i) const noexcept {
        return ptr[i];
    }

    int useCount() const noexcept { return count ? *count : 0; }
    bool unique() const noexcept { return useCount() == 1; }

    std::size_t size() const noexcept { return is_array ? length : 0; }
    bool isArray() const noexcept { return is_array; }

    void reset(T* p = nullptr, bool arr = false, std::size_t len = 0) {
        release();
        ptr = p; is_array = arr; length = arr ? len : 0;
        count = p ? new int(1) : nullptr;
    }
};

#endif 
