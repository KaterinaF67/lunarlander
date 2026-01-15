#ifndef DYNAMICARRAY_H
#define DYNAMICARRAY_H

#include <stdexcept>
#include <algorithm>
#include <utility>

template <class T>
class DynamicArray {
private:
    T* data;
    int sz;

public:
    DynamicArray() : data(nullptr), sz(0) {}

    DynamicArray(int n) : data(nullptr), sz(0) {
        if (n > 0) {
            sz = n;
            data = new T[n];
            for (int i = 0; i < n; ++i) data[i] = T();
        }
    }

    DynamicArray(T* items, int count) : data(nullptr), sz(0) {
        if (count > 0) {
            sz = count;
            data = new T[sz];
            for (int i = 0; i < sz; ++i)
                data[i] = items[i];
        }
    }

    DynamicArray(const DynamicArray& other) : data(nullptr), sz(0) {
        if (other.sz > 0) {
            sz = other.sz;
            data = new T[sz];
            for (int i = 0; i < sz; ++i)
                data[i] = other.data[i];
        }
    }

    DynamicArray& operator=(const DynamicArray& other) {
        if (this != &other) {
            delete[] data;
            sz = other.sz;
            if (sz > 0) {
                data = new T[sz];
                for (int i = 0; i < sz; ++i)
                    data[i] = other.data[i];
            } else {
                data = nullptr;
            }
        }
        return *this;
    }

    ~DynamicArray() {
        delete[] data;
    }

    int size() const { return sz; }
    bool empty() const { return sz == 0; }

    T& operator[](int i) {
        if (i < 0 || i >= sz) throw std::out_of_range("index");
        return data[i];
    }

    const T& operator[](int i) const {
        if (i < 0 || i >= sz) throw std::out_of_range("index");
        return data[i];
    }

    T& front() { return data[0]; }
    T& back()  { return data[sz - 1]; }

    const T& front() const { return data[0]; }
    const T& back()  const { return data[sz - 1]; }

    T* begin() { return data; }
    T* end()   { return data + sz; }

    const T* begin() const { return data; }
    const T* end()   const { return data + sz; }

    void Resize(int n) {
        if (n == sz) return;
        if (n <= 0) {
            delete[] data;
            data = nullptr;
            sz = 0;
            return;
        }
        T* nd = new T[n];
        int m = std::min(sz, n);
        for (int i = 0; i < m; ++i) nd[i] = data[i];
        for (int i = m; i < n; ++i) nd[i] = T();
        delete[] data;
        data = nd;
        sz = n;
    }

    void clear() {
        Resize(0);
    }

    void push_back(const T& value) {
        Resize(sz + 1);
        data[sz - 1] = value;
    }

    template<typename... Args>
    void emplace_back(Args&&... args) {
        Resize(sz + 1);
        data[sz - 1] = T(std::forward<Args>(args)...);
    }

    void erase(T* it) {
        int idx = int(it - data);
        if (idx < 0 || idx >= sz) return;
        for (int i = idx; i < sz - 1; ++i)
            data[i] = data[i + 1];
        Resize(sz - 1);
    }

    int GetSize() const { return sz; }

    T Get(int index) const {
        if (index < 0 || index >= sz) throw std::out_of_range("Get()");
        return data[index];
    }

    void Set(int index, const T& value) {
        if (index < 0 || index >= sz) throw std::out_of_range("Set()");
        data[index] = value;
    }
};

#endif
