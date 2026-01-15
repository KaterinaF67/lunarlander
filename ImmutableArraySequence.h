#ifndef IMMUTABLEARRAYSEQUENCE_H
#define IMMUTABLEARRAYSEQUENCE_H

#include "ArraySequence.h"

template <class T>
class ImmutableArraySequence: public ArraySequence<T>{
private:
    ArraySequence<T>* createSeq(const DynamicArray<T>& d_array) const override {
        return new ImmutableArraySequence<T>(d_array);
    }
public:
    ImmutableArraySequence(T* items, int count) : ArraySequence<T>(items, count) {}
    ImmutableArraySequence() : ArraySequence<T>() {}
    ImmutableArraySequence(const DynamicArray<T>& d_array) : ArraySequence<T>(d_array) {}
    ~ImmutableArraySequence() override = default;
    ArraySequence<T>* Clone() const override {
        return createSeq(*this->array);
    }
    ArraySequence<T>* Instance()override { return this->Clone(); }
};
#endif