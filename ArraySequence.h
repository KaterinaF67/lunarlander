#ifndef ARRAYSEQUENCE_H
#define ARRAYSEQUENCE_H
#include <algorithm>
#include "Sequence.h"
#include "DynamicArray.h"
#include <stdexcept>
template <class T>
class ArraySequence: public Sequence<T>{
protected:
    DynamicArray<T>*array;

    virtual ArraySequence<T>* createSeq(const DynamicArray<T>& arr) const = 0;

    ArraySequence<T>* AppendInternal(T item){
        array->Resize(array->GetSize() + 1);
        array->Set(array->GetSize() - 1, item);
        return this;
    }
    ArraySequence<T>* PrependInternal(T item){
        array->Resize(array->GetSize() + 1);
        for (int i = array->GetSize() - 1; i > 0; i--) {
            array->Set(i, array->Get(i - 1));
        }
        array->Set(0, item);
        return this;
    }
    ArraySequence<T>* InsertAtInternal(T item, int index){
        array->Resize(array->GetSize() + 1);
        for (int i = array->GetSize() - 1; i > index; i--) {
            array->Set(i, array->Get(i - 1));
        }
        array->Set(index, item);
        return this;
    }
    ArraySequence<T>* GetSubsequenceInternal(int startIndex, int endIndex) {
        if (startIndex < 0 || endIndex >= array->GetSize() || startIndex > endIndex) {
            throw std::out_of_range("index out of range");
        }
        DynamicArray<T>* d_array = new DynamicArray<T>(endIndex-startIndex+1);
        for (int i=0;i<d_array->GetSize();i++){
            d_array->Set(i,array->Get(i+startIndex));
        }
        delete array;
        array=new DynamicArray<T>(*d_array);
        return this;
    }
    ArraySequence<T>* ConcatInternal(Sequence <T> *sequence){ //разные мб?

        DynamicArray<T>* d_array = new DynamicArray<T>(array->GetSize()+sequence->GetLength());
        for (int i=0;i<array->GetSize();i++){
            d_array->Set(i,array->Get(i));
        }
        for (int i=array->GetSize();i<d_array->GetSize();i++){
            d_array->Set(i,sequence->Get(i-array->GetSize()));
        }
        this->array->Resize(d_array->GetSize());
        for (int i = 0; i < array->GetSize(); i++) {
            this->array->Set(i, d_array->Get(i));
        }
        delete d_array;
        return this;
    }
    ArraySequence<T>* MapInternal(T (*f)(T)) {
        for (int i=0;i<array->GetSize();i++){
            array->Set(i,f(array->Get(i)));
        }
        return this;
    }
    ArraySequence<T>* WhereInternal(bool (*h)(T)){
        DynamicArray<T>* d_array= new DynamicArray<T>(0);
        for (int i=0; i<array->GetSize();i++){
            if (h(array->Get(i))){
                d_array->Resize(d_array->GetSize()+1);
                d_array->Set(d_array->GetSize()-1, array->Get(i));
            }
        }
        delete array;
        array=d_array;
        return this;
    }    

public:
    ArraySequence (T* items, int count): array(new DynamicArray<T>(items,count)){}
    ArraySequence (): array(new DynamicArray<T>(0)){}
    ArraySequence (const DynamicArray<T>& d_array): array(new DynamicArray<T>(d_array)){} //array?? в файле List
    ~ArraySequence() override { delete array; }

    T GetFirst()const override{
        if (array==nullptr||array->GetSize()==0) {
            throw std::out_of_range("index out of range");
        }
        return array->Get(0);
    }
    T GetLast()const override{
        if (array==nullptr||array->GetSize()==0) {
            throw std::out_of_range("index out of range");
        }
        return array->Get(array->GetSize()-1);
    }
    T Get(int index) const override{
        if (index<0||index>=array->GetSize()) {
            throw std::out_of_range("index out of range");
        }
        return array->Get(index);
    }
    int GetLength()const override{return array->GetSize();}
    
    virtual ArraySequence<T>* Instance()= 0;
    virtual ArraySequence<T>* Clone()const = 0;

    ArraySequence<T>* Append(T item) override {return Instance()->AppendInternal(item);}
    ArraySequence<T>*Prepend(T item) override  {return Instance()->PrependInternal(item);}
    ArraySequence<T>*InsertAt(T item, int index)override  {
        if (index<0||index>array->GetSize()) {
            throw std::out_of_range("index out of range");
        }
        return Instance()->InsertAtInternal(item,index);
    }
    ArraySequence<T>* GetSubsequence(int startIndex, int endIndex)override{
        if (startIndex<0||startIndex>=array->GetSize()||endIndex<0||endIndex>=array->GetSize()||endIndex<startIndex) {
            throw std::out_of_range("index out of range");
        }
        return Instance()->GetSubsequenceInternal(startIndex,endIndex);
    }
    ArraySequence <T>* Concat(Sequence <T> *list) override{return Instance()->ConcatInternal(list);}
    ArraySequence <T>* Map(T (*f)(T))override{return Instance()->MapInternal(f);}
    ArraySequence<T> *Where(bool (*h)(T)) override {return Instance()->WhereInternal(h);}
    T Reduce(T (*f)(T,T), T start) const override{
        T result=start;
        for (int i=0; i<array->GetSize();i++){
            result=f(array->Get(i),result);
        }
        return result;
    }
    // ArraySequence<Pair<T1,T2>> *Zip(Sequence<T1>* seq1,Sequence<T2>* seq2) const override{
    //     return Instance()->ZipInternal(seq1,seq2);
    // }
};
#endif
//в private f_internal
//в public f{return instance()->finternal(item)}
//  в mutable instance