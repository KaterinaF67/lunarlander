#ifndef SEQUENCE_H
#define SEQUENCE_H


template <class T>
class Sequence {
public:
    virtual ~Sequence() = default;

    virtual T GetFirst()const = 0;
    virtual T GetLast()const = 0;
    virtual T Get(int index) const = 0;
    virtual Sequence<T>* GetSubsequence(int startIndex, int endIndex) = 0;
    virtual int GetLength()const = 0;
    virtual Sequence<T>* Append(T item) = 0;
    virtual Sequence<T>* Prepend(T item) = 0;
    virtual Sequence<T>* InsertAt(T item, int index) = 0;
    virtual Sequence <T>* Concat(Sequence <T>* list) = 0;

    virtual Sequence<T>* Map(T(*f)(T)) = 0;
    virtual Sequence<T>* Where(bool (*h)(T)) = 0;
    virtual T Reduce(T(*f)(T, T), T start) const = 0;
    // virtual Secuence<Pair<T1,T2>> *Zip(Sequence<T1>* seq1,Sequence<T2>* seq2)const =0;
};
#endif