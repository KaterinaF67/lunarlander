#ifndef LAZYSEQUENCE_H
#define LAZYSEQUENCE_H

#include "Sequence.h"
#include "ImmutableArraySequence.h"
#include "USptr.h"
#include <functional>
#include <stdexcept>

class Cardinal {
private:
    size_t value;
    bool is_infinite; //беск

public:
    Cardinal(size_t v = 0, bool inf = false) : value(v), is_infinite(inf) {}

    bool isInfinite() const { return is_infinite; }
    size_t getValue() const { return value; }

    static Cardinal infinite() { return Cardinal(0, true); }

    bool operator==(const Cardinal& other) const {
        return is_infinite == other.is_infinite && (!is_infinite ? value == other.value : true);
    }

    bool operator>=(const Cardinal& other) const {
        if (is_infinite) return true;
        if (other.is_infinite) return false;
        return value >= other.value;
    }
};

template<typename T>
class Generator {
private:
    std::function<T(int)> generator_func;
    std::function<bool(int)> has_next_func;

public:
    Generator(std::function<T(int)> gen_func, std::function<bool(int)> has_func = nullptr): generator_func(gen_func), has_next_func(has_func) {}

    T GetNext(int index) {
        if (has_next_func && !has_next_func(index)) {
            throw std::out_of_range("Index out of range");
        }
        return generator_func(index);
    }

    bool HasNext(int index) const {
        return has_next_func ? has_next_func(index) : true;
    }
};

template<typename T>
class LazySequence : public Sequence<T> {
private:
    mutable UnqPtr<ImmutableArraySequence<T>> materialized_data;
    mutable size_t materialized_count;

    std::function<T(int)> generator_function;
    std::function<bool(int)> finite_check;

    mutable Cardinal length;
    mutable bool is_finite; //k
    mutable bool length_determined;
   
    void materialize_up_to(int index) const {
        if (index < 0) {
            throw std::out_of_range("index out of range");
        }
    
        if (materialized_count > static_cast<size_t>(index)) {
            return;
        }
        size_t target_count = index + 1;
    
        T* new_array = new T[target_count];
        for (size_t i = 0; i < materialized_count; i++) {
            new_array[i] = materialized_data->Get(i);
        }
    
        for (size_t i = materialized_count; i < target_count; i++) {
            if (finite_check && !finite_check(i)) {
                is_finite = true;
                length = Cardinal(i);
                length_determined = true;
                delete[] new_array;
                throw std::out_of_range("index out of range");
            }
            new_array[i] = generator_function(i);
        }
    
        materialized_data.reset(new ImmutableArraySequence<T>(new_array, target_count));
        materialized_count = target_count;
    
        delete[] new_array;
    }

    void fully_materialize() const {
        if (!is_finite) return;
        if (length_determined) return;
        
        size_t real_length = 0;
        while (finite_check && finite_check(real_length)) {
            real_length++;
        }
        
        length = Cardinal(real_length);
        length_determined = true;
        
        if (real_length == 0) return;
        
        T* new_array = new T[real_length];
        for (size_t i = 0; i < real_length; i++) {
            new_array[i] = generator_function(i);
        }
        
        materialized_data.reset(new ImmutableArraySequence<T>(new_array, real_length));
        materialized_count = real_length;
        delete[] new_array;
    }


public:
    LazySequence(): materialized_data(nullptr), materialized_count(0), generator_function(nullptr), finite_check(nullptr), length(0), is_finite(true), length_determined(true){}

    LazySequence(T* items, int count) : materialized_count(0), length(count), is_finite(true), length_determined(true) {
        ShrdPtr<T> shared_items = ShrdPtr<T>::make_array(count);
        for (int i = 0; i < count; i++) {
            shared_items.get()[i] = items[i];
        }

        generator_function = [shared_items, count](int index) -> T {
            if (index < 0 || index >= count) {
                throw std::out_of_range("Index out of range");
            }
            return shared_items.get()[index];
        };

        finite_check = [count](int index) -> bool {
            return index < count;
        };
    }

    LazySequence(Sequence<T>* seq) : materialized_count(0), length(seq->GetLength()), is_finite(true), length_determined(true) {
        int seq_length = seq->GetLength();
        ShrdPtr<T> shared_items = ShrdPtr<T>::make_array(seq_length);
        for (int i = 0; i < seq_length; i++) {
            shared_items.get()[i] = seq->Get(i);
        }

        generator_function = [shared_items, seq_length](int index) -> T {
            if (index < 0 || index >= seq_length) {
                throw std::out_of_range("Index out of range");
            }
            return shared_items.get()[index];
        };

        finite_check = [seq_length](int index) -> bool {
            return index < seq_length;
        };
    }

    LazySequence(std::function<T(int)> gen_func, std::function<bool(int)> finite_func = nullptr): materialized_count(0), is_finite(finite_func != nullptr), length_determined(false),generator_function(gen_func), finite_check(finite_func) {
        if (is_finite) {
            length = Cardinal(0); 
        } else {
            length = Cardinal::infinite();
            length_determined = true;
        }
    }

    LazySequence(const LazySequence<T>& other) 
        : materialized_count(other.materialized_count),
          generator_function(other.generator_function), 
          finite_check(other.finite_check),
          length(other.length), 
          is_finite(other.is_finite),
          length_determined(other.length_determined) {
        if (other.materialized_data) {
            T* temp_array = new T[other.materialized_count];
            for (size_t i = 0; i < other.materialized_count; i++) {
                temp_array[i] = other.materialized_data->Get(i);
            }
            materialized_data.reset(new ImmutableArraySequence<T>(temp_array, other.materialized_count));
            delete[] temp_array;
        }
    }

    ~LazySequence() override = default;

    T GetFirst() const override {
        if (is_finite && length_determined && length.getValue() == 0) {
            throw std::out_of_range("index out of range");
        }
        materialize_up_to(0);
        if (materialized_count == 0) { 
            throw std::out_of_range("index out of range");
        }
        return materialized_data->GetFirst();
    }

    T GetLast() const override {
        if (!is_finite) {  // ф-и нет
            throw std::out_of_range("out of range");
        }
        fully_materialize();
        size_t last_index = length.getValue();
        if (last_index == 0) {
            throw std::out_of_range("index out of range");
        }
        return materialized_data ? materialized_data->Get(last_index - 1): throw std::out_of_range("index out of range");
    }

    T Get(int index) const override {
        if (index < 0) {
            throw std::out_of_range("index out of range");
        }
        materialize_up_to(index);
        if (static_cast<size_t>(index) >= materialized_count) {
            throw std::out_of_range("index out of range");
        }
        return materialized_data->Get(index);
    }

    Sequence<T>* GetSubsequence(int startIndex, int endIndex) override {
        if (startIndex < 0 || endIndex < 0 || startIndex > endIndex) {
            throw std::out_of_range("index out of range");
        }
        materialize_up_to(endIndex);
        if (static_cast<size_t>(endIndex) >= materialized_count) {
            throw std::out_of_range("index out of range");
        }

        T* temp_array = new T[endIndex - startIndex + 1];
        for (int i = 0; i < endIndex - startIndex + 1; i++) {
            temp_array[i] = materialized_data->Get(startIndex + i);
        }
        LazySequence<T>* result = new LazySequence<T>(temp_array, endIndex - startIndex + 1);
        delete[] temp_array;
        return result;
    }

    int GetLength() const override {
        if (!is_finite) {
            throw std::out_of_range("out of range");
        }
        fully_materialize();
        return length.getValue();
    }

    Cardinal GetCardinalLength() const {
        return length;
    }

    size_t GetMaterializedCount() const {
        return materialized_count;
    }

    Sequence<T>* Append(T item) override {
        if (!is_finite) {
            throw std::out_of_range("out of range");
        }
        
        auto current_generator = generator_function;
        
        int orig_length = GetLength();

        auto new_generator = [current_generator, orig_length, item](int index) -> T {
            if (index < orig_length) {
                return current_generator(index);
            }
            if (index == orig_length) {
                return item;
            }
            throw std::out_of_range("index out of range");
        };

        auto new_finite_check = [orig_length](int index) -> bool {
            return index <= orig_length;
        };
        
        return new LazySequence<T>(new_generator, new_finite_check);
    }


    Sequence<T>* Prepend(T item) override {
        auto current_generator = generator_function;
        auto current_finite_check = finite_check;

        auto new_generator = [current_generator, item](int index) -> T {
            if (index == 0) {
                return item;
            }
            return current_generator(index - 1);
        };

        std::function<bool(int)> new_finite_check;
        if (is_finite) {
            new_finite_check = [current_finite_check](int index) -> bool {
                if (index == 0) return true;
                return current_finite_check(index - 1);
            };
        } else {
            new_finite_check = nullptr;
        }

        return new LazySequence<T>(new_generator, new_finite_check);
    }

    Sequence<T>* InsertAt(T item, int index) override {
        if (index < 0) {
            throw std::out_of_range("index out of range");
        }

        auto current_generator = generator_function;
        auto current_finite_check = finite_check;

        auto new_generator = [current_generator, item, index](int i) -> T {
            if (i == index) {
                return item;
            }
            if (i < index) {
                return current_generator(i);
            }
            return current_generator(i - 1);
        };

        std::function<bool(int)> new_finite_check;
        if (is_finite) {
            new_finite_check = [current_finite_check](int i) -> bool {
                if (i == 0) return true;
                return current_finite_check(i - 1) || current_finite_check(i);
            };
        } else {
            new_finite_check = nullptr;
        }

        return new LazySequence<T>(new_generator, new_finite_check);
    }

    Sequence<T>* Concat(Sequence<T>* list) override {
        if (!is_finite) {
            return new LazySequence<T>(*this);
        }

        auto current_generator = generator_function;
        auto current_finite_check = finite_check;

    
        int first_len = GetLength();
        int list_length = list->GetLength();
    
        ShrdPtr<T> list_data = ShrdPtr<T>::make_array(list_length);
        for (int i = 0; i < list_length; i++) {
            list_data.get()[i] = list->Get(i);
        }

        auto new_generator = [current_generator, list_data, first_len, list_length](int index) -> T {
            if (index < first_len) {
                return current_generator(index);
            }
            int second_index = index - first_len;
            if (second_index >= 0 && second_index < list_length) {
                return list_data.get()[second_index];
            }
            throw std::out_of_range("Index out of range");
        };

        auto new_finite_check = [first_len, list_length](int index) -> bool {
            return index < first_len + list_length;
        };

        return new LazySequence<T>(new_generator, new_finite_check);
    }


    
    Sequence<T>* Map(T (*f)(T)) override {
        auto current_generator = generator_function;
        auto current_finite_check = finite_check;

        auto new_generator = [current_generator, f](int index) -> T {
            return f(current_generator(index));
        };

        return new LazySequence<T>(new_generator, current_finite_check);
    }
    Sequence<T>* Where(bool (*h)(T)) override {
        auto current_generator = generator_function;
        auto current_finite_check = finite_check;
        auto new_generator = [current_generator, current_finite_check, h](int index) -> T {
            int original_index = 0;
            int filtered_count = 0;
            
            while (true) {
                if (current_finite_check && !current_finite_check(original_index)) {
                    throw std::out_of_range("index out of range");
                }
                
                T current = current_generator(original_index);
                
                if (h(current)) {
                    if (filtered_count == index) {
                        return current;
                    }
                    filtered_count++;
                }
                
                original_index++;
            }
        };
        

        std::function<bool(int)> new_finite_check = current_finite_check;
        
        return new LazySequence<T>(new_generator, new_finite_check);
    }



    T Reduce(T (*f)(T, T), T start) const override {
        if (!is_finite) {
            throw std::out_of_range("out of range");
        }

        T result = start;
        fully_materialize();

        for (size_t i = 0; i < length.getValue(); i++) {
            result = f(result, Get(i));
        }

        return result;
    }
};

#endif 