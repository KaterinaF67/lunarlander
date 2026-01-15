#ifndef STREAMS_H
#define STREAMS_H

#include <string>
#include <fstream>
#include <sstream>
#include <functional>
#include <exception>
#include "LazySequence.h"

class EndOfStreamException : public std::exception {
public:
    const char* what() const noexcept override {
        return "End of stream reached";
    }
};

template<typename T>
class Deserializer {
public:
    virtual ~Deserializer() = default;
    virtual std::string Serialize(const T& item) const = 0;
    virtual T Deserialize(const std::string& str) const = 0;
};

template<typename T, typename U> class MappedReadOnlyStream;
template<typename T> class FilteredReadOnlyStream;

template<typename T>
class ReadOnlyStream {
protected:
    std::ifstream* fileStream;
    std::stringstream* stringStream;
    T* arrayData;
    size_t arraySize;
    size_t currentPosition;
    bool isFileStream;
    bool isStringStream;
    bool isArrayStream;
    bool opened;
    Deserializer<T>* deserializer;
    
    LazySequence<T>* lazySequence; //
    bool isLazyStream;  //

public:
    ReadOnlyStream(const std::string& filename, Deserializer<T>* deser)
        : fileStream(new std::ifstream(filename)), stringStream(nullptr),
          lazySequence(nullptr), arrayData(nullptr), arraySize(0), currentPosition(0),
          isFileStream(true), isStringStream(false), isArrayStream(false),
          isLazyStream(false), opened(false), deserializer(deser) {}

    ReadOnlyStream(const std::string& data, Deserializer<T>* deser, bool isString)
        : fileStream(nullptr), stringStream(new std::stringstream(data)),
          lazySequence(nullptr), arrayData(nullptr), arraySize(0), currentPosition(0),
          isFileStream(false), isStringStream(isString), isArrayStream(false),
          isLazyStream(false), opened(false), deserializer(deser) {}

    ReadOnlyStream(T* items, size_t count)
        : fileStream(nullptr), stringStream(nullptr),
          lazySequence(nullptr), arrayData(items), arraySize(count), currentPosition(0),
          isFileStream(false), isStringStream(false), isArrayStream(true),
          isLazyStream(false), opened(false), deserializer(nullptr) {}

    ReadOnlyStream(LazySequence<T>* lazy)    //
        : fileStream(nullptr), stringStream(nullptr),
          lazySequence(lazy), arrayData(nullptr), arraySize(0), currentPosition(0),
          isFileStream(false), isStringStream(false), isArrayStream(false),
          isLazyStream(true), opened(false), deserializer(nullptr) {}

    ReadOnlyStream()
        : fileStream(nullptr), stringStream(nullptr),
          lazySequence(nullptr), arrayData(nullptr), arraySize(0), currentPosition(0),
          isFileStream(false), isStringStream(false), isArrayStream(false),
          isLazyStream(false), opened(false), deserializer(nullptr) {}

    virtual ~ReadOnlyStream() {
        if (opened) Close();
        delete fileStream;
        delete stringStream;
    }

    virtual bool IsEndOfStream() const {
        if (!opened) return true;
        if (isLazyStream) {
            try {
                lazySequence->Get(currentPosition);
                return false;
            } catch (...) {
                return true;
            }
        }
        if (isArrayStream) return currentPosition >= arraySize;
        if (isFileStream) return fileStream->eof() || fileStream->peek() == EOF;
        if (isStringStream) return stringStream->eof();
        return true;
    }

    virtual T Read() {
        if (!opened) throw std::runtime_error("Stream not open");
        if (IsEndOfStream()) throw EndOfStreamException();
        T result;
        
        if (isLazyStream) {
            result = lazySequence->Get(currentPosition++);
        } else if (isArrayStream) {
            result = arrayData[currentPosition++];
        } else {
            std::string line;
            if (isFileStream) {
                while (std::getline(*fileStream, line) && line.empty()) {}
                if (line.empty() && fileStream->eof()) throw EndOfStreamException();
            } else {
                while (std::getline(*stringStream, line) && line.empty()) {}
                if (line.empty() && stringStream->eof()) throw EndOfStreamException();
            }
            result = deserializer->Deserialize(line);
            currentPosition++;
        }
        return result;
    }

    size_t GetPosition() const { return currentPosition; }
    bool IsCanSeek() const { return isArrayStream || isLazyStream; }
    size_t Seek(size_t index) {
        if (!IsCanSeek()) throw std::runtime_error("Seek not supported");
        
        if (isLazyStream) {
            currentPosition = index;
        } else if (isArrayStream) {
            currentPosition = index > arraySize ? arraySize : index;
        }
        return currentPosition;
    }

    bool IsCanGoBack() const { return isArrayStream || isLazyStream; }

    virtual void Open() {
        if (isFileStream && !fileStream->is_open()) throw std::runtime_error("Cannot open file");
        opened = true;
        currentPosition = 0;
        if (isFileStream) fileStream->clear();
        if (isStringStream) stringStream->clear();
    }

    virtual void Close() {
        if (isFileStream && fileStream->is_open()) fileStream->close();
        opened = false;
    }

    template<typename U>
    MappedReadOnlyStream<T, U>* Map(std::function<U(T)> mapper) {
        return new MappedReadOnlyStream<T, U>(this, mapper);
    }

    FilteredReadOnlyStream<T>* Where(std::function<bool(T)> predicate) {
        return new FilteredReadOnlyStream<T>(this, predicate);
    }

    template<typename U, typename Func>
    U Reduce(U initial, Func reducer) {
        U result = initial;
        while (!IsEndOfStream()) {
            try {
                result = reducer(result, Read());
            } catch (const EndOfStreamException&) { break; }
        }
        return result;
    }

    
    static ReadOnlyStream<T>* Of(LazySequence<T>* lazy) {
        return new ReadOnlyStream<T>(lazy);
    }

    void ForEach(std::function<void(const T&)> action) {
        this->Open();
        while (!this->IsEndOfStream()) {
            try {
                action(this->Read());
            } catch (const EndOfStreamException&) {
                break;
            }
        }
        this->Close();
    }

    size_t Count() {
        size_t count = 0;
        this->Open();
        while (!this->IsEndOfStream()) {
            try {
                this->Read();
                count++;
            } catch (const EndOfStreamException&) {
                break;
            }
        }
        this->Close();
        return count;
    }
};

template<typename T, typename U>
class MappedReadOnlyStream : public ReadOnlyStream<U> {
private:
    ReadOnlyStream<T>* sourceStream;
    std::function<U(T)> mapper;
public:
    MappedReadOnlyStream(ReadOnlyStream<T>* source, std::function<U(T)> mapFunc)
        : ReadOnlyStream<U>(), sourceStream(source), mapper(mapFunc) {
        this->opened = true;
    }
    bool IsEndOfStream() const override { return sourceStream->IsEndOfStream(); }
    U Read() override {
        if (!this->opened) throw std::runtime_error("Stream not open");
        if (IsEndOfStream()) throw EndOfStreamException();
        U val = mapper(sourceStream->Read());
        this->currentPosition++;
        return val;
    }
    void Open() override { 
        this->opened = true;
        sourceStream->Open();
    }
    void Close() override { 
        this->opened = false;
        sourceStream->Close(); 
    }
};

template<typename T>
class FilteredReadOnlyStream : public ReadOnlyStream<T> {
private:
    ReadOnlyStream<T>* sourceStream;
    std::function<bool(T)> predicate;
    T cachedItem;
    bool hasCachedItem = false;
    void findNext() {
        hasCachedItem = false;
        while (!sourceStream->IsEndOfStream()) {
            T v = sourceStream->Read();
            if (predicate(v)) { cachedItem = v; hasCachedItem = true; break; }
        }
    }
public:
    FilteredReadOnlyStream(ReadOnlyStream<T>* source, std::function<bool(T)> pred)
        : ReadOnlyStream<T>(), sourceStream(source), predicate(pred) {
        this->opened = true;
        findNext();
    }
    bool IsEndOfStream() const override { return !hasCachedItem; }
    T Read() override {
        if (!this->opened) throw std::runtime_error("Stream not open");
        if (!hasCachedItem) throw EndOfStreamException();
        T res = cachedItem;
        this->currentPosition++;
        findNext();
        return res;
    }
    void Open() override {
        this->opened = true;
        sourceStream->Open();
        if (!hasCachedItem) findNext();
    }
    void Close() override { this->opened = false; sourceStream->Close(); hasCachedItem = false; }
};

template<typename T>
class WriteOnlyStream {
private:
    std::ofstream* fileStream;
    T* arrayData;
    size_t arrayCapacity;
    size_t currentPosition;
    bool isFileStream;
    bool isArrayStream;
    bool opened;
    Deserializer<T>* deserializer;
public:
    WriteOnlyStream(const std::string& filename, Deserializer<T>* deser)
        : fileStream(new std::ofstream(filename, std::ios::out | std::ios::trunc)),
          arrayData(nullptr), arrayCapacity(0), currentPosition(0), isFileStream(true), isArrayStream(false), opened(false), deserializer(deser) {}
    WriteOnlyStream(T* buffer, size_t capacity)
        : fileStream(nullptr), arrayData(buffer), arrayCapacity(capacity), currentPosition(0), isFileStream(false), isArrayStream(true), opened(false), deserializer(nullptr) {}
    ~WriteOnlyStream() { if (opened) Close(); delete fileStream; }
    size_t GetPosition() const { return currentPosition; }
    size_t Write(T item) {
        if (!opened) throw std::runtime_error("Stream not open");
        if (isFileStream) (*fileStream) << deserializer->Serialize(item) << std::endl;
        else { if (currentPosition>=arrayCapacity) throw std::runtime_error("Buffer overflow"); arrayData[currentPosition]=item; }
        return ++currentPosition;
    }
    void Open() { if (isFileStream && !fileStream->is_open()) throw std::runtime_error("Cannot open file"); opened=true; if(isArrayStream) currentPosition=0; }
    void Close() { if(isFileStream&&fileStream->is_open()) fileStream->close(); opened=false; }
};

class IntDeserializer : public Deserializer<int> {
public:
    std::string Serialize(const int& item) const override { return std::to_string(item); }
    int Deserialize(const std::string& str) const override { if (str.empty()) throw std::runtime_error("Cannot deserialize empty string"); return std::stoi(str); }
};

class DoubleDeserializer : public Deserializer<double> {
public:
    std::string Serialize(const double& item) const override { return std::to_string(item); }
    double Deserialize(const std::string& str) const override { if (str.empty()) throw std::runtime_error("Cannot deserialize empty string"); return std::stod(str); }
};

class StringDeserializer : public Deserializer<std::string> {
public:
    std::string Serialize(const std::string& item) const override { return item; }
    std::string Deserialize(const std::string& str) const override { return str; }
};

class FileIntDeserializer : public Deserializer<int> {
public:
    std::string Serialize(const int& item) const override { return std::to_string(item); }
    int Deserialize(const std::string& str) const override { if(str.empty()) throw std::runtime_error("Cannot deserialize empty string"); return std::stoi(str); }
};

#endif