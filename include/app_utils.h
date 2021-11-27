/**
 * Help application utility functions/classes.
 */
#ifndef APP_UTILS_H
#define APP_UTILS_H

#include <utility>
#include <type_traits>
#include <iterator>

#include <vector>

/**
 * Simple linked list.
 */
template<typename T>
class SimpleList {
private:
    struct item_t {
        T value;
        item_t *next = nullptr;

        template<typename ...Args>
        explicit item_t(Args &&...args) : value(std::forward<Args>(args)...)
        {
        }
    };

    item_t *_items = nullptr;

    void _append(item_t *elem)
    {
        item_t **last_item = &_items;
        while (*last_item != nullptr) {
            last_item = &(*last_item)->next;
        }
        *last_item = elem;
    }

public:
    SimpleList()
    {
    }

    ~SimpleList()
    {
        clear();
    }

    template<typename... Args>
    T *create_and_append(Args &&... args)
    {
        auto elem = new item_t(std::forward<Args>(args)...);
        _append(elem);
        return &(elem->value);
    }

    void clear()
    {
        item_t *item = _items;
        while (item != nullptr) {
            item_t *next_item = item->next;
            delete item;
            item = next_item;
        }
        _items = nullptr;
    }

    class Iterator {
        friend SimpleList;
        item_t *_item;

        explicit Iterator(item_t *item) : _item(item)
        {}

    public:
        using iterator_category = std::input_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = T;
        using pointer = T *;
        using reference = T &;

        reference operator*() const
        { return _item->value; }

        pointer operator->()
        { return &(_item->value); }

        Iterator &operator++()
        {
            _item = _item->next;
            return *this;
        }

        Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const Iterator &other) const
        { return _item == other._item; };

        bool operator!=(const Iterator &other) const
        { return _item != other._item; };
    };

    Iterator begin()
    { return Iterator(_items); }

    Iterator end()
    { return Iterator(nullptr); }
};

/**
 * Simple logger.
 */
class SimpleLogger {
private:
    SimpleLogger(const SimpleLogger &) = delete;

    SimpleLogger &operator=(const SimpleLogger &) = delete;

    bool _enabled = true;
public:
    SimpleLogger() = default;

    ~SimpleLogger() = default;

    void set_enabled(bool value);

    bool is_enabled() const;

    void log(const char *level_name, const char *msg, ...);

    void vlog(const char *level_name, const char *msg, va_list args);

    void info(const char *msg, ...);

    void error(const char *msg, ...);
};


#endif //APP_UTILS_H
