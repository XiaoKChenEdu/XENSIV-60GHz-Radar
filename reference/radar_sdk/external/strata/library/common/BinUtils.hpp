#pragma once

#include <cstdint>
#include <stdexcept>
#include <type_traits>


/**
 * Swaps two bits within a value
 * @param n value with bits to swap
 * @param a first bit position
 * @param b second bit position
 * @return value with swapped bits
 */
template <typename T>
inline T swapBits(T n, unsigned int a, unsigned int b)
{
    // if both bits are different, then toggle both, else just return same value
    return (((n >> a) ^ (n >> b)) & 1) ? (n ^ (1u << a) ^ (1u << b)) : n;
}

template <typename T, class = typename std::enable_if<std::is_unsigned<T>::value>::type>
inline T getBitmask(unsigned width, unsigned offset = 0u)
{
    constexpr auto bit_size = sizeof(T) * 8;
    if (width + offset > bit_size)
    {
        throw std::range_error("width + offset exceed type width");
    }
    return (bit_size == width) ? static_cast<T>(-1)
                               : ((static_cast<T>(1) << width) - 1) << offset;
}

template <unsigned int bits, unsigned int shift = 0, typename T>
inline T maskBits(T value)
{
    return ((1u << bits) - 1) & value;
}

template <typename T2 = uint8_t, typename T1>
inline T2 getBitCount(T1 value)
{
    T2 count = 0;
    while (value)
    {
        value &= (value - 1);
        count++;
    }
    return count;
}

template <typename T>
inline T reverseBits(T value)
{
    static_assert(std::is_unsigned<T>::value, "Value needs to be an unsigned number to avoid sign extension.");

    T result = 0u;
    T lower  = 1u;
    T upper  = (1u << (sizeof(value) * 8 - 1));
    for (; lower < upper; lower <<= 1u, upper >>= 1u)
    {
        result |= (lower & value) ? upper : 0;
        result |= (upper & value) ? lower : 0;
    }

    return result;
}


#include <vector>

template <typename T>
void reshape(T array[], std::size_t rows, std::size_t cols)
{
    // first and last element don't have to be moved
    const auto size = rows * cols - 1;
    std::vector<bool> b(size, false);  // hash to mark moved elements
    b[0] = true;

    std::size_t i = 1;
    while (i < size)
    {
        auto cycleBegin = i;         // holds start of cycle
        T t             = array[i];  // holds element to be replaced, eventually becomes next element to move
        do
        {
            // Input matrix [r x c]
            // Output matrix
            // i_new = (i*r)%size
            const auto next = (i * rows) % size;  // location of 't' to be moved
            std::swap(array[next], t);
            b[i] = true;
            i    = next;
        } while (i != cycleBegin);

        // Get Next Move (what about querying random location?)
        for (i = 1; i < size && b[i]; i++)
            ;
    }
}

template <typename T>
void reshape(const T in[], T out[], std::size_t rows, std::size_t cols)
{
    std::size_t cnt = 0;
    for (std::size_t row = 0; row < rows; row++)
    {
        for (std::size_t col = 0; col < cols; col++)
        {
            out[cnt++] = in[col + cols * row];
        }
    }
}
