/**
 * @copyright 2018 Infineon Technologies
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>


namespace strata
{

    template <typename DataType>
    class DataConsumer
    {
    public:
        template <typename SizeType>
        DataConsumer(const DataType *data, SizeType size) :
            m_data {data},
            m_end {data + size}
        {
        }

        DataConsumer(const DataType *begin, const DataType *end) :
            m_data {begin},
            m_end {end}
        {
        }

        template <typename T>
        void consume(T *begin, T *end)
        {
            static_assert(sizeof(T) % sizeof(DataType) == 0, "Cannot consume type that is not a multiple of the underlying buffer");

            auto *dest_begin = reinterpret_cast<DataType *>(begin);
            auto *dest_end   = reinterpret_cast<DataType *>(end);

            const auto *updated_data = m_data + (dest_end - dest_begin);
            if (updated_data > m_end)
            {
                throw std::length_error("consuming more than remaining in underlying buffer (possibly check struct packing)");
            }
            std::copy(m_data, updated_data, dest_begin);
            m_data = updated_data;
        }

        template <typename T, typename SizeType = std::size_t>
        inline void consume(T *data, SizeType size)
        {
            consume(data, data + size);
        }

        template <typename T>
        inline void consume(T *value)
        {
            consume(value, value + 1);
        }

        template <typename T>
        inline void consume(T &value)
        {
            consume(&value);
        }

        bool finished() const
        {
            return (m_data == m_end);
        }

    private:
        const DataType *m_data;
        const DataType *m_end;
    };

}
