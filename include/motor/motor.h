#pragma once
#include "serial/serial.h"
#include <string>
#include <vector>
#include <functional>

namespace Motor
{
    namespace CMD
    {
        class Command : public std::vector<uint8_t>
        {
        public:
            Command(size_t s = 0, bool big_endian = false)
            : vector<uint8_t>(s)
            , need_big_endian(big_endian)
            {
                union
                {
                    uint32_t i;
                    char c[4];
                } bint = {0x01020304};

                is_big_endian = bint.c[0] == 1;
            }

            template <typename T>
            Command& append(T data)
            {
                static auto reverse_endian = [&](T u) -> T
                {
                    union
                    {
                        T u;
                        unsigned char u8[sizeof(T)];
                    } source, dest;

                    source.u = u;

                    for (size_t k = 0; k < sizeof(T); k++)
                        dest.u8[k] = source.u8[sizeof(T) - k - 1];

                    return dest.u;
                };

                if(is_big_endian != need_big_endian)
                    data = reverse_endian(data);
                
                auto ptr = reinterpret_cast<uint8_t*>(&data);
                for (size_t i = 0; i < sizeof(T); i++)
                {
                    push_back(*ptr);
                    ptr++;
                }
                
                return *this;
            }

        public : 
            std::function<Command&(uint8_t)> init;
            std::function<Command&()> build;

        private:
            bool is_big_endian;
            bool need_big_endian;
        };
    }
    
    
    class Motor
    {
    public:
        Motor() {};
        virtual ~Motor() {};

        // speed (degree / s)
        virtual bool rotate(double speed) = 0;
        // run with last speed user set
        virtual bool rotate() = 0;

        virtual bool pause() = 0;

        // degree: 0 ~ 360
        // speed: -2000 ~ 2000 (degree / s)
        virtual bool rotateTo(double degree, double speed) = 0;
        // degree: 0 ~ 360
        virtual bool rotateTo(double degree) = 0;

        // degree: 0 ~ 360
        // speed: -2000 ~ 2000 (degree / s)
        virtual bool rotateMore(double degree, double speed) = 0;
        // degree: 0 ~ 360
        virtual bool rotateMore(double degree) = 0;
    
    protected:
        Motor(double speed, size_t rev_size, size_t snd_size)
        : m_speed(speed)
        , m_rev(rev_size)
        , m_snd(snd_size)
        , is_busy(false)
        {}

    protected:
        double m_speed;
        bool is_busy;
        std::vector<uint8_t> m_rev;
        CMD::Command m_snd;
    };
}