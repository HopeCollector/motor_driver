#pragma once
#include "../motor.h"
#include <iostream>

#ifdef CXX17
#include <filesystem>
#include <regex>
#endif

namespace Motor
{
    namespace CMD
    {
        namespace RMDS
        {
            static const size_t IDX_FRAME_HEAD = 0;
            static const size_t IDX_CMD = 1;
            static const size_t IDX_ID = 2;
            static const size_t IDX_DATA_LEN = 3;
            static const size_t IDX_HEAD_CHK = 4;
            static const size_t IDX_DATA_START = 5;
            static const size_t FRAME_LEN = 5;
            static const uint8_t FRAME_HEAD = 0x3E;

            static const double MAX_SPEED = 20000.0;

            // Control
            static const uint8_t MOTOR_START = 0x88;
            static const uint8_t MOTOR_PAUSE = 0x81;
            static const uint8_t MOTOR_SHUTDOWN = 0x80;

            // R: Read, W: Write
            // [R/W]_[Target]_[Dst]
            // PID
            static const uint8_t R_PID = 0x30;
            static const uint8_t W_PID_RAM = 0x31;
            static const uint8_t W_PID_ROM = 0x32;

            // ACC
            static const uint8_t R_ACC = 0x33;
            static const uint8_t W_ACC = 0x34;

            // Encoder
            static const uint8_t R_ENCODER = 0x90;
            static const uint8_t W_ENCODER_ROM = 0x91;
            static const uint8_t W_CURPOS_ROM = 0x19;

            // Angle
            static const uint8_t R_MULTILOOPANG = 0x92;  // 当前电机的多圈绝对值角度
            static const uint8_t R_SINGALLOOPANG = 0x94; // 当前电机的单圈绝对值角度
            static const uint8_t W_DEFANG = 0x95;

            // Error & State
            static const uint8_t R_ERROR = 0x9A;
            static const uint8_t W_ERROR = 0x9B;
            static const uint8_t R_STATE1 = 0x9A;
            static const uint8_t R_STATE2 = 0x9C;
            static const uint8_t R_STATE3 = 0x9D;

            // Close Loop Control
            // 输出指定功率
            static const uint8_t W_POWER = 0xA0;
            // 输出指定速度
            static const uint8_t W_SPEED = 0xA2;

            // Angle Control (1：默认速度，2：指定速度)
            // 旋转到特定角度，这里有考虑从上电开始电机全部旋转角度
            // 比如电机上电后旋转四圈后使用下面指令旋转到 0°，会让电机往回转四圈
            // -360 ~ 360
            static const uint8_t W_MLANG1 = 0xA3;
            static const uint8_t W_MLANG2 = 0xA4;
            // 旋转到特定角度，不考虑电机已经旋转了多少圈
            // 比如电机上电后旋转了四圈多 10°
            // 在不指定倒转的情况下会旋转 350° 变成第五圈的 0 °
            // 0 ~ 360 (允许倒转)
            static const uint8_t W_SLANG1 = 0xA5;
            static const uint8_t W_SLANG2 = 0xA6;
            // 从当前位置开始再选转指定角度
            // -360 ~ 360
            static const uint8_t W_ADDANG1 = 0xA7; // 默认速度旋转指定角度 0 ~ 360
            static const uint8_t W_ADDANG2 = 0xA8; // 指定速度旋转指定角度 0 ~ 360

            static const uint8_t R_INFO = 0x12;
        }
    }

    class RMDS : public Motor
    {
    public:
        RMDS() = delete;
        RMDS(std::string dev, uint8_t id);
        virtual ~RMDS();

        // speed (degree / s)
        // 如果 speed = -1, 电机会按照上一次速度 m_speed 运行
        // m_speed 不会是 0
        virtual bool rotate(double speed) override;
        virtual bool rotate() override
        {
            return rotate(m_speed);
        }

        virtual bool pause() override
        {
            m_snd.init(CMD::RMDS::MOTOR_PAUSE)
                .build();
            m_sender.write(m_snd);
            m_rev.clear();
            auto rev = m_sender.read(m_rev, 5);
            return rev == 5;
        }

        // degree: 0 ~ 360
        // speed: -2000 ~ 2000 (degree / s)
        virtual bool rotateTo(double degree, double speed) override;
        // degree: 0 ~ 360
        virtual bool rotateTo(double degree) override
        {
            return rotateTo(degree, m_speed);
        };

        // degree: -360 ~ 360
        // speed: -2000 ~ 2000 (degree / s)
        virtual bool rotateMore(double degree, double speed) override;
        // degree: -360 ~ 360
        virtual bool rotateMore(double degree) override
        {
            return rotateMore(degree, m_speed);
        };

        double getCurrentPose();

        // 这个系列电机独有的多圈工作模式, 不过其记忆的位置都是上电之后的, 这个需要注意
        bool rotateMTo(double degree, double speed);
        bool rotateMTo(double degree)
        {
            return rotateMTo(degree, m_speed);
        }

        bool rotateMMore(double degree, double speed);
        bool rotateMMore(double degree)
        {
            return rotateMMore(degree, m_speed);
        }

        double getCurrentMPose();

        #ifdef CXX17
        static std::string findDev(uint8_t id, bool is_log = false);
        #endif

    private:
        bool send();

        inline void updateSpeed(double &speed)
        {
            if (speed != 0)
                m_speed = speed;
        }

    private:
        uint8_t m_id;
        serial::Serial m_sender;
    };

    RMDS::RMDS(std::string dev, uint8_t id)
        : m_sender(dev, 115200, serial::Timeout::simpleTimeout(50)), m_id(id), Motor(180.0, 100, 100)
    {
        m_snd.init = [&](uint8_t cmd) -> CMD::Command &
        {
            m_snd.resize(CMD::RMDS::FRAME_LEN);
            m_snd[CMD::RMDS::IDX_FRAME_HEAD] = CMD::RMDS::FRAME_HEAD;
            m_snd[CMD::RMDS::IDX_CMD] = cmd;
            m_snd[CMD::RMDS::IDX_ID] = m_id;
            m_snd[CMD::RMDS::IDX_HEAD_CHK] = 0;
            return m_snd;
        };

        m_snd.build = [&]() -> CMD::Command &
        {
            if (m_snd.size() > CMD::RMDS::FRAME_LEN)
            {
                m_snd.push_back(0);
                m_snd[CMD::RMDS::IDX_DATA_LEN] = m_snd.size() - CMD::RMDS::FRAME_LEN - 1;
                for (size_t i = 0; i < m_snd[CMD::RMDS::IDX_DATA_LEN]; i++)
                {
                    m_snd.back() += m_snd[CMD::RMDS::IDX_DATA_START + i];
                }
            }
            else
                m_snd[CMD::RMDS::IDX_DATA_LEN] = 0;

            for (size_t i = 0; i < CMD::RMDS::IDX_HEAD_CHK; i++)
            {
                m_snd[CMD::RMDS::IDX_HEAD_CHK] += m_snd[i];
            }
            return m_snd;
        };

        if (!m_sender.isOpen())
        {
            try
            {
                m_sender.open();
            }
            catch (serial::IOException &e)
            {
                std::cerr << "Unable to open port " << std::endl;
                exit(-1);
            }
        }

        // find motor
        {
        }
    }

    RMDS::~RMDS()
    {
        if (m_sender.isOpen())
        {
            m_sender.close();
        }
    }

    bool RMDS::rotate(double speed)
    {
        if (is_busy || speed > CMD::RMDS::MAX_SPEED)
            return false;

        is_busy = true;

        const int rev_len = 13;
        updateSpeed(speed);
        int32_t _speed = static_cast<int32_t>(speed * 100);

        m_snd.init(CMD::RMDS::W_SPEED)
            .append(_speed)
            .build();

        m_sender.write(m_snd);

        m_rev.clear();
        auto ret = m_sender.read(m_rev, rev_len);
        is_busy = false;
        return ret == rev_len;
    }

    bool RMDS::rotateTo(double degree, double speed)
    {
        if (is_busy || speed > CMD::RMDS::MAX_SPEED || speed == 0 || degree < 0 || degree > 359.99)
            return false;

        is_busy = true;

        const int rev_len = 13;
        updateSpeed(speed);
        uint32_t _speed = static_cast<uint32_t>(std::abs(speed) * 100);
        uint16_t _degree = static_cast<uint16_t>(degree * 100);
        uint8_t direction = speed < 0;

        m_snd.init(CMD::RMDS::W_SLANG2)
            .append(direction)
            .append(_degree)
            .append(static_cast<uint8_t>(0))
            .append(_speed)
            .build();

        m_sender.write(m_snd);
        m_rev.clear();
        auto ret = m_sender.read(m_rev, rev_len);
        is_busy = false;
        return ret == rev_len;
    }

    bool RMDS::rotateMore(double degree, double speed)
    {
        if (is_busy || speed > CMD::RMDS::MAX_SPEED)
            return false;

        is_busy = true;

        const int rev_len = 13;
        updateSpeed(speed);
        int32_t _degree = static_cast<int32_t>(degree * 100);
        uint32_t _speed = static_cast<uint32_t>(speed * 100);

        m_snd.init(CMD::RMDS::W_ADDANG2)
            .append(_degree)
            .append(_speed)
            .build();

        m_rev.clear();
        m_sender.write(m_snd);
        auto ret = m_sender.read(m_rev, rev_len);
        is_busy = false;
        return ret == rev_len;
    }

    double RMDS::getCurrentPose()
    {
        if (is_busy)
            return 400.0;
        is_busy = true;

        const int rev_len = 8;
        double angle = 600.0;

        if (m_snd[CMD::RMDS::IDX_CMD] != CMD::RMDS::R_SINGALLOOPANG)
        {
            m_snd.init(CMD::RMDS::R_SINGALLOOPANG)
                .build();
        }

        auto wt_len = m_sender.write(m_snd);
        if(wt_len != m_snd.size())
        {
            std::cerr << "ERROR: Only write " <<  wt_len << " bytes" << std::endl;
        }

        m_rev.clear();
        auto ret = m_sender.read(m_rev, rev_len);
        if (ret == rev_len)
        {
            auto tmp = reinterpret_cast<uint16_t *>(&m_rev[CMD::RMDS::IDX_DATA_START]);
            angle = (static_cast<double>(*tmp)) / 100.0;
        }
        else
        {
            std::cerr << "ERROR: Only read " << ret << " bytes" << std::endl;
        }
        is_busy = false;
        return angle;
    }

    bool RMDS::rotateMTo(double degree, double speed)
    {
        if (is_busy || speed > CMD::RMDS::MAX_SPEED || speed == 0)
            return false;

        is_busy = true;

        const int rev_len = 13;
        updateSpeed(speed);
        uint32_t _speed = static_cast<uint32_t>(std::abs(speed) * 100);
        int64_t _degree = static_cast<int64_t>(degree * 100);

        m_snd.init(CMD::RMDS::W_MLANG2)
            .append(_degree)
            .append(_speed)
            .build();

        m_rev.clear();
        m_sender.write(m_snd);
        auto ret = m_sender.read(m_rev, rev_len);
        is_busy = false;
        return ret == rev_len;
    }

    bool RMDS::rotateMMore(double degree, double speed)
    {
       return rotateMore(degree, speed);
    }

    double RMDS::getCurrentMPose()
    {
        if (is_busy)
            return -1.0;
        is_busy = true;

        const int rev_len = 14;
        double angle = -2.0;

        if (m_snd[CMD::RMDS::IDX_CMD] != CMD::RMDS::R_MULTILOOPANG)
        {
            m_snd.init(CMD::RMDS::R_MULTILOOPANG)
                .build();
        }

        m_rev.clear();
        auto wt_len = m_sender.write(m_snd);
        if (wt_len != m_snd.size())
        {
            std::cerr << "ERROR: Only write " << wt_len << " bytes" << std::endl;
        }

        auto ret = m_sender.read(m_rev, rev_len);
        if (ret == rev_len)
        {
            auto tmp = reinterpret_cast<int64_t *>(&m_rev[CMD::RMDS::IDX_DATA_START]);
            angle = (static_cast<double>(*tmp)) / 100.0;
        }
        else
        {
            std::cerr << "ERROR: Only read " << ret << " bytes" << std::endl;
        }
        is_busy = false;
        return angle;
    }

    #ifdef CXX17
    std::string RMDS::findDev(uint8_t id, bool is_log)
    {
        namespace fs = std::filesystem;

        std::string ret = "";
        auto check_perms = [](const fs::perms &&perms) -> bool
        {
            using p = fs::perms;
            return (
                (perms & p::owner_read) != p::none &&
                (perms & p::owner_write) != p::none &&
                (perms & p::group_read) != p::none &&
                (perms & p::group_write) != p::none &&
                (perms & p::others_read) != p::none &&
                (perms & p::others_write) != p::none);
        };
        std::regex regex("/dev/ttyUSB[0-9]*");
        serial::Serial ser("", 115200, serial::Timeout::simpleTimeout(1000));
        std::array<uint8_t, 5> cmd{0x3e, 0x12, id, 0x00, 0x50};
        std::array<uint8_t, 48> rec;
        cmd[4] += id;

        for (auto &entry : fs::directory_iterator("/dev/"))
        {
            auto &&dev = entry.path().string();

            if (!std::regex_match(dev, regex) ||
                !check_perms(fs::status(dev).permissions()))
                continue;

            size_t read = 0;
            try
            {
                ser.setPort(dev);
                ser.open();
                ser.write(cmd.data(), 5);
                read = ser.read(rec.data(), 48);
            }
            catch (const std::exception &e)
            {
                continue;
            }

            if (read != 48 ||
                rec[4] != (0x50 + id + 0x2a))
                continue;

            if(is_log)
            {
                char *info = reinterpret_cast<char *>(&rec[5]);
                std::string dev_driver_name(info, 20);
                info += 20;
                std::string dev_motor_name(info, 20);
                info += 20;
                uint8_t dev_hardware_ver = *info;
                info++;
                uint8_t dev_firmware_ver = *info;
                std::cout
                    << "Driver Name:\t" << dev_driver_name << "\n"
                    << "Motor Name:\t" << dev_motor_name << "\n"
                    << "Hardware Version:\t" << +dev_hardware_ver << "\n"
                    << "Fireware Version:\t" << +dev_hardware_ver
                    << std::endl;
            }
            ret = dev;
            ser.close();
            break;
        }

        return ret;
    }
    #endif
}