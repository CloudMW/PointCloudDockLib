//
// Created by Mengfanyong on 2025/12/17.
//

#ifndef VISIONUTILS_COMMON_UTILS_IO_HPP
#define VISIONUTILS_COMMON_UTILS_IO_HPP

#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/nvp.hpp>
#include <fstream>
#include <spdlog/spdlog.h>

namespace common_utils
{
    namespace io
    {
        template <typename StCommon>
        int
        ReadXml
        (const char* fileName, StCommon& stRcp)
        {
            std::ifstream out(fileName);
            if (!out.good())
            {
                SPDLOG_ERROR("Failed to open the Xml!"); //异常跳出
                return 1;
            }
            boost::archive::xml_iarchive oa(out);
            oa >> BOOST_SERIALIZATION_NVP(stRcp);
            //out.close();

            //
            return 0;
        }
    }
}


#endif //VISIONUTILS_COMMON_UTILS_IO_HPP
