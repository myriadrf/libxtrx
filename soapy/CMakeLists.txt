set(CMAKE_CXX_STANDARD 11)
project(soapyxtrx C CXX)

find_package(SoapySDR)

add_executable(test_xtrx_soapy test_xtrx_soapy.c)
target_link_libraries(test_xtrx_soapy ${SoapySDR_LIBRARIES})

SOAPY_SDR_MODULE_UTIL(
    TARGET XTRXSupport
    SOURCES
        Registration.cpp
        SoapyXTRX.cpp
    LIBRARIES
        xtrx
)

