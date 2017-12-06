set(CMAKE_CXX_STANDARD 11)
project(soapyxtx C CXX)

find_package(SoapySDR CONFIG)

SOAPY_SDR_MODULE_UTIL(
    TARGET XTRXSupport
    SOURCES
        Registration.cpp
        SoapyXTRX.cpp
    LIBRARIES
        xtrx
)
