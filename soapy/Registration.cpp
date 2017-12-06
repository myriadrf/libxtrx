#include "SoapyXTRX.h"
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

static SoapySDR::KwargsList findIConnection(const SoapySDR::Kwargs &matchArgs)
{
    SoapySDR::KwargsList results;

    SoapySDR::Kwargs xtrxArgs;
    xtrxArgs["type"] = "fairwaves";
	xtrxArgs["dev"] = "/dev/xtrx0";
    results.push_back(xtrxArgs);

    return results;
}

static SoapySDR::Device *makeIConnection(const SoapySDR::Kwargs &args)
{
    return new SoapyXTRX(args);
}

static SoapySDR::Registry registerIConnection("xtrx", &findIConnection, &makeIConnection, SOAPY_SDR_ABI_VERSION);


