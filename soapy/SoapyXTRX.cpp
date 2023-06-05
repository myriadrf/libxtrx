#include "SoapyXTRX.h"
#include <stdexcept>
#include <iostream>
#include <memory>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include <SoapySDR/Formats.hpp>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <string.h>


void SoapyXTRX::xtrx_logfunc(int sevirity, const char* message)
{
	// TODO parse sevirity
	(void)sevirity;

	SoapySDR::log(SOAPY_SDR_INFO, message);
}

std::map<std::string, std::weak_ptr<XTRXHandle>> XTRXHandle::s_created;

std::shared_ptr<XTRXHandle> XTRXHandle::get(const std::string& name)
{
	auto idx = s_created.find(name);
	if (idx != s_created.end()) {
		 if (std::shared_ptr<XTRXHandle> obj = idx->second.lock())
			 return obj;
	}

	std::shared_ptr<XTRXHandle> obj = std::make_shared<XTRXHandle>(name);
	s_created.insert(make_pair(name, obj));
	return obj;
}

XTRXHandle::XTRXHandle(const std::string& name)
{
	int res = xtrx_open_string(name.c_str(), &_dev);
	if (res < 0)
		throw std::runtime_error(std::string("XTRXHandle::XTRXHandle(")+name.c_str()+") - unable to open the device: error: " + strerror(-res));
	devcnt = res;

	SoapySDR::log(SOAPY_SDR_INFO, std::string("Created: `") + name.c_str() + "`");
}

XTRXHandle::~XTRXHandle()
{
	xtrx_close(_dev);
}

SoapyXTRX::SoapyXTRX(const SoapySDR::Kwargs &args)
{
	SoapySDR::logf(SOAPY_SDR_INFO, "Make connection: '%s'", args.count("dev") ? args.at("dev").c_str() : "*");

	unsigned loglevel = 3;
#ifdef __linux
	const char* lenv = getenv("SOAPY_XTRX_LOGLEVEL");
	if (lenv) {
		loglevel = atoi(lenv);
	}
#endif
	const std::string& dev = (args.count("dev")) ? args.at("dev") : "";

	if (args.count("loglevel")) {
		loglevel = std::stoi(args.at("loglevel"));
	}
	xtrx_log_setlevel(loglevel, NULL);

	_dev = XTRXHandle::get(dev);

	if (args.count("refclk")) {
		xtrx_set_ref_clk(_dev->dev(), std::stoi(args.at("refclk")), XTRX_CLKSRC_INT);

		SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::SoapyXTRX() set ref to internal clock");
	}
	if (args.count("extclk")) {
		xtrx_set_ref_clk(_dev->dev(), std::stoi(args.at("extclk")), XTRX_CLKSRC_EXT);

		SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::SoapyXTRX() set ref to external clock");
	}
}

SoapyXTRX::~SoapyXTRX(void)
{
}

/*******************************************************************
 * Identification API
 ******************************************************************/
std::string SoapyXTRX::getDriverKey(void) const
{
	return "xtrxsoapy";
}

std::string SoapyXTRX::getHardwareKey(void) const
{
	return "xtrxdev";
}

SoapySDR::Kwargs SoapyXTRX::getHardwareInfo(void) const
{
	SoapySDR::Kwargs info;
	return info;
}

/*******************************************************************
 * Channels API
 ******************************************************************/
size_t SoapyXTRX::getNumChannels(const int /*direction*/) const
{
	return 2;
}

bool SoapyXTRX::getFullDuplex(const int /*direction*/, const size_t /*channel*/) const
{
	return true;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/
std::vector<std::string> SoapyXTRX::listAntennas(const int direction, const size_t /*channel*/) const
{
	std::vector<std::string> ants;
	if (direction == SOAPY_SDR_RX)
	{
		ants.push_back("LNAH");
		ants.push_back("LNAL");
		ants.push_back("LNAW");
	}
	if (direction == SOAPY_SDR_TX)
	{
		ants.push_back("TXH");
		ants.push_back("TXW");
	}
	return ants;
}

void SoapyXTRX::setAntenna(const int direction, const size_t channel, const std::string &name)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::setAntenna(%d, %s)", int(channel), name.c_str());
	xtrx_antenna_t a;

	if (direction == SOAPY_SDR_RX)
	{
		if (name == "LNAH" || name == "RXH") a = XTRX_RX_H;
		else if (name == "LNAL" || name == "RXL") a = XTRX_RX_L;
		else if (name == "LNAW" || name == "RXW") a = XTRX_RX_W;
		else if (name == "AUTO") a = XTRX_RX_AUTO;
		else throw std::runtime_error("SoapyXTRX::setAntenna(RX, "+name+") - unknown antenna name");

		_rx_ant = a;
	}
	else if (direction == SOAPY_SDR_TX)
	{
		if (name == "BAND1" || name == "B1" || name == "TXH") a = XTRX_TX_H;
		else if (name == "BAND2" || name == "B2" || name == "TXW") a = XTRX_TX_W;
		else if (name == "AUTO") a = XTRX_TX_AUTO;
		else throw std::runtime_error("SoapyXTRX::setAntenna(TX, "+name+") - unknown antenna name");

		_tx_ant = a;
	} else {
		throw std::runtime_error("SoapyXTRX::setAntenna(?)");
	}

	int res = xtrx_set_antenna(_dev->dev(), a);
	if (res) {
		throw std::runtime_error("SoapyXTRX::setAntenna(TX, "+name+") xtrx_set_antenna() err");
	}
}

std::string SoapyXTRX::getAntenna(const int direction, const size_t /*channel*/) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	if (direction == SOAPY_SDR_RX)
	{
		switch (_rx_ant)
		{
		case XTRX_RX_H: return "LNAH";
		case XTRX_RX_L: return "LNAL";
		case XTRX_RX_W: return "LNAW";
		default: return "NONE";
		}
	}

	if (direction == SOAPY_SDR_TX)
	{
		switch (_tx_ant)
		{
		case XTRX_TX_H: return "TXH";
		case XTRX_TX_W: return "TXW";
		default: return "NONE";
		}
	}

	return "";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyXTRX::hasDCOffsetMode(const int direction, const size_t /*channel*/) const
{
	return (direction == SOAPY_SDR_RX);
}

void SoapyXTRX::setDCOffsetMode(const int direction, const size_t /*channel*/, const bool /*automatic*/)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	if (direction == SOAPY_SDR_RX) {
		//rfic->SetRxDCRemoval(automatic);
	}
}

bool SoapyXTRX::getDCOffsetMode(const int direction, const size_t /*channel*/) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	if (direction == SOAPY_SDR_RX) {
		// return rfic->GetRxDCRemoval();
	}

	return false;
}

bool SoapyXTRX::hasDCOffset(const int direction, const size_t /*channel*/) const
{
	return (direction == SOAPY_SDR_TX);
}

void SoapyXTRX::setDCOffset(const int direction, const size_t /*channel*/, const std::complex<double> &/*offset*/)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	if (direction == SOAPY_SDR_TX) {
		//rfic->SetTxDCOffset(offset.real(), offset.imag());
	}
}

std::complex<double> SoapyXTRX::getDCOffset(const int /*direction*/, const size_t /*channel*/) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	double I = 0.0, Q = 0.0;
	//if (direction == SOAPY_SDR_TX) rfic->GetTxDCOffset(I, Q);
	return std::complex<double>(I, Q);
}

bool SoapyXTRX::hasIQBalance(const int /*direction*/, const size_t /*channel*/) const
{
	return true;
}

void SoapyXTRX::setIQBalance(const int /*direction*/, const size_t /*channel*/, const std::complex<double> &/*balance*/)
{
	//TODO
}

std::complex<double> SoapyXTRX::getIQBalance(const int /*direction*/, const size_t /*channel*/) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	return std::complex<double>(0,0);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyXTRX::listGains(const int direction, const size_t /*channel*/) const
{
	std::vector<std::string> gains;
	if (direction == SOAPY_SDR_RX)
	{
		gains.push_back("LNA");
		gains.push_back("TIA");
		gains.push_back("PGA");
	}
	else if (direction == SOAPY_SDR_TX)
	{
		gains.push_back("PAD");
	}
	return gains;
}

void SoapyXTRX::setGain(const int direction, const size_t channel, const double value)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	xtrx_channel_t chan = to_xtrx_channels(channel);
	SoapySDR::logf(SOAPY_SDR_FATAL /*SOAPY_SDR_DEBUG(*/, "SoapyXTRX::setGain(, %d, --, %g dB)", int(channel), value);

	if (direction == SOAPY_SDR_RX)
	{
		double actual;
		xtrx_set_gain(_dev->dev(), chan, XTRX_RX_LNA_GAIN, value, &actual);
	}

	else SoapySDR::Device::setGain(direction, channel, value);
}

void SoapyXTRX::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	SoapySDR::logf(SOAPY_SDR_FATAL /*SOAPY_SDR_DEBUG(*/, "SoapyXTRX::setGain(, %d, %s, %g dB)", int(channel), name.c_str(), value);

	xtrx_channel_t chan = to_xtrx_channels(channel);
    if (direction == SOAPY_SDR_RX and (name == "LNA" || name == "LB"))
	{
		xtrx_set_gain(_dev->dev(), chan, XTRX_RX_LNA_GAIN, value, &_actual_rx_gain_lna[channel]);
		return;
	}

	else if (direction == SOAPY_SDR_RX and name == "TIA")
	{
		xtrx_set_gain(_dev->dev(), chan, XTRX_RX_TIA_GAIN, value, &_actual_rx_gain_tia[channel]);
		return;
	}

	else if (direction == SOAPY_SDR_RX and name == "PGA")
	{
		xtrx_set_gain(_dev->dev(), chan, XTRX_RX_PGA_GAIN, value, &_actual_rx_gain_pga[channel]);
		return;
	}

	else if (direction == SOAPY_SDR_TX and name == "PAD")
	{
		xtrx_set_gain(_dev->dev(), chan, XTRX_TX_PAD_GAIN, value, &_actual_tx_gain_pad[channel]);
	}

	else throw std::runtime_error("SoapyXTRX::setGain("+name+") - unknown gain name");

	SoapySDR::logf(SOAPY_SDR_DEBUG, "Actual %s[%d] gain %g dB", name.c_str(), int(channel), this->getGain(direction, channel, name));
}

double SoapyXTRX::getGain(const int direction, const size_t channel, const std::string &name) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	to_xtrx_channels(channel);

    if (direction == SOAPY_SDR_RX and (name == "LNA" || name == "LB"))
	{
		return _actual_rx_gain_lna[channel];
	}

	else if (direction == SOAPY_SDR_RX and name == "TIA")
	{
		return _actual_rx_gain_tia[channel];
	}

	else if (direction == SOAPY_SDR_RX and name == "PGA")
	{
		return _actual_rx_gain_pga[channel];
	}

	else if (direction == SOAPY_SDR_TX and name == "PAD")
	{
		return _actual_tx_gain_pad[channel];
	}

	else throw std::runtime_error("SoapyXTRX::getGain("+name+") - unknown gain name");
}

SoapySDR::Range SoapyXTRX::getGainRange(const int direction, const size_t channel) const
{
	if (direction == SOAPY_SDR_RX)
	{
		//make it so gain of 0.0 sets PGA at its mid-range
		//return SoapySDR::Range(-12.0, 19.0+12.0+30.0); //-12.0 causes bad calculation of SoapySDR::getGain
		return SoapySDR::Range(0.0, 19.0+12.0+30.0);
	}
	return SoapySDR::Device::getGainRange(direction, channel);
}

SoapySDR::Range SoapyXTRX::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
	if (direction == SOAPY_SDR_RX and name == "LNA") return SoapySDR::Range(0.0, 30.0);
	if (direction == SOAPY_SDR_RX and name == "TIA") return SoapySDR::Range(0.0, 12.0);
	if (direction == SOAPY_SDR_RX and name == "PGA") return SoapySDR::Range(-12.0, 19.0);
	if (direction == SOAPY_SDR_TX and name == "PAD") return SoapySDR::Range(-52.0, 0.0);

	return SoapySDR::Device::getGainRange(direction, channel, name);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/
SoapySDR::ArgInfoList SoapyXTRX::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
	auto infos = SoapySDR::Device::getFrequencyArgsInfo(direction, channel);
	/*{
		SoapySDR::ArgInfo info;
		info.key = "CORRECTIONS";
		info.name = "Corrections";
		info.value = "true";
		info.description = "Automatically apply DC/IQ corrections";
		info.type = SoapySDR::ArgInfo::BOOL;
		infos.push_back(info);
	}*/
	return infos;
}

void SoapyXTRX::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &/*args*/)
{
	xtrx_channel_t chan = to_xtrx_channels(channel);
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::setFrequency(, %d, %s, %g MHz)", int(channel), name.c_str(), frequency/1e6);
	int res;

	if (name == "RF")
	{
		double targetRfFreq = frequency;
		if (targetRfFreq < 30e6) targetRfFreq = 30e6;
		if (targetRfFreq > 3.8e9) targetRfFreq = 3.8e9;

		if (direction == SOAPY_SDR_TX) {
			res = xtrx_tune(_dev->dev(), XTRX_TUNE_TX_FDD, targetRfFreq, &_actual_rf_tx);
		} else {
			res = xtrx_tune(_dev->dev(), XTRX_TUNE_RX_FDD, targetRfFreq, &_actual_rf_rx);
		}
		if (res) {
			throw std::runtime_error("SoapyXTRX::setFrequency("+name+") unable to tune!");
		}
		return;
	}
	else if (name == "BB")
	{
		if (direction == SOAPY_SDR_TX) {
			res = xtrx_tune_ex(_dev->dev(), XTRX_TUNE_BB_TX, chan, frequency, &_actual_bb_tx[channel]);
		} else {
			res = xtrx_tune_ex(_dev->dev(), XTRX_TUNE_BB_RX, chan, frequency, &_actual_bb_rx[channel]);
		}
		if (res) {
			throw std::runtime_error("SoapyXTRX::setFrequency("+name+") unable to tune!");
		}
		return;
	}
	throw std::runtime_error("SoapyXTRX::setFrequency("+name+") unknown name");
}

double SoapyXTRX::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
	to_xtrx_channels(channel);
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	if (name == "RF")
	{
		if (direction == SOAPY_SDR_TX) {
			return _actual_rf_tx;
		} else {
			return _actual_rf_rx;
		}
	}
	else if (name == "BB")
	{
		if (direction == SOAPY_SDR_TX) {
			return _actual_bb_tx[channel];
		} else {
			return _actual_bb_rx[channel];
		}
	}

	throw std::runtime_error("SoapyXTRX::getFrequency("+name+") unknown name");
}

std::vector<std::string> SoapyXTRX::listFrequencies(const int /*direction*/, const size_t /*channel*/) const
{
	std::vector<std::string> opts;
	opts.push_back("RF");
	opts.push_back("BB");
	return opts;
}

SoapySDR::RangeList SoapyXTRX::getFrequencyRange(const int direction, const size_t /*channel*/, const std::string &name) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	SoapySDR::RangeList ranges;
	if (name == "RF")
	{
		ranges.push_back(SoapySDR::Range(30e6, 3.8e9));
	}
	else if (name == "BB")
	{
		uint64_t out = 0;
		int res = xtrx_val_get(_dev->dev(), (direction == SOAPY_SDR_TX) ? XTRX_TX : XTRX_RX, XTRX_CH_AB, XTRX_LMS7_DATA_RATE, &out);
		if (res)
			ranges.push_back(SoapySDR::Range(-0.0, 0.0));
		else
			ranges.push_back(SoapySDR::Range(-(double)out / 2, (double)out / 2));
	}
	return ranges;
}

SoapySDR::RangeList SoapyXTRX::getFrequencyRange(const int /*direction*/, const size_t /*channel*/) const
{
	SoapySDR::RangeList ranges;
	ranges.push_back(SoapySDR::Range(30e6, 3.8e9));
	return ranges;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyXTRX::setSampleRate(const int direction, const size_t channel, const double rate)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::setSampleRate(%d, %s, %g MHz)", int(channel), (direction == SOAPY_SDR_TX) ? "TX" : "RX", rate/1e6);
	if (direction == SOAPY_SDR_RX)
	{
		_tmp_rx = rate;
		if (std::abs(_tmp_rx - _actual_rx_rate) < 10)
			return;
	}
	else if (direction == SOAPY_SDR_TX)
	{
		_tmp_tx = rate;
		if (std::abs(_tmp_tx - _actual_tx_rate) < 10)
			return;
	}
	else
	{
		return;
	}

	int ret = xtrx_set_samplerate(_dev->dev(), 0, _tmp_rx, _tmp_tx,
								  0, //XTRX_SAMPLERATE_FORCE_UPDATE,
								  &_actual_masterclock, &_actual_rx_rate, &_actual_tx_rate);

	if (ret) {
		SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyXTRX::setSampleRate(%d, %s, %g MHz) - error %d",
					   int(channel), (direction == SOAPY_SDR_TX) ? "TX" : "RX", rate/1e6, ret);

		throw std::runtime_error("SoapyXTRX::setSampleRate() unable to set samplerate!");
	}
}

double SoapyXTRX::getSampleRate(const int direction, const size_t /*channel*/) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);

	if (direction == SOAPY_SDR_RX)
	{
		return _actual_rx_rate;
	}
	else if (direction == SOAPY_SDR_TX)
	{
		return _actual_tx_rate;
	}

	return 0;
}

SoapySDR::RangeList SoapyXTRX::getSampleRateRange(const int direction, const size_t /*channel*/) const
{
	SoapySDR::RangeList ranges;
	if (direction == SOAPY_SDR_TX)
	{
		ranges.push_back(SoapySDR::Range(2.1e6, 56.25e6));
	}
	else
	{
		ranges.push_back(SoapySDR::Range(0.2e6, 56.25e6));
	}
	ranges.push_back(SoapySDR::Range(61.4375e6, 80e6));
	return ranges;
}

std::vector<double> SoapyXTRX::listSampleRates(const int direction, const size_t channel) const
{
	std::vector<double> rates;
	for (int i = 2; i < 57; i++)
	{
		rates.push_back(i*1e6);
	}
	return rates;
}
/*******************************************************************
 * Bandwidth API
 ******************************************************************/

void SoapyXTRX::setBandwidth(const int direction, const size_t channel, const double bw)
{
	if (bw == 0.0) return; //special ignore value

	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::setBandwidth(, %d, %g MHz)",  int(channel), bw/1e6);
	xtrx_channel_t chan = to_xtrx_channels(channel);

	if (direction == SOAPY_SDR_RX)
	{
		xtrx_tune_rx_bandwidth(_dev->dev(), chan, bw, &_actual_rx_bandwidth[channel]);
	}
	else if (direction == SOAPY_SDR_TX)
	{
		xtrx_tune_tx_bandwidth(_dev->dev(), chan, bw, &_actual_tx_bandwidth[channel]);
	}

	//restore dc offset mode
}

double SoapyXTRX::getBandwidth(const int direction, const size_t channel) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	/*xtrx_channel_t chan = */to_xtrx_channels(channel);

	if (direction == SOAPY_SDR_RX)
	{
		return _actual_rx_bandwidth[channel];
	}
	else if (direction == SOAPY_SDR_TX)
	{
		return _actual_tx_bandwidth[channel];
	}

	return 0;
}

SoapySDR::RangeList SoapyXTRX::getBandwidthRange(const int direction, const size_t /*channel*/) const
{
	SoapySDR::RangeList bws;

	if (direction == SOAPY_SDR_RX)
	{
		bws.push_back(SoapySDR::Range(1e6, 60e6));
	}
	if (direction == SOAPY_SDR_TX)
	{
		bws.push_back(SoapySDR::Range(0.8e6, 16e6));
		bws.push_back(SoapySDR::Range(28e6, 60e6));
	}

	return bws;
}

/*******************************************************************
 * Clocking API
 ******************************************************************/

void SoapyXTRX::setMasterClockRate(const double rate)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	//_ref_clk = rate;
	// xtrx_set_ref_clk(_dev->dev(), _ref_clk, _ref_source);
	// TODO: get reference clock in case of autodetection
}

double SoapyXTRX::getMasterClockRate(void) const
{
	return 0;
	//std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	//int64_t v;

	//int res = xtrx_val_get(_dev->dev(), XTRX_TRX, XTRX_CH_AB, XTRX_REF_REFCLK, &v);
	//if (res)
	//	throw std::runtime_error("SoapyXTRX::getMasterClockRate() unable to get master clock!");

	//return v;
}

SoapySDR::RangeList SoapyXTRX::getMasterClockRates(void) const
{
	SoapySDR::RangeList clks;
	clks.push_back(SoapySDR::Range(0, 0)); // means autodetect
	clks.push_back(SoapySDR::Range(10e6, 52e6));
	return clks;
}

std::vector<std::string> SoapyXTRX::listClockSources(void) const
{
	return { "internal", "extrernal", "ext+pps" };
}

void SoapyXTRX::setClockSource(const std::string &source)
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	if (source == "internal")
		_ref_source = XTRX_CLKSRC_INT;
	else if (source == "extrernal")
		_ref_source = XTRX_CLKSRC_EXT;
	else if (source == "ext+pps")
		_ref_source = XTRX_CLKSRC_EXT_W1PPS_SYNC;
	else
		return;

	xtrx_set_ref_clk(_dev->dev(), _ref_clk, _ref_source);
}

std::string SoapyXTRX::getClockSource(void) const
{
	switch (_ref_source) {
	case XTRX_CLKSRC_INT: return "internal";
	case XTRX_CLKSRC_EXT: return "extrernal";
	case XTRX_CLKSRC_EXT_W1PPS_SYNC: return "ext+pps";
	}

	return "<unknown>";
}

/*******************************************************************
 * Time API
 ******************************************************************/

bool SoapyXTRX::hasHardwareTime(const std::string &what) const
{
	//assume hardware time when no argument is specified
	//some boards may not ever support hw time, so TODO
	return what.empty();
}

long long SoapyXTRX::getHardwareTime(const std::string &what) const
{
	if (what.empty())
	{
		return 0;
	}
	else
	{
		throw std::invalid_argument("SoapyXTRX::getHardwareTime("+what+") unknown argument");
	}
}

void SoapyXTRX::setHardwareTime(const long long timeNs, const std::string &what)
{
	if (what.empty())
	{
	}
	else
	{
		throw std::invalid_argument("SoapyXTRX::setHardwareTime("+what+") unknown argument");
	}
}

/*******************************************************************
 * Sensor API
 ******************************************************************/

std::vector<std::string> SoapyXTRX::listSensors(void) const
{
	std::vector<std::string> sensors;
	sensors.push_back("clock_locked");
	sensors.push_back("lms7_temp");
	sensors.push_back("board_temp");
	return sensors;
}

SoapySDR::ArgInfo SoapyXTRX::getSensorInfo(const std::string &name) const
{
	SoapySDR::ArgInfo info;
	if (name == "clock_locked")
	{
		info.key = "clock_locked";
		info.name = "Clock Locked";
		info.type = SoapySDR::ArgInfo::BOOL;
		info.value = "false";
		info.description = "CGEN clock is locked, good VCO selection.";
	}
	else if (name == "lms7_temp")
	{
		info.key = "lms7_temp";
		info.name = "LMS7 Temperature";
		info.type = SoapySDR::ArgInfo::FLOAT;
		info.value = "0.0";
		info.units = "C";
		info.description = "The temperature of the LMS7002M in degrees C.";
	}
	else if (name == "board_temp")
	{
		info.key = "board_temp";
		info.name = "XTRX board temerature";
		info.type = SoapySDR::ArgInfo::FLOAT;
		info.value = "0.0";
		info.units = "C";
		info.description = "The temperature of the XTRX board in degrees C.";
	}
	return info;
}

std::string SoapyXTRX::readSensor(const std::string &name) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	int res = 0;
	uint64_t val;
	if (name == "clock_locked")
	{
		return "true";
	}
	else if (name == "lms7_temp")
	{
		return "0.0";
	}
	else if (name == "board_temp")
	{
		res = xtrx_val_get(_dev->dev(), XTRX_TRX, XTRX_CH_AB, XTRX_BOARD_TEMP, &val);
		if (res)
			throw std::runtime_error("SoapyXTRX::readSensor("+name+") error: " + std::to_string(res));

		return std::to_string(val / 256.0);
	}

	throw std::runtime_error("SoapyXTRX::readSensor("+name+") - unknown sensor name");
}

std::vector<std::string> SoapyXTRX::listSensors(const int /*direction*/, const size_t /*channel*/) const
{
	std::vector<std::string> sensors;
	sensors.push_back("lo_locked");
	return sensors;
}

SoapySDR::ArgInfo SoapyXTRX::getSensorInfo(const int /*direction*/, const size_t /*channel*/, const std::string &name) const
{
	SoapySDR::ArgInfo info;
	if (name == "lo_locked")
	{
		info.key = "lo_locked";
		info.name = "LO Locked";
		info.type = SoapySDR::ArgInfo::BOOL;
		info.value = "false";
		info.description = "LO synthesizer is locked, good VCO selection.";
	}
	return info;
}

std::string SoapyXTRX::readSensor(const int /*direction*/, const size_t /*channel*/, const std::string &name) const
{
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);

	if (name == "lo_locked")
	{
		return "true";
	}

	throw std::runtime_error("SoapyXTRX::readSensor("+name+") - unknown sensor name");
}

/*******************************************************************
 * Register API
 ******************************************************************/

void SoapyXTRX::writeRegister(const unsigned addr, const unsigned /*value*/)
{
	throw std::runtime_error(
				"SoapyXTRX::WriteRegister("+std::to_string(addr)+") FAIL");
}

unsigned SoapyXTRX::readRegister(const unsigned addr) const
{
	throw std::runtime_error(
				"SoapyXTRX::ReadRegister("+std::to_string(addr)+") FAIL");
}

/*******************************************************************
 * Settings API
 ******************************************************************/
SoapySDR::ArgInfoList SoapyXTRX::getSettingInfo(void) const
{
	SoapySDR::ArgInfoList infos;

	return infos;
}

void SoapyXTRX::writeSetting(const std::string &key, const std::string &/*value*/)
{
	throw std::runtime_error("unknown setting key: "+key);
}

SoapySDR::ArgInfoList SoapyXTRX::getSettingInfo(const int direction, const size_t channel) const
{
	// TODO
	(void)direction;
	(void)channel;

	SoapySDR::ArgInfoList infos;
	return infos;
}

void SoapyXTRX::writeSetting(const int direction, const size_t channel,
							 const std::string &key, const std::string &value)
{
	// TODO
	(void)direction;
	(void)channel;
	(void)key;
	(void)value;

	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	throw std::runtime_error("unknown setting key: "+key);
}

/*******************************************************************
 * I2C API
 ******************************************************************/
void SoapyXTRX::writeI2C(const int addr, const std::string &/*data*/)
{
	throw std::runtime_error(
				"SoapyXTRX::writeI2C("+std::to_string(addr)+") FAIL");
}

std::string SoapyXTRX::readI2C(const int addr, const size_t /*numBytes*/)
{
	throw std::runtime_error(
				"SoapyXTRX::readI2C("+std::to_string(addr)+") FAIL");
}

/*******************************************************************
 * SPI API
 ******************************************************************/
unsigned SoapyXTRX::transactSPI(const int addr, const unsigned /*data*/, const size_t /*numBits*/)
{
	throw std::runtime_error(
				"SoapyXTRX::transactSPI("+std::to_string(addr)+") FAIL");
}




/*******************************************************************
 * Stream data structure
 ******************************************************************/
struct XTRXConnectionStream
{
};

/*******************************************************************
 * Stream information
 ******************************************************************/
std::vector<std::string> SoapyXTRX::getStreamFormats(const int direction, const size_t /*channel*/) const
{
	std::vector<std::string> formats;
	formats.push_back(SOAPY_SDR_CF32);
	if (direction == SOAPY_SDR_RX) {
		formats.push_back(SOAPY_SDR_CS8);
		//formats.push_back(SOAPY_SDR_CS12);
	}
	formats.push_back(SOAPY_SDR_CS16);
	return formats;
}

std::string SoapyXTRX::getNativeStreamFormat(const int direction, const size_t /*channel*/, double &fullScale) const
{
	if (direction == SOAPY_SDR_RX) {
		fullScale = 2048;
	} else {
		fullScale = 32768;
	}
	return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyXTRX::getStreamArgsInfo(const int direction, const size_t /*channel*/) const
{
	SoapySDR::ArgInfoList argInfos;

	//float scale
	{
		SoapySDR::ArgInfo info;
		info.key = "floatScale";
		info.name = "Float Scale";
		info.description = "The buffer will be scaled (or expected to be scaled) to [-floatScale;floatScale)";
		info.type = SoapySDR::ArgInfo::FLOAT;
		info.value = "1.0";
		argInfos.push_back(info);
	}

	//link format
	{
		SoapySDR::ArgInfo info;
		info.key = "linkFormat";
		info.name = "Link Format";
		info.description = "The format of the samples over the link.";
		info.type = SoapySDR::ArgInfo::STRING;
		info.options.push_back(SOAPY_SDR_CS16);
		info.optionNames.push_back("Complex int16");
		info.value = SOAPY_SDR_CS16;

		if (direction == SOAPY_SDR_RX) {
			info.options.push_back(SOAPY_SDR_CS8);
			info.optionNames.push_back("Complex int8");

			//info.options.push_back(SOAPY_SDR_CS12);
			//info.optionNames.push_back("Complex int12");
		}

		argInfos.push_back(info);
	}

	return argInfos;
}

#define STREAM_RX ((SoapySDR::Stream *)(uintptr_t(SOAPY_SDR_RX)+0x8000))
#define STREAM_TX ((SoapySDR::Stream *)(uintptr_t(SOAPY_SDR_TX)+0x8000))
#define STREAM_STR(STREAM) ((STREAM == STREAM_RX) ? "RX" : "TX")

/*******************************************************************
 * Stream config
 ******************************************************************/
SoapySDR::Stream *SoapyXTRX::setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels,
		const SoapySDR::Kwargs &args)
{
	//TODO: multi stream
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
	xtrx_run_stream_params_t *params;
	xtrx_antenna_t antenna;
	size_t num_channels = channels.size();
	if (num_channels < 1)
		num_channels = 1;

	if (direction == SOAPY_SDR_RX) {
		if (_rx_stream != SS_NONE) {
			std::runtime_error("SoapyXTRX::setupStream(RX) stream is already allocated");
		}

		params = &_stream_params.rx;
		_rx_channels = num_channels;
		antenna = XTRX_RX_AUTO;

		xtrx_stop(_dev->dev(), XTRX_RX);
	} else if (direction == SOAPY_SDR_TX) {
		if (_tx_stream != SS_NONE) {
			std::runtime_error("SoapyXTRX::setupStream(TX) stream is already allocated");
		}

		params = &_stream_params.tx;
		_tx_channels = num_channels;
		antenna = XTRX_TX_AUTO;

		xtrx_stop(_dev->dev(), XTRX_TX);
	} else {
		throw std::runtime_error("SoapyXTRX::setupStream(?) unsupported direction");
	}

	xtrx_wire_format_t wfmt = XTRX_WF_16;
	bool wfmt_given = false;
	if (args.count("linkFormat")) {
		const std::string& link_fmt = args.at("linkFormat");
		if (link_fmt == SOAPY_SDR_CS16) {
			wfmt = XTRX_WF_16;
		} else if (link_fmt == SOAPY_SDR_CS12) {
			wfmt = XTRX_WF_12;
		} else if (link_fmt == SOAPY_SDR_CS8) {
			wfmt = XTRX_WF_8;
		} else {
			throw std::runtime_error("SoapyXTRX::setupStream([linkFormat="+link_fmt+"]) unsupported link format");
		}
		wfmt_given = true;
	}

	if (format == SOAPY_SDR_CF32) {
		params->hfmt = XTRX_IQ_FLOAT32;
		if (wfmt_given) {
			params->wfmt = wfmt;
		} else {
			params->wfmt = XTRX_WF_16;
		}
	} else if (format == SOAPY_SDR_CS16) {
		params->hfmt = XTRX_IQ_INT16;
		params->wfmt = XTRX_WF_16;
	} else if (format == SOAPY_SDR_CS8 && direction == SOAPY_SDR_RX) {
		params->hfmt = XTRX_IQ_INT8;
		params->wfmt = XTRX_WF_8;
	} else {
		throw std::runtime_error("SoapyXTRX::setupStream(format="+format+") unsupported format");
	}

	params->flags = 0;
	params->paketsize = 0;

	if (args.count("floatScale")) {
		const std::string& float_scale = args.at("floatScale");
		params->scale = std::atof(float_scale.c_str());
		if (params->scale <= 0) {
			throw std::runtime_error("SoapyXTRX::setupStream([floatScale="+float_scale+") unsupported scale");
		}
		params->flags |= XTRX_RSP_SCALE;
	}

	if (args.count("syncRxTxStreamsAct")) {
		const std::string& sync_rx_tx_streams_act = args.at("syncRxTxStreamsAct");
		if (sync_rx_tx_streams_act == "true") {
			_sync_rx_tx_streams_act = true;
		} else if (sync_rx_tx_streams_act == "false") {
			_sync_rx_tx_streams_act = false;
		} else {
			throw std::runtime_error("SoapyXTRX::setupStream([syncRxTxStreamsAct="+sync_rx_tx_streams_act+"])"
									 "unsupported value");
		}
	}

	if (num_channels == 1) {
		params->flags |= XTRX_RSP_SISO_MODE;
		if (channels.size() == 0 || channels[0] == 0) {
			params->chs = XTRX_CH_AB;
			params->flags |= XTRX_RSP_SISO_MODE;
		} else if (channels[0] == 1) {
			params->chs = XTRX_CH_AB;
			params->flags |= XTRX_RSP_SISO_MODE | XTRX_RSP_SWAP_AB;
		} else {
			throw std::runtime_error("SoapyXTRX::setupStream([x]) unsupported channels");
		}
	} else if (num_channels == 2) {
		if (channels[0] == 0 && channels[1] == 1) {
			params->chs = XTRX_CH_AB;
		} else if (channels[0] == 1 && channels[1] == 0) {
			params->chs = XTRX_CH_AB;
			params->flags |= XTRX_RSP_SWAP_AB;
		} else {
			throw std::runtime_error("SoapyXTRX::setupStream([x,y]) unsupported channels");
		}
	} else {
		throw std::runtime_error("SoapyXTRX::setupStream() unsupported number of channels!");
	}

	int res = xtrx_set_antenna(_dev->dev(), antenna);
	if (res) {
		throw std::runtime_error("SoapyXTRX::setupStream() set antenna AUTO xtrx_set_antenna() err");
	}

	if (direction == SOAPY_SDR_RX) {
		_rx_stream = SS_ALOCATED;
		return STREAM_RX;
	} else {
		_tx_stream = SS_ALOCATED;
		return STREAM_TX;
	}
}

void SoapyXTRX::closeStream(SoapySDR::Stream *stream)
{
	//TODO: multi stream
	(void)stream;
	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);
}

size_t SoapyXTRX::getStreamMTU(SoapySDR::Stream *stream) const
{
	//TODO: multi stream
	(void)stream;

	return 4096;
}

int SoapyXTRX::activateStream(
		SoapySDR::Stream *stream,
		const int flags,
		const long long timeNs,
		const size_t numElems)
{
	if (numElems > 32767) {
		throw std::runtime_error("SoapyXTRX::activateStream() - too much packet size");
	}

	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);

	if (_sync_rx_tx_streams_act && (_rx_stream == SS_ACTIVATED) && (_tx_stream == SS_ACTIVATED)) {
		SoapySDR::logf(SOAPY_SDR_INFO, "SoapyXTRX::activateStream(%s) Sync TX and RX streams activation mode"
					   " is used, RX and TX streams have been configured and activated earlier during"
					   " %s stream activation.",
					   STREAM_STR(stream), (stream == STREAM_RX) ? "TX" : "RX");
		return 0;
	}

	if ((stream == STREAM_RX) || _sync_rx_tx_streams_act) {
		if (_rx_stream != SS_ALOCATED)
			throw std::runtime_error("SoapyXTRX::activateStream() - RX stream isn't allocated!");

		if (_actual_rx_rate < 1) {
			//throw std::runtime_error("SoapyXTRX::activateStream() - the RX sample rate has not been configured!");
			setSampleRate(SOAPY_SDR_RX, 0, 2.1e6);
		}

		if (flags & SOAPY_SDR_HAS_TIME) {
			_stream_params.rx_stream_start = (master_ts)SoapySDR::timeNsToTicks(timeNs, _actual_rx_rate);
		} else {
			_stream_params.rx_stream_start = 32768;
		}
		_stream_params.rx.paketsize = (uint16_t)numElems;
		_stream_params.dir = XTRX_RX;
	}
	if ((stream == STREAM_TX) || _sync_rx_tx_streams_act) {
		if (_tx_stream != SS_ALOCATED)
			throw std::runtime_error("SoapyXTRX::activateStream() - TX stream isn't allocated!");

		if (_actual_tx_rate < 1)
			throw std::runtime_error("SoapyXTRX::activateStream() - the TX sample rate has not been configured!");

		_stream_params.tx.paketsize = (uint16_t)numElems;
		_stream_params.tx_repeat_buf = NULL;
		_stream_params.dir = XTRX_TX;

		if (flags & SOAPY_SDR_HAS_TIME) {
			_tx_internal = SoapySDR::timeNsToTicks(timeNs, _actual_tx_rate);
		} else {
			_tx_internal = 32768;
		}
	}
	if ((stream != STREAM_RX) && (stream != STREAM_TX)) {
		throw std::runtime_error("SoapyXTRX::activateStream() - incorrect stream");
	}

	if (_sync_rx_tx_streams_act) {
		_stream_params.dir = XTRX_TRX;
	}

	_stream_params.nflags = 0;
	int res = xtrx_run_ex(_dev->dev(), &_stream_params);
	if (res == 0) {
		if ((stream == STREAM_RX) || _sync_rx_tx_streams_act) {
			_rx_stream = SS_ACTIVATED;
		}
		if ((stream == STREAM_TX) || _sync_rx_tx_streams_act) {
			_tx_stream = SS_ACTIVATED;
		}
	}

	if (_sync_rx_tx_streams_act) {
		SoapySDR::logf(SOAPY_SDR_INFO, "SoapyXTRX::activateStream(%s) Sync TX and RX streams activation mode"
					   " is used, RX and TX streams are configured and activated, TX Samples per packet: %d,"
					   " RX Samples per packet: %d, res =  %d",
					   STREAM_STR(stream), _stream_params.tx.paketsize, _stream_params.rx.paketsize, res);
	} else {
		SoapySDR::logf(SOAPY_SDR_INFO, "SoapyXTRX::activateStream(%s) %d Samples per packet; res = %d",
					   STREAM_STR(stream), numElems, res);
	}

	return (res) ? SOAPY_SDR_NOT_SUPPORTED : 0;
}

int SoapyXTRX::deactivateStream(
		SoapySDR::Stream *stream,
		const int flags,
		const long long timeNs)
{
	// TODO: use timeNs
	(void)flags;
	(void)timeNs;

	std::unique_lock<std::recursive_mutex> lock(_dev->accessMutex);

	if (stream == STREAM_RX) {
		if (_rx_stream != SS_ACTIVATED)
			return SOAPY_SDR_STREAM_ERROR;

		xtrx_stop(_dev->dev(), XTRX_RX);
		_rx_stream = SS_ALOCATED;

		return 0;
	} else if (stream == STREAM_TX) {
		if (_tx_stream != SS_ACTIVATED)
			return SOAPY_SDR_STREAM_ERROR;

		xtrx_stop(_dev->dev(), XTRX_TX);
		_tx_stream = SS_ALOCATED;

		return 0;
	}
	return SOAPY_SDR_STREAM_ERROR;
}

/*******************************************************************
 * Stream API
 ******************************************************************/
int SoapyXTRX::readStream(
		SoapySDR::Stream *stream,
		void * const *buffs,
		size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
	// TODO: timeoutUs
	(void)timeoutUs;

	if (stream != STREAM_RX || _rx_stream != SS_ACTIVATED) {
		return SOAPY_SDR_STREAM_ERROR;
	}

	xtrx_recv_ex_info rex;
	rex.samples = numElems;
	rex.buffer_count = (unsigned)_rx_channels;
	rex.buffers = buffs;
	rex.flags = RCVEX_DONT_INSER_ZEROS; //RCVEX_EXTRA_LOG;

	int res = xtrx_recv_sync_ex(_dev->dev(), &rex);
	if (res) {
		SoapySDR::logf(SOAPY_SDR_INFO, "SoapyXTRX::readStream(%d) res = %d", numElems, res);
	}

	flags |= SOAPY_SDR_HAS_TIME;
	timeNs = SoapySDR::ticksToTimeNs(rex.out_first_sample, _actual_rx_rate);

	return (res) ? SOAPY_SDR_TIMEOUT : rex.out_samples;
}

int SoapyXTRX::writeStream(
		SoapySDR::Stream *stream,
		const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs)
{
	// TODO: timeoutUs
	(void)timeoutUs;

	if (stream != STREAM_TX || _tx_stream != SS_ACTIVATED) {
		return SOAPY_SDR_STREAM_ERROR;
	}

	long long ts = (flags & SOAPY_SDR_HAS_TIME) ?
				SoapySDR::timeNsToTicks(timeNs, _actual_tx_rate) : _tx_internal;

	unsigned toSend = numElems;

	xtrx_send_ex_info_t nfo;
	nfo.buffer_count = _tx_channels;
	nfo.buffers = buffs;
	nfo.flags = 0;
	nfo.samples = toSend;
	nfo.ts = ts;
	nfo.timeout = timeoutUs / 1000;

	//std::cerr << "SAMPLES: " << numElems << " TS:" << ts <<std::endl;
	int res = xtrx_send_sync_ex(_dev->dev(), &nfo);

	if (~(flags & SOAPY_SDR_HAS_TIME)) {
		_tx_internal += toSend;
	}

	return (res) ? SOAPY_SDR_TIMEOUT : toSend;
}

int SoapyXTRX::readStreamStatus(
		SoapySDR::Stream *stream,
		size_t &chanMask,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
	(void)stream;
	(void)chanMask;
	(void)flags;
	(void)timeNs;
	(void)timeoutUs;

	return SOAPY_SDR_TIMEOUT; //SOAPY_SDR_NOT_SUPPORTED;
}

xtrx_channel_t SoapyXTRX::to_xtrx_channels(const size_t channel) const
{
	if (channel == 0)
		return XTRX_CH_A;
	else if (channel == 1)
		return XTRX_CH_B;
	else
		throw std::runtime_error("SoapyXTRX: incorret number of channel provided");
}
