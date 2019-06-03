#include "adf4355.h"
#include <unistd.h>
#include <errno.h>
#include <xtrxll_log.h>

#define MAKE_ADF4355_R0(a,p,n) \
	((((a) & 1) << 21) | (((p) & 1) << 20) | (((n) & 0xffff) << 4) | 0x0)
#define MAKE_ADF4355_R1(f) \
	((((f) & 0xffffff) << 4) | 0x1)
#define MAKE_ADF4355_R2(f, m) \
	((((f) & 0x3fff) << 18) | (((m) & 0x3fff) << 4) | 0x2)
#define MAKE_ADF4355_R3(sdr, phr, pha, p) \
	((((sdr) & 1) << 30) | (((phr) & 1) << 29) | (((pha) & 1) << 28) | (((p) & 0xffffff) << 4) | 0x3)

#define MAKE_ADF4355_R4(mxo, rdbr, ddbr, r, dbuf, cs, refm, mux, pdp, pd, cps, cr) \
	((((mxo) & 7) << 27) | \
	(((rdbr) & 1) << 26) | \
	(((ddbr) & 1) << 25) | \
	(((r) & 0x3ff) << 15) | \
	(((dbuf) & 1) << 14) | \
	(((cs) & 0xf) << 10) | \
	(((refm) & 1) << 9) | \
	(((mux) & 1) << 8) | \
	(((pdp) & 1) << 7) | \
	(((pd) & 1) << 6) | \
	(((cps) & 1) << 5) | \
	(((cr) & 1) << 4) | \
	0x4)
#define MAKE_ADF4355_R5() 0x800025
#define MAKE_ADF4355_R6(gb, nb, fs, rfds, cpbc, mtld, auxen, auxop, rfen, rfop) \
	((((gb) & 1) << 30) | \
	(((nb) & 1) << 29) | \
	(((0xc) & 0xf) << 25) | \
	(((fs) & 1) << 24) | \
	(((rfds) & 0x7) << 21) | \
	(((cpbc) & 0xff) << 13) | \
	(((mtld) & 1) << 11) | \
	(((auxen) & 1) << 9) | \
	(((auxop) & 3) << 7) | \
	(((rfen) & 1) << 6) | \
	(((rfop) & 3) << 4) | \
	0x6)
#define MAKE_ADF4355_R7(les, ldcc, lolm, fracnld, lodm) \
	((((0x4) & 0x3f) << 26) | \
	(((les) & 1) << 25) | \
	(((ldcc) & 0x3) << 8) | \
	(((lolm) & 1) << 7) | \
	(((fracnld) & 0x3) << 5) | \
	(((lodm) & 1) << 4) | \
	0x7)

#define MAKE_ADF4355_R8() 0x1A69A6B8
#define MAKE_ADF4355_R9(vcob, to, slt) \
	((((vcob) & 0xff) << 24) | \
	(((to) & 0x3ff) << 14) | \
	(((0x1f) & 0x1f) << 9) | \
	(((slt) & 0x1f) << 4) | \
	0x9)
#define MAKE_ADF4355_R10(adccd, adcc, adce) \
	((((0x3) & 0x3) << 29) | \
	(((adccd) & 0xff) << 6) | \
	(((adcc) & 0x1) << 5) | \
	(((adce) & 0x1) << 4) | \
	0xa)
#define MAKE_ADF4355_R11() 0x0081200B

#define MAKE_ADF4355_R12(rc) \
	((((rc) & 0xffff) << 16) | \
	0x50c)

int adf4355_pd(spi_out_func_t spi_func, void* spi_obj)
{
	uint32_t adf4 = MAKE_ADF4355_R4(6, 0, 0, 1, 0, 2, 0, 1, 1, 1, 0, 0);
	return spi_func(spi_obj, adf4);
}

int adf4355_muxout(spi_out_func_t spi_func, void* spi_obj,
				   enum adf4355_muxout mux)
{
	unsigned icp_idx = 2;
	unsigned ref_doubler = 1;
	uint32_t adf4 = MAKE_ADF4355_R4(mux & 0x7, ref_doubler, 0, 1/*R*/, 0 /* db rfdiv*/, icp_idx, 0, 1, 1, 0, 0, 0);
	return spi_func(spi_obj, adf4);
}

int adf4355_tune(spi_out_func_t spi_func, void* spi_obj,
						uint64_t outfreq, unsigned fref, unsigned flags)
{
	int res;

	static const uint16_t icp_ua[16] = {
		310,
		630,
		940,
		1250,
		1560,
		1880,
		2190,
		2500,
		2810,
		3130,
		3440,
		3750,
		4060,
		4380,
		4690,
		5000,
	};
#define VCO_MIN 3300e6
#define VCO_MAX 6600e6

	unsigned ref_doubler = 1;
	unsigned fpd = fref << ref_doubler;
	unsigned frac;
	unsigned intn;
	unsigned div;

	unsigned div_val;
	uint64_t vco_freq;
	for (div = 0, div_val = 1; div_val < 128; div++, div_val <<= 1) {
		vco_freq = ((uint64_t)outfreq) * div_val;
		if (vco_freq > VCO_MAX)
			return -EINVAL;
		if (vco_freq >= VCO_MIN)
			break;
	}
	if (div_val > 64) {
		return -EINVAL;
	}
	intn = vco_freq / fpd;
	frac = (vco_freq - ((uint64_t)fpd) * intn) * ((uint64_t)1 << 24) / fpd;

	unsigned icp_idx = 2; // Default for the best spurs
	if (flags & ADF4355_EN_INIT) {
	//if (1) {
		unsigned cp_bleed_c = (39 * fpd / 61440000) * icp_ua[icp_idx] / 900;
		if (cp_bleed_c > 255)
			cp_bleed_c = 255;
		unsigned a_en = (flags & ADF4355_EN_A) ? 1 : 0;
		unsigned b_en = (flags & ADF4355_EN_B) ? 1 : 0;
		unsigned rf_pwr = 2; // -4; -1; +2; +5 dBm pwr level

		unsigned vco_band = (fpd + 2400000 - 1) / 2400000;
		if (vco_band > 255)
			vco_band = 255;

		unsigned adc_div = ((((fpd + 100000 - 1) / 100000) - 2) + 3) / 4;
		if (adc_div < 0)
			adc_div = 1;
		else if (adc_div > 255)
			adc_div = 255;

		uint32_t adfregs[12] = {
			MAKE_ADF4355_R12(1),
			MAKE_ADF4355_R11(),
			MAKE_ADF4355_R10(adc_div, 1, 1),
			MAKE_ADF4355_R9(vco_band, 0x0ff, 0x14),
			MAKE_ADF4355_R8(),
			MAKE_ADF4355_R7(0, 3, 0, 0, 0),
			MAKE_ADF4355_R6(0, 0, 1, div, cp_bleed_c, 0, b_en, rf_pwr, a_en, rf_pwr),
			MAKE_ADF4355_R5(),
			MAKE_ADF4355_R4(6, ref_doubler, 0, 1/*R*/, 0 /* db rfdiv*/, icp_idx, 0, 1, 1, 0, 0, 0),
			MAKE_ADF4355_R3(0, 0, 0, 0),
			MAKE_ADF4355_R2(0, 130),
			MAKE_ADF4355_R1(frac),
		};
		for (unsigned i = 0; i < 12; i++) {
			XTRXLLS_LOG("ADF4", XTRXLL_INFO_LMS, "REG OUT %08x\n", adfregs[i]);
			int res = spi_func(spi_obj, adfregs[i]);
			if (res) {
				return res;
			}
		}

		usleep(1000);

		uint32_t adf0 = MAKE_ADF4355_R0(0, 0, intn);
		res = spi_func(spi_obj, adf0);
		if (res) {
			return res;
		}

		usleep(1000);
	}

	uint32_t adfregs2[] = {
		//MAKE_ADF4355_R10(adc_div, 1, 1),
		MAKE_ADF4355_R4(MUXOUT_DLCK, ref_doubler, 0, 1/*R*/, 0 /* db rfdiv*/, icp_idx, 0, 1, 1, 0, 0, 1),
		MAKE_ADF4355_R2(0, 130),
		MAKE_ADF4355_R1(frac),
		MAKE_ADF4355_R0(0, 0, intn),
		MAKE_ADF4355_R4(6, ref_doubler, 0, 1/*R*/, 0 /* db rfdiv*/, icp_idx, 0, 1, 1, 0, 0, 0),
	};
	for (unsigned i = 0; i < sizeof(adfregs2)/sizeof(adfregs2[0]); i++) {
		int res = spi_func(spi_obj, adfregs2[i]);
		if (res) {
			return res;
		}
	}

	usleep(1000);

	uint32_t adf1 = MAKE_ADF4355_R0(1, 0, intn);
	res = spi_func(spi_obj, adf1);
	if (res) {
		return res;
	}

	uint32_t tmp = MAKE_ADF4355_R4(6, ref_doubler, 0, 1/*R*/, 0 /* db rfdiv*/, icp_idx, 0, 1, 1, 0, 0, 0);
	res = spi_func(spi_obj, tmp);
	if (res) {
		return res;
	}
	return 0;
}
