#ifndef ADF4355_H_
#define ADF4355_H_

#include <stdint.h>

enum adf4355_flags {
	ADF4355_EN_INIT = 1,
	ADF4355_EN_A = 2,
	ADF4355_EN_B = 4,
};

enum adf4355_muxout {
	MUXOUT_3ST = 0,
	MUXOUT_DVDD = 1,
	MUXOUT_DGND = 2,
	MUXOUT_RDIV = 3,
	MUXOUT_NDIV = 4,
	MUXOUT_ALCK = 5,
	MUXOUT_DLCK = 6,
};

typedef int (*spi_out_func_t)(void *obj, uint32_t out);


int adf4355_pd(spi_out_func_t spi_func, void* spi_obj);
int adf4355_muxout(spi_out_func_t spi_func, void* spi_obj,
				   enum adf4355_muxout mux);
int adf4355_tune(spi_out_func_t spi_func, void* spi_obj,
				 uint64_t outfreq, unsigned fref, unsigned flags);


#endif
