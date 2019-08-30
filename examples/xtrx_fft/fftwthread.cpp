#include "fftwthread.h"
#include <math.h>


FFTWThread::FFTWThread(MainWindow *parent)
: FFTThread(parent)
{
}

FFTWThread::~FFTWThread()
{
	fftwf_destroy_plan(fftplan);
	fftwf_free(in);
	fftwf_free(out);
}

#define OPT_ORD 3
void FFTWThread::run()
{
	const unsigned MAX_FFTS = _wnd->max_ffts;

	fft_size = _wnd->fft_size;
	const unsigned max_samples = fft_size * MAX_FFTS;

	//float fft_data_buffer[2 * max_samples * MAX_DEVS];
	float *fft_data_buffer = new float[2 * max_samples * MAX_DEVS];
	xtrx_recv_ex_info_t rx_nfo;

	void *buffers[MAX_DEVS];
	for (int i = 0; i < MAX_DEVS; ++i) {
		buffers[i] = &fft_data_buffer[2 * max_samples * i];
	}

	rx_nfo.samples = max_samples;
	rx_nfo.buffer_count = _wnd->devices;
	rx_nfo.buffers = buffers;
	rx_nfo.flags = RCVEX_DONT_INSER_ZEROS;
	rx_nfo.timeout = 5000;

	for (unsigned i=0; i<fft_size; ++i) {
		_wnd->x[i] = i;
	}

	in = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * fft_size);
	out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * fft_size);
	fftplan = fftwf_plan_dft_1d(fft_size, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

	unsigned vd = 0;
	unsigned bidx = MAX_FFTS;
	float ly[fft_size];
	float nl[fft_size];

	for (unsigned i = 0; i < fft_size; ++i) {
		ly[i] = 1;
		nl[i] = 0;
	}

	for (uint64_t k = 0; !isInterruptionRequested(); k++) {
		unsigned avg = *(volatile unsigned*)&fft_avg;
		int cwnd = *(volatile int*)&_wnd->cwnd;

		if (avg < 1)
			avg = 1;
		double scale = 1. / avg;
		unsigned aidx;

		for (aidx = 0; aidx < avg; aidx++, bidx++) {
			if (bidx >= MAX_FFTS) {
				int res = xtrx_recv_sync_ex(_wnd->dev, &rx_nfo);
				if (res) {
					return;
				}
				bidx = 0;
			}

			for (int d = 0; d < _wnd->devices; ++d) {
				// DO FFT CALC
				if (cwnd == 0) {
					memcpy(in,
						   fft_data_buffer + 2 * max_samples * d + 2 * fft_size * bidx,
						   sizeof(fftwf_complex) * fft_size);
				} else {
					const float* wnd = &_wnd->wnd[cwnd].at(0);
					for (unsigned i = 0; i < fft_size; i++) {
						in[i][0] = wnd[i] * fft_data_buffer[2 * max_samples * d + 2 * fft_size * bidx + 2 * i + 0];
						in[i][1] = wnd[i] * fft_data_buffer[2 * max_samples * d + 2 * fft_size * bidx + 2 * i + 1];
					}
				}

				fftwf_execute(fftplan);

#ifdef OPT_ORD
				for (unsigned i = 0; i < fft_size; ++i) {
					float energy = (out[i][0] * out[i][0] + out[i][1] * out[i][1]);
					if (energy == 0) {
						energy = 1.f/2048/2048/fft_size;
					}
					ly[i] *= energy;
				}

				if (aidx % OPT_ORD == 0) {
					for (unsigned i = 0; i < fft_size; ++i) {
						nl[i] += log10f(ly[i]);
						ly[i] = 1;
					}
				}
#else
				for (unsigned i = 0; i < fft_size; ++i) {
					float energy = (out[i][0] * out[i][0] + out[i][1] * out[i][1]);
					if (energy == 0) {
						energy = 1.f/2048/2048/fft_size;
					}
					nl[i] += log10f(energy);
				}
#endif
			}
		}
#ifdef OPT_ORD
		if (aidx % OPT_ORD != 0) {
			for (unsigned i = 0; i < fft_size; ++i) {
				nl[i] += log10f(ly[i]);
				ly[i] = 1;
			}
		}
#endif

		for (int d = 0; d < _wnd->devices; ++d) {
			QVector<double>* py[] = {&_wnd->y1[d], &_wnd->y2[d], &_wnd->y3[d], &_wnd->y4[d]};
			QVector<double>& y = *(py[vd]);
			float pcorr = _wnd->wnd_p_corr[cwnd];

			for (unsigned i = 0; i < fft_size; i++) {
				double scl = scale * 10.0 * nl[i] - 20.0 * log10f(fft_size) - pcorr; // - 10 * log10f(2);
				y[i ^ (fft_size/2)] = scl;

				nl[i] = 0;
			}

			_wnd->dc_pwr[d] =  y[fft_size / 2];

			printf("DC = %.3f\n", _wnd->dc_pwr[d]);
		}

		emit newRxData(vd);

		vd++;
		if (vd > 3)
			vd = 0;
	}

}
