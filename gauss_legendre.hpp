#pragma once

#include "common.hpp"

template <int Order, typename float_t>
struct GaussLegendreValues;

template <int Order, typename float_t>
struct GaussLegendreIntegration {
	template <typename IntegrandFunc>
	static float_t calculate_integral(IntegrandFunc&& func, float_t start, float_t end) {
		auto integral = static_cast<float_t>(0.0);

		for (int i = 0; i < Order; ++i) {
			auto xi = GaussLegendreValues<Order, float_t>::xi(i) * ((end - start) / float_t(2.0))
					+ ((end + start) / float_t(2.0));
			integral += GaussLegendreValues<Order, float_t>::wi(i) * func(xi);
		}

		return ((end - start) / float_t(2.0)) * integral;
	}
};

#define float_t float
#define float_t_namespace GLQ_Impl_float32
#define TYPED_NUMBER(N) ZN_CONCAT_LABEL(N, f)
#include "gauss_legendre_values.inl"
#undef TYPED_NUMBER
#undef float_t_namespace
#undef float_t

#define float_t double
#define float_t_namespace GLQ_Impl_float64
#define TYPED_NUMBER(N) N
#include "gauss_legendre_values.inl"
#undef TYPED_NUMBER
#undef float_t_namespace
#undef float_t
