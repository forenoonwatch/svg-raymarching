#pragma once

static constexpr const char* g_fstFragment = R"(
#version 460

layout (origin_upper_left) in vec4 gl_FragCoord;

layout (location = 0) uniform float u_time;
layout (location = 1) uniform vec2 u_resolution;
layout (location = 2) uniform uint u_npts;

layout (binding = 3, std430) readonly buffer b_curves {
	vec2 curves[];
};

layout (location = 0) out vec4 o_color;

float dot2( vec2 v ) { return dot(v,v); }
float cro( vec2 a, vec2 b ) { return a.x*b.y-a.y*b.x; }
float cos_acos_3( float x ) { x=sqrt(0.5+0.5*x); return x*(x*(x*(x*-0.008972+0.039071)-0.107074)+0.576975)+0.5; } // https://www.shadertoy.com/view/WltSD7

float sdBezier(in vec2 pos, in vec2 A, in vec2 B, in vec2 C, out vec2 outQ) {
	vec2 a = B - A;
	vec2 b = A - 2.0*B + C;
	vec2 c = a * 2.0;
	vec2 d = A - pos;

	// cubic to be solved (kx*=3 and ky*=3)
	float kk = 1.0/dot(b,b);
	float kx = kk * dot(a,b);
	float ky = kk * (2.0*dot(a,a)+dot(d,b))/3.0;
	float kz = kk * dot(d,a);      

	float res = 0.0;
	float sgn = 0.0;

	float p  = ky - kx*kx;
	float q  = kx*(2.0*kx*kx - 3.0*ky) + kz;
	float p3 = p*p*p;
	float q2 = q*q;
	float h  = q2 + 4.0*p3;

	if (h >= 0.0) {   // 1 root
		h = sqrt(h);

#if 0
		vec2 x = (vec2(h,-h)-q)/2.0;
#if 0
		// When p≈0 and p<0, h-q has catastrophic cancelation. So, we do
		// h=√(q²+4p³)=q·√(1+4p³/q²)=q·√(1+w) instead. Now we approximate
		// √ by a linear Taylor expansion into h≈q(1+½w) so that the q's
		// cancel each other in h-q. Expanding and simplifying further we
		// get x=vec2(p³/q,-p³/q-q). And using a second degree Taylor
		// expansion instead: x=vec2(k,-k-q) with k=(1-p³/q²)·p³/q
		if( abs(p)<0.001 )
		{
		float k = p3/q;              // linear approx
		//float k = (1.0-p3/q2)*p3/q;  // quadratic approx 
		x = vec2(k,-k-q);  
		}
#endif

		vec2 uv = sign(x)*pow(abs(x), vec2(1.0/3.0));
		float t = uv.x + uv.y;
#else
		h = (q<0.0) ? h : -h; // copysign()
		float x = (h-q)/2.0;
		float v = sign(x)*pow(abs(x),1.0/3.0);
		float t = v - p/v;
#endif

		// from NinjaKoala - single newton iteration to account for cancellation
		t -= (t*(t*t+3.0*p)+q)/(3.0*t*t+3.0*p);

		t = clamp( t-kx, 0.0, 1.0 );
		vec2  w = d+(c+b*t)*t;
		outQ = w + pos;
		res = dot2(w);
		sgn = cro(c+2.0*b*t,w);
	}
	else 
	{   // 3 roots
		float z = sqrt(-p);
#if 0
		float v = acos(q/(p*z*2.0))/3.0;
		float m = cos(v);
		float n = sin(v);
#else
		float m = cos_acos_3( q/(p*z*2.0) );
		float n = sqrt(1.0-m*m);
#endif
		n *= sqrt(3.0);
		vec3  t = clamp( vec3(m+m,-n-m,n-m)*z-kx, 0.0, 1.0 );
		vec2  qx=d+(c+b*t.x)*t.x; float dx=dot2(qx), sx=cro(a+b*t.x,qx);
		vec2  qy=d+(c+b*t.y)*t.y; float dy=dot2(qy), sy=cro(a+b*t.y,qy);
		if( dx<dy ) {res=dx;sgn=sx;outQ=qx+pos;} else {res=dy;sgn=sy;outQ=qy+pos;}
	}

	return sqrt( res )*sign(sgn);
}

void main() {
	//vec2 p = (2.0 * gl_FragCoord.xy - u_resolution) / u_resolution.y;
	vec2 p = gl_FragCoord.xy;

	//vec2 v0 = vec2(1.3, 0.9) * cos(u_time * 0.5 + vec2(0, 5));
	//vec2 v1 = vec2(1.3, 0.9) * cos(u_time * 0.6 + vec2(3, 4));
	//vec2 v2 = vec2(1.3, 0.9) * cos(u_time * 0.7 + vec2(2, 0));
	
	//vec2 v0 = curves[0];
	
	float d = 100000;

	int i = 0;
	while (i < (u_npts - 3)) {
		vec2 v0 = curves[i];
		vec2 v1 = curves[i + 1];
		vec2 v2 = curves[i + 2];

		vec2 kk;
		float dd = sdBezier(p, v0, v1, v2, kk);

		d = min(d, abs(dd) - 5);

		i += 2;
	}

	vec3 col = (d > 0) ? vec3(0.9, 0.6, 0.3) : vec3(0.65, 0.85, 1.0);
	col *= 1.0 - exp(-4.0 * abs(d));
	col *= 0.8 + 0.2 * cos(110.0 * d);
	col = mix(col, vec3(1.0), 1.0 - smoothstep(0.0, 0.015, abs(d)));

	//vec3 col = d > 1 ? vec3(1, 1, 1) : vec3(0, 0 ,0);

	o_color = vec4(col, 1);
}
)";

