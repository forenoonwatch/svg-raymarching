#pragma once

static constexpr const char* g_svgVertex = R"(
#version 460

void main() {
	float x = float((gl_VertexID & 1) << 2) - 1.0;
	float y = float((gl_VertexID & 2) << 1) - 1.0;
	gl_Position = vec4(x, -y, 0, 1);
}
)";

static constexpr const char* g_svgFragment = R"(
#version 460

layout (origin_upper_left) in vec4 gl_FragCoord;

layout (location = 0) uniform float u_time;
layout (location = 1) uniform vec2 u_resolution;
layout (location = 2) uniform uint u_nshapes;

#define FLAG_OPEN_PATH 1

layout (binding = 3, std430) readonly buffer b_curves {
	vec2 points[65536];
	uint pointCounts[1024];
	uint pathCounts[1024];
	uint fillColors[1024];
	uint strokeColors[1024];
	float strokeWidths[1024];
	uint pathFlags[1024];
};

layout (location = 0) out vec4 o_color;

// Solve cubic equation for roots
vec3 solveCubic(float a, float b, float c)
{
    float p = b - a*a / 3.0, p3 = p*p*p;
    float q = a * (2.0*a*a - 9.0*b) / 27.0 + c;
    float d = q*q + 4.0*p3 / 27.0;
    float offset = -a / 3.0;
    if(d >= 0.0) { 
        float z = sqrt(d);
        vec2 x = (vec2(z, -z) - q) / 2.0;
        vec2 uv = sign(x)*pow(abs(x), vec2(1.0/3.0));
        return vec3(offset + uv.x + uv.y);
    }
    float v = acos(-sqrt(-27.0 / p3) * q / 2.0) / 3.0;
    float m = cos(v), n = sin(v)*1.732050808;
    return vec3(m + m, -n - m, n - m) * sqrt(-p / 3.0) + offset;
}

const float NaN = intBitsToFloat(int(0xFFC00000u));
const float PI_V = 3.14159265358979323846264338327;

vec3 solveCubic2(float A, float B, float C) {
	vec3 s = vec3(NaN, NaN, NaN);

	float sqA = A * A;
	float p = 1.0 / 3.0 * (-1.0 / 3.0 * sqA + B);
	float q = 1.0 / 2.0 * (2.0 / 27.0 * A * sqA - 1.0 / 3.0 * A * B + C);

	float cbp = p * p * p;
	float D = q * q + cbp;

	float sub = 1.0 / 3.0 * A;

	if (D == 0) {
		if (q == 0) {
			s.x = 0.0 - sub;
		}
		else {
			float u = pow(-q, 1.0 / 3.0);
			s.x = 2 * u - sub;
			s.y = -u - sub;
		}
	}
	else if (D < 0) {
		float phi = 1.0 / 3.0 * acos(-q / sqrt(-cbp));
		float t = 2 * sqrt(-p);

		s.x = t * cos(phi) - sub;
		s.y = -t * cos(phi + PI_V / 3.0) - sub;
		s.z = -t * cos(phi - PI_V / 3.0) - sub;
	}
	else {
		float sqrtD = sqrt(D);
		float u = pow(sqrtD - q, 1.0 / 3.0);
		float v = -pow(sqrtD + q, 1.0 / 3.0);

		s.x = u + v - sub;
	}

	return s;
}

// Find the signed distance from a point to a bezier curve
float sdBezier(vec2 A, vec2 B, vec2 C, vec2 p)
{    
    B = mix(B + vec2(1e-4), B, step(1e-6, abs(B * 2.0 - A - C)));
    vec2 a = B - A, b = A - B * 2.0 + C, c = a * 2.0, d = A - p;
    vec3 k = vec3(3.*dot(a,b),2.*dot(a,a)+dot(d,b),dot(d,a)) / dot(b,b);      
    vec3 t = clamp(solveCubic(k.x, k.y, k.z), 0.0, 1.0);
    vec2 pos = A + (c + b*t.x)*t.x;
    float dis = length(pos - p);
    pos = A + (c + b*t.y)*t.y;
    dis = min(dis, length(pos - p));
    pos = A + (c + b*t.z)*t.z;
    dis = min(dis, length(pos - p));
    return dis;
}

dvec2 solveQuadratic(double a, double b, double c) {
	/*float det = b * b - 4.0 * a * c;
	float detSqrt = sqrt(det);
	float rcp = 0.5 / a;
	float bOver2A = b * rcp;

	if (b >= 0) {
		return vec2(-detSqrt * rcp - bOver2A, 2 * c / (-b - detSqrt));
	}
	else {
		return vec2(2 * c / (-b + detSqrt), detSqrt * rcp - bOver2A);
	}*/

	double det = double(b) * double(b) - 4.0 * double(a) * double(c);
	double detSqrt = sqrt(det);
	double rcp = double(0.5) / double(a);
	double bOver2A = double(b) * rcp;

	if (b <= 0.0) {
		return vec2(-detSqrt * rcp - bOver2A, 2.0 * double(c) / (-double(b) - detSqrt));
	}
	else {
		return vec2(2.0 * double(c) / (-double(b) + detSqrt), detSqrt * rcp - bOver2A);
	}
}

float dot2( vec2 v ) { return dot(v,v); }
float cro( vec2 a, vec2 b ) { return a.x*b.y-a.y*b.x; }
float cos_acos_3( float x ) { x=sqrt(0.5+0.5*x); return x*(x*(x*(x*-0.008972+0.039071)-0.107074)+0.576975)+0.5; } // https://www.shadertoy.com/view/WltSD7

float sdBezier2(in vec2 pos, in vec2 A, in vec2 B, in vec2 C, out vec2 outQ) {
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

int getIntersectCount(in vec2 P0, in vec2 P1, in vec2 P2, in vec2 p) {
	double a = double(P0.y) - 2.0 * double(P1.y) + double(P2.y);
	double b = 2.0 * (double(P1.y) - double(P0.y));
	double c = double(P0.y) - double(p.y);
	dvec2 roots = solveQuadratic(a, b, c);
	int intersectCount = 0;

	if (roots[0] >= 0.0 && roots[0] <= 1.0) {
		double t = roots[0];
		double curveX = double(P0.x) * (1.0 - t) * (1.0 - t)
			+ 2.0 * double(P1.x) * (1.0 - t) * t
			+ double(P2.x) * t * t;

		if (double(p.x) <= curveX) {
			++intersectCount;
		}
	}

	if (roots[1] >= 0.0 && roots[1] <= 1.0) {
		double t = roots[1];
		double curveX = double(P0.x) * (1.0 - t) * (1.0 - t)
			+ 2.0 * double(P1.x) * (1.0 - t) * t
			+ double(P2.x) * t * t;

		if (double(p.x) <= curveX) {
			++intersectCount;
		}
	}

	return intersectCount;
}

vec4 colorShape(uint ishape, inout uint ip, inout uint i, uint pathEndIndex) {
	//vec2 p = gl_FragCoord.xy * 0.75 + vec2(100, 50);
	vec2 p = gl_FragCoord.xy;

	float minD = 1e10;
	bool hasFill = fillColors[ishape] != 0;
	int intersectCount = 0;

	for (; ip < pathEndIndex; ++ip) {
		bool isOpen = (pathFlags[ip] & 1) != 0;
		uint istart = i;

		for (; i < pointCounts[ip] - 1; i += 2) {
			float d = sdBezier(points[i], points[i + 1], points[i + 2], p);

			if (d < minD) {
				minD = d;
			}

			intersectCount += getIntersectCount(points[i], points[i + 1], points[i + 2], p);
		}

		if (hasFill && isOpen) {
			intersectCount += getIntersectCount(points[istart], points[pointCounts[ip] - 1],
					points[pointCounts[ip] - 1], p);
		}

		i = pointCounts[ip];
	}

	vec4 c = unpackUnorm4x8(strokeColors[ishape]);

	if (hasFill && intersectCount % 2 != 0) {
		minD *= -1.0;
	}

	if (hasFill) {
		float a = c.a * smoothstep(-1.0, 1.0, minD + strokeWidths[ishape]);
		c = mix(unpackUnorm4x8(fillColors[ishape]), c, a);
	}

	return vec4(c.rgb, c.a * smoothstep(1.0, -1.0, minD - strokeWidths[ishape]));
	//return vec4(unpackUnorm4x8(fillColors[ishape]).rgb, smoothstep(1.0, -1.0, minD));
}

void main() {
	uint ip = 0;
	uint i = 0;

	o_color = vec4(0.5, 0.5, 0.5, 0);

	for (uint ishape = 0; ishape < u_nshapes; ++ishape) {
		vec4 color = colorShape(ishape, ip, i, pathCounts[ishape]);
		o_color = vec4(color.rgb * color.a + o_color.rgb * (1.0 - color.a), 1.0);
	}
}
)";

