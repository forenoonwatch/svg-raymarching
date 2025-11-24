#pragma once

constexpr const char* g_hatchVertex = R"(
#version 460

layout (location = 0) uniform vec2 u_invScreenSize;
layout (location = 1) uniform vec4 u_extents;

layout (location = 0) out vec2 v_texCoord;

void main() {
	uint b = 1 << (gl_VertexID % 6);
	vec2 baseCoord = vec2((0x19 & b) != 0, (0xB & b) != 0);
	gl_Position = vec4(fma(baseCoord, u_extents.zw, u_extents.xy) * u_invScreenSize, 0, 1);
	gl_Position.xy = fma(gl_Position.xy, vec2(2.0, -2.0), vec2(-1.0, 1.0));
	v_texCoord = baseCoord;
}
)";

constexpr const char* g_hatchFragment = R"(
#version 460

layout (origin_upper_left) in vec4 gl_FragCoord;

layout (location = 2) uniform vec2 u_minCurve[3];
layout (location = 5) uniform vec2 u_maxCurve[3];

layout (location = 0) in vec2 v_texCoord;

layout (location = 0) out vec4 o_color;

// Test if point p crosses line (a, b), returns sign of result
double testCross(dvec2 a, dvec2 b, dvec2 p) {
    return sign((b.y-a.y) * (p.x-a.x) - (b.x-a.x) * (p.y-a.y));
}

// Determine which side we're on (using barycentric parameterization)
double signBezier(dvec2 A, dvec2 B, dvec2 C, dvec2 p)
{ 
    B = mix(B + dvec2(1e-4), B, step(1e-6, abs(B * 2.0 - A - C)));
    dvec2 a = C - A, b = B - A, c = p - A;
	double denom = (a.x * b.y - b.x * a.y);
    dvec2 bary = dvec2(c.x * b.y - b.x * c.y, a.x*c.y-c.x*a.y);
    dvec2 d = dvec2(bary.y * 0.5, 0.0) + dvec2(denom) - bary.x - bary.y;
    return mix(
		sign(d.x * d.x - d.y),
		mix(-1.0, 1.0, step(testCross(A, B, p) * testCross(B, C, p), 0.0)),
        step((d.x - d.y), 0.0)) * testCross(A, C, B);
}

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

// Find the signed distance from a point to a bezier curve
float sdBezier(vec2 A, vec2 B, vec2 C, vec2 p)
{    
    B = mix(B + vec2(1e-4), B, step(1e-6, abs(B * 2.0 - A - C)));
    vec2 a = B - A, b = A - B * 2.0 + C, c = a * 2.0, d = A - p;
    vec3 k = vec3(3.*dot(a,b),2.*dot(a,a)+dot(d,b),dot(d,a)) / dot(b,b);      
    vec3 t = clamp(solveCubic(k.x, k.y, k.z), 0.0, 1.0);

    vec2 pos = A + (c + b*t.x)*t.x;
    float dis = length(pos - p);
	float sgn = sign(pos.y - p.y);

    pos = A + (c + b*t.y)*t.y;
	float l = length(pos - p);
	if (l < dis) {
		dis = l;
		sgn = sign(pos.y - p.y);
	}

    pos = A + (c + b*t.z)*t.z;
	l = length(pos - p);
	if (l < dis) {
		dis = l;
		sgn = sign(pos.y - p.y);
	}

    return dis * sgn;
}

float dot2( in vec2 v ) { return dot(v,v); }

// borrowed from here https://www.iquilezles.org/www/articles/distfunctions2d/distfunctions2d.htm
float sdBezierIQ(in vec2 A, in vec2 B, in vec2 C, in vec2 pos )
{
    vec2 a = B - A;
    vec2 b = A - 2.0*B + C;
    vec2 c = a * 2.0;
    vec2 d = A - pos;
    float kk = 1.0/dot(b,b);
    float kx = kk * dot(a,b);
    float ky = kk * (2.0*dot(a,a)+dot(d,b)) / 3.0;
    float kz = kk * dot(d,a);
    float res = 0.0;
    float p = ky - kx*kx;
    float p3 = p*p*p;
    float q = kx*(2.0*kx*kx-3.0*ky) + kz;
    float h = q*q + 4.0*p3;
    if( h >= 0.0)
    {
        h = sqrt(h);
        vec2 x = (vec2(h,-h)-q)/2.0;
        vec2 uv = sign(x)*pow(abs(x), vec2(1.0/3.0));
        float t = clamp( uv.x+uv.y-kx, 0.0, 1.0 );
        res = dot2(d + (c + b*t)*t);
    }
    else
    {
        float z = sqrt(-p);
        float v = acos( q/(p*z*2.0) ) / 3.0;
        float m = cos(v);
        float n = sin(v)*1.732050808;
        vec3  t = clamp(vec3(m+m,-n-m,n-m)*z-kx,0.0,1.0);
        res = min( dot2(d+(c+b*t.x)*t.x),
                   dot2(d+(c+b*t.y)*t.y) );
        // the third root cannot be the closest
        // res = min(res,dot2(d+(c+b*t.z)*t.z));
    }
    return sqrt( res );
}

float cro( vec2 a, vec2 b ) { return a.x*b.y-a.y*b.x; }
float cos_acos_3( float x ) { x=sqrt(0.5+0.5*x); return x*(x*(x*(x*-0.008972+0.039071)-0.107074)+0.576975)+0.5; } // https://www.shadertoy.com/view/WltSD7

// signed distance to a quadratic bezier
float sdBezierIQ2( in vec2 A, in vec2 B, in vec2 C, in vec2 pos, out vec2 outQ) {    
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


    if( h>=0.0 ) 
    {   // 1 root
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
    	//sgn = cro(c+2.0*b*t,w);
		sgn = w.x;
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
        if( dx<dy ) {
			res=dx;
			//sgn=sx;
			sgn = qx.x;
			outQ=qx+pos;
		} else {
			res=dy;
			//sgn=sy;
			sgn = qy.x;
			outQ=qy+pos;
		}
    }
    
    //return sqrt( res )*sign(sgn);
    return sqrt( res );//*sign(sgn);
}

void main() {
	vec2 qMin;
	float dmin = sdBezierIQ2(u_minCurve[0], u_minCurve[1], u_minCurve[2], v_texCoord, qMin);
	vec2 qMax;
	float dmax = sdBezierIQ2(u_maxCurve[0], u_maxCurve[1], u_maxCurve[2], v_texCoord, qMax);

	bool swapped = false;

	if (qMin.x > qMax.x) {
		swapped = true;
		vec2 tmp = qMin;
		qMin = qMax;
		qMax = tmp;
	}

	//o_color = vec4(v_texCoord, 0, 1);
	o_color = v_texCoord.x >= qMin.x && v_texCoord.x <= qMax.x
		? (swapped ? vec4(1, 0, 0, 1) : vec4(0, 1, 0, 1)) : vec4(0, 0, 0, 0);
	//o_color = swapped ? vec4(1, 0, 0, 1) : vec4(0, 1, 0, 1);
}
)";
