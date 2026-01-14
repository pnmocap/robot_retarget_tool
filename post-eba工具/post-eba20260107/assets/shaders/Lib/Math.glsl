#ifndef _MATH_CONSTANTS_GLSL
#define _MATH_CONSTANTS_GLSL

// -- CONSTANTS:

#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.2831853071795864769252867665590
#define HALF_PI 1.5707963267948966192313216916398
#define EPSILON 0.00001

// -- FUNCTIONS:

#define clamp01(x) clamp(x, 0.0, 1.0)

float max3(vec3 v) { return max(max(v.x, v.y), v.z); }

float lerp(float a, float b, float t) { return (1.0 - t) * a + b * t; }
float invlerp(float a, float b, float v) { return (v - a) / (b - a); }

float remap(vec2 i, vec2 o, float v) {
  const float t = invlerp(i.x, i.y, v);
  return lerp(o.x, o.y, t);
}

bool isApproximatelyEqual(float a, float b) {
  return abs(a - b) <= (abs(a) < abs(b) ? abs(b) : abs(a)) * EPSILON;
}

float random(vec2 co) {
  // http://byteblacksmith.com/improvements-to-the-canonical-one-liner-glsl-rand-for-opengl-es-2-0/
  const float a = 12.9898;
  const float b = 78.233;
  const float c = 43758.5453;
  const float dt = dot(co, vec2(a, b));
  const float sn = mod(dt, PI);
  return fract(sin(sn) * c);
}

// https://softwareengineering.stackexchange.com/questions/212808/treating-a-1d-data-structure-as-2d-grid/212813

uint flatten2D(uvec2 id, uint width) { return id.x + width * id.y; }
uvec2 unflatten2D(uint i, uint width) { return uvec2(i % width, i / width); }

uint flatten3D(uvec3 id, uvec2 size) {
  return id.x + size.x * id.y + size.x * size.y * id.z;
}
uvec3 unflatten3D(uint i, uvec2 size) {
  return uvec3(i % size.x, (i / size.x) % size.y, i / (size.x * size.y));
}

vec2 rotateUV(vec2 uv, float rotation) {
    float mid = 0.5;
    return vec2(
        cos(rotation) * (uv.x - mid) + sin(rotation) * (uv.y - mid) + mid,
        cos(rotation) * (uv.y - mid) - sin(rotation) * (uv.x - mid) + mid
    );
}

vec2 rotateUV(vec2 uv, float rotation, vec2 mid) {
    return vec2(
      cos(rotation) * (uv.x - mid.x) + sin(rotation) * (uv.y - mid.y) + mid.x,
      cos(rotation) * (uv.y - mid.y) - sin(rotation) * (uv.x - mid.x) + mid.y
    );
}

vec2 rotateUV(vec2 uv, float rotation, float mid) {
    return vec2(
      cos(rotation) * (uv.x - mid) + sin(rotation) * (uv.y - mid) + mid,
      cos(rotation) * (uv.y - mid) - sin(rotation) * (uv.x - mid) + mid
    );
}

// Wraps noise for tiling texture creation
// @param v = unwrapped texture parameter
// @param bTiling = true to tile, false to not tile
// @param RepeatSize = number of units before repeating
// @return either original or wrapped coord
vec3 NoiseTileWrap(vec3 v,  bool bTiling, float RepeatSize)
{
	return bTiling ? (fract(v / RepeatSize) * RepeatSize) : v;
}

// 3D random number generator inspired by PCGs (permuted congruential generator)
// Using a **simple** Feistel cipher in place of the usual xor shift permutation step
// @param v = 3D integer coordinate
// @return three elements w/ 16 random bits each (0-0xffff).
// ~8 ALU operations for result.x    (7 mad, 1 >>)
// ~10 ALU operations for result.xy  (8 mad, 2 >>)
// ~12 ALU operations for result.xyz (9 mad, 3 >>)
uvec3 Rand3DPCG16(ivec3 p)
{
	// taking a signed int then reinterpreting as unsigned gives good behavior for negatives
	uvec3 v = uvec3(p);

	// Linear congruential step. These LCG constants are from Numerical Recipies
	// For additional #'s, PCG would do multiple LCG steps and scramble each on output
	// So v here is the RNG state
	v = v * 1664525u + 1013904223u;

	// PCG uses xorshift for the final shuffle, but it is expensive (and cheap
	// versions of xorshift have visible artifacts). Instead, use simple MAD Feistel steps
	//
	// Feistel ciphers divide the state into separate parts (usually by bits)
	// then apply a series of permutation steps one part at a time. The permutations
	// use a reversible operation (usually ^) to part being updated with the result of
	// a permutation function on the other parts and the key.
	//
	// In this case, I'm using v.x, v.y and v.z as the parts, using + instead of ^ for
	// the combination function, and just multiplying the other two parts (no key) for 
	// the permutation function.
	//
	// That gives a simple mad per round.
	v.x += v.y*v.z;
	v.y += v.z*v.x;
	v.z += v.x*v.y;
	v.x += v.y*v.z;
	v.y += v.z*v.x;
	v.z += v.x*v.y;

	// only top 16 bits are well shuffled
	return v >> 16u;
}



// 3D jitter offset within a voronoi noise cell
// @param pos - integer lattice corner
// @return random offsets vector
vec3 VoronoiCornerSample(vec3 pos, int Quality)
{
	// random values in [-0.5, 0.5]
	vec3 noise = vec3(Rand3DPCG16(ivec3(pos))) / 0xffff - 0.5;

	// quality level 1 or 2: searches a 2x2x2 neighborhood with points distributed on a sphere
	// scale factor to guarantee jittered points will be found within a 2x2x2 search
	if (Quality <= 2)
	{
		return normalize(noise) * 0.2588;
	}

	// quality level 3: searches a 3x3x3 neighborhood with points distributed on a sphere
	// scale factor to guarantee jittered points will be found within a 3x3x3 search
	if (Quality == 3)
	{
		return normalize(noise) * 0.3090;
	}

	// quality level 4: jitter to anywhere in the cell, needs 4x4x4 search
	return noise;
}

// compare previous best with a new candidate
// not producing point locations makes it easier for compiler to eliminate calculations when they're not needed
// @param minval = location and distance of best candidate seed point before the new one
// @param candidate = candidate seed point
// @param offset = 3D offset to new candidate seed point
// @param bDistanceOnly = if true, only set maxval.w with distance, otherwise maxval.w is distance and maxval.xyz is position
// @return position (if bDistanceOnly is false) and distance to closest seed point so far
vec4 VoronoiCompare(vec4 minval, vec3 candidate, vec3 offset, bool bDistanceOnly)
{
	if (bDistanceOnly)
	{
		return vec4(0, 0, 0, min(minval.w, dot(offset, offset)));
	}
	else
	{
		float newdist = dot(offset, offset);
		return newdist > minval.w ? minval : vec4(candidate, newdist);
	}
}

// 220 instruction Worley noise
vec4 VoronoiNoise3D_ALU(vec3 v, int Quality, bool bTiling, float RepeatSize, bool bDistanceOnly)
{
	vec3 fv = fract(v),  fv2 = fract(v + 0.5);
	vec3 iv = floor(v), iv2 = floor(v + 0.5);

	// with initial minimum distance = infinity (or at least bigger than 4), first min is optimized away
	vec4 mindist = vec4(0,0,0,100);
	vec3 p;
	ivec3 offset;

	// quality level 3: do a 3x3x3 search
	if (Quality == 3)
	{
		for (offset.x = -1; offset.x <= 1; ++offset.x)
		{
			for (offset.y = -1; offset.y <= 1; ++offset.y)
			{
				for (offset.z = -1; offset.z <= 1; ++offset.z)
				{
					p = offset + VoronoiCornerSample(NoiseTileWrap(iv2 + offset, bTiling, RepeatSize), Quality);
					mindist = VoronoiCompare(mindist, iv2 + p, fv2 - p, bDistanceOnly);
				}
			}
		}
	}

	// everybody else searches a base 2x2x2 neighborhood
	else
	{
		for (offset.x = 0; offset.x <= 1; ++offset.x)
		{
			for (offset.y = 0; offset.y <= 1; ++offset.y)
			{
				for (offset.z = 0; offset.z <= 1; ++offset.z)
				{
					p = offset + VoronoiCornerSample(NoiseTileWrap(iv + offset, bTiling, RepeatSize), Quality);
					mindist = VoronoiCompare(mindist, iv + p, fv - p, bDistanceOnly);

					// quality level 2, do extra set of points, offset by half a cell
					if (Quality == 2)
					{
						// 467 is just an offset to a different area in the random number field to avoid similar neighbor artifacts
						p = offset + VoronoiCornerSample(NoiseTileWrap(iv2 + offset, bTiling, RepeatSize) + 467, Quality);
						mindist = VoronoiCompare(mindist, iv2 + p, fv2 - p, bDistanceOnly);
					}
				}
			}
		}
	}

	// quality level 4: add extra sets of four cells in each direction
	if (Quality >= 4)
	{
		for (offset.x = -1; offset.x <= 2; offset.x += 3)
		{
			for (offset.y = 0; offset.y <= 1; ++offset.y)
			{
				for (offset.z = 0; offset.z <= 1; ++offset.z)
				{
					// along x axis
					p = offset.xyz + VoronoiCornerSample(NoiseTileWrap(iv + offset.xyz, bTiling, RepeatSize), Quality);
					mindist = VoronoiCompare(mindist, iv + p, fv - p, bDistanceOnly);

					// along y axis
					p = offset.yzx + VoronoiCornerSample(NoiseTileWrap(iv + offset.yzx, bTiling, RepeatSize), Quality);
					mindist = VoronoiCompare(mindist, iv + p, fv - p, bDistanceOnly);

					// along z axis
					p = offset.zxy + VoronoiCornerSample(NoiseTileWrap(iv + offset.zxy, bTiling, RepeatSize), Quality);
					mindist = VoronoiCompare(mindist, iv + p, fv - p, bDistanceOnly);
				}
			}
		}
	}

	// transform squared distance to real distance
	return vec4(mindist.xyz, sqrt(mindist.w));
}


#endif
