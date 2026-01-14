#version 460 core

// VertexFormat.hpp

layout(location = 0) in vec3 a_Position;
#ifdef HAS_COLOR
layout(location = 1) in vec3 a_Color0;
#endif
#ifdef HAS_NORMAL
layout(location = 2) in vec3 a_Normal;
#endif
#ifdef HAS_TEXCOORD0
layout(location = 3) in vec2 a_TexCoord0;
#  ifdef HAS_TANGENTS
layout(location = 5) in vec3 a_Tangent;
layout(location = 6) in vec3 a_Bitangent;
#  endif
#endif
#ifdef HAS_TEXCOORD1
layout(location = 4) in vec2 a_TexCoord1;
#endif
#ifdef IS_SKINNED
layout(location = 7) in ivec4 a_Joints;
layout(location = 8) in vec4 a_Weights;
#endif

struct Transform {
  mat4 modelMatrix;
  mat4 normalMatrix;
  mat4 modelViewProjMatrix;
};
layout(location = 0) uniform Transform u_Transform;

#ifdef IS_SKINNED
struct Bones {
  mat4 bones[128];
};
layout(location = 15) uniform Bones u_Bones;
#endif 

out gl_PerVertex { vec4 gl_Position; };

layout(location = 0) out VertexData {
  vec4 fragPos; // world-space
#ifdef HAS_NORMAL
#  ifdef HAS_TANGENTS
  mat3 TBN; // tangent-space -> world-space
#  else
  vec3 normal;
#  endif
#endif
#ifdef HAS_TEXCOORD0
  vec2 texCoord0;
#endif
#ifdef HAS_TEXCOORD1
  vec2 texCoord1;
#endif
#ifdef HAS_COLOR
  vec3 color;
#endif
}
vs_out;

#pragma USER_SAMPLERS

struct Input
{
    vec3 position;
    vec3 normal;
};

void main() {

  Input vs_input;
  vs_input.position = a_Position;
#ifdef HAS_NORMAL
  vs_input.normal = a_Normal;
#endif

#pragma USER_CODE

  vec4 worldPos;
  mat4 m;
#ifdef IS_SKINNED
  m = u_Bones.bones[a_Joints[0]] * a_Weights[0] +
           u_Bones.bones[a_Joints[1]] * a_Weights[1] +
           u_Bones.bones[a_Joints[2]] * a_Weights[2] +
           u_Bones.bones[a_Joints[3]] * a_Weights[3];
  worldPos = m * vec4(vs_input.position, 1.0);
#else
  worldPos = vec4(vs_input.position, 1.0);
#endif 

  vs_out.fragPos = u_Transform.modelMatrix * worldPos;

#ifdef HAS_NORMAL
# ifdef IS_SKINNED
  const mat3 normalMatrix = mat3(u_Transform.normalMatrix) * mat3(m);
#  else
  const mat3 normalMatrix = mat3(u_Transform.normalMatrix);
#  endif
  const vec3 N = normalize(normalMatrix * vs_input.normal);
#  ifdef HAS_TANGENTS
  vec3 T = normalize(normalMatrix * a_Tangent);
  T = normalize(T - dot(T, N) * N);
  vec3 B = normalize(normalMatrix * a_Bitangent);
  vs_out.TBN = mat3(T, B, N);
#  else
  vs_out.normal = N;
#  endif
#endif

#ifdef HAS_TEXCOORD0
  vs_out.texCoord0 = a_TexCoord0;
#endif
#ifdef HAS_TEXCOORD1
  vs_out.texCoord1 = a_TexCoord1;
#endif

#ifdef HAS_COLOR
  vs_out.color = a_Color0;
#endif


  gl_Position = u_Transform.modelViewProjMatrix * worldPos;
}
