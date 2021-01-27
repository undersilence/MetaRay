#include "MTR/softras/shader.h"

template <class vecXf>
static vecXf bc_interpolate_attr(const vecXf &a, const vecXf &b, const vecXf &c,
                                 const vec3f &weight) {
  vecXf result;
  for (int i = 0; i < a.rows(); i++) {
    result(i) = weight.dot(vec3f(a(i), b(i), c(i)));
  }
  return result;
}

vec4f SoftShader::vertex_shader(A2V &a2v, V2F &v2f) {
  vec4f position;
  position << a2v.position, 1.0f;
  v2f.color = position;
  // v2f.view_pos = (a2v.model * a2v.view * position).head<3>();
  return project * model * view * position;
}

vec4f SoftShader::fragment_shader(V2F &v2f) { return v2f.color; }

V2F SoftShader::interpolate_attr(V2F *v2f, vec3f bc_weight) {
  V2F result;
  result.color =
      bc_interpolate_attr(v2f[0].color, v2f[1].color, v2f[2].color, bc_weight);
  return result;
}

// vec4f simple_vertex_shader(A2V &a2v, V2F &v2f) {
//   vec4f position;
//   position << a2v.position, 1.0f;
//   v2f.color = position;
//   // v2f.view_pos = (a2v.model * a2v.view * position).head<3>();
//   return a2v.project * a2v.model * a2v.view * position;
// }

// vec4f simple_fragment_shader(V2F &v2f) { return v2f.color; }
