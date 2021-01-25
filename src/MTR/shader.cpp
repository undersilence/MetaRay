#include "MTR/shader.h"
#include "MTR/math_defs.h"
#include "MTR/softras/shader.h"
#include <iostream>

template <class A2V, class V2F, class OutFormat>
void SimpleShader<A2V, V2F, OutFormat>::test() {
  printf("hello from simple shader\n");
}

template <class A2V, class V2F, class OutFormat>
V2F SimpleShader<A2V, V2F, OutFormat>::vertex_shader(const A2V &a2f) {
  printf("hello from vertex shader\n");
  return V2F();
}

template <class A2V, class V2F, class OutFormat>
OutFormat SimpleShader<A2V, V2F, OutFormat>::fragment_shader(const V2F &v2f) {
  printf("hello from fragment shader\n");
  return OutFormat();
}