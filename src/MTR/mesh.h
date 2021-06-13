#pragma once

#ifndef _METARAY_MESH_
#define _METARAY_MESH_

#include "MTR/math_defs.h"

namespace mtr {
struct _BasicMeshVertex {
  vec3f position;
  vec3f normal;
  vec2f texcoords;
};

temlpate<class Vertex = _BasicMeshVertex> class Mesh {
 public:
  Mesh = default;
  virtual ~Mesh = default;

  load_vertices(const std::vector<Vertex>& _vertices);
  load_indices(const std::vector<unsigned int>& _indices);
  load_textures(const std::vector<Texture2D>& _textures);

 private:
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  // std::vector<Texture2D> textures;
}

}  // namespace mtr
#endif  //_METARAY_MESH_