/*! @file VisualGeom.h
 *  @brief Class representing spatial inertia tensors
 *
 */

#ifndef VISUAL_GEOM_H
#define VISUAL_GEOM_H

#include <cppTypes.h>
#include <mujoco/mjvisualize.h>
/*!
 * Visual Geom
 */
template <typename T = float>
class VisualGeom {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisualGeom() {
    static_assert(std::is_same<T, float>::value, "Mujoco VisualGeom only supports float type");
  }
  VisualGeom(Vec3<T>& pos, Vec3<T>& size, Vec4<T>& rgba, Vec9<T>& mat, mjtGeom& geomType, bool is_static = false)
        : _pos(pos), _size(size), _rgba(rgba), _mat(mat), _type(geomType), _is_static(is_static) {}
  mjtGeom _type;
  Vec3<T> _pos{0, 0, 0};
  Vec3<T> _size{0.1, 0.1, 0.1};
  Vec4<T> _rgba{1.0, 0.0, 0.0, 1.0};
  Vec9<T> _mat{1, 0, 0, 0, 1, 0, 0, 0, 1};
  bool _is_static{false};
};

#endif  // VISUAL_GEOM_H
