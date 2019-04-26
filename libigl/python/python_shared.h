#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/complex.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "py_doc.h"

#include <Eigen/Dense>

template<typename Scalar>
void assert_is_VectorX(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if (v.cols() != 1)
    throw std::runtime_error(name + " must be a column vector.");
}

template<typename Scalar>
void assert_is_RowVectorX(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if (v.rows() != 1)
    throw std::runtime_error(name + " must be a row vector.");
}

template<typename Scalar>
void assert_is_Vector2(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if ((v.cols() != 1) || (v.rows() != 2))
    throw std::runtime_error(name + " must be a column vector with 2 entries.");
}

template<typename Scalar>
void assert_is_RowVector2(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if ((v.cols() != 2) || (v.rows() != 1))
    throw std::runtime_error(name + " must be a row vector with 2 entries.");
}

template<typename Scalar>
void assert_is_Vector3(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if ((v.cols() != 1) || (v.rows() != 3))
    throw std::runtime_error(name + " must be a column vector with 3 entries.");
}

template<typename Scalar>
void assert_is_RowVector3(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if ((v.cols() != 3) || (v.rows() != 1))
    throw std::runtime_error(name + " must be a row vector with 3 entries.");
}

template<typename Scalar>
void assert_is_Vector4(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if ((v.cols() != 1) || (v.rows() != 4))
    throw std::runtime_error(name + " must be a column vector with 4 entries.");
}

template<typename Scalar>
void assert_is_RowVector4(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if ((v.cols() != 4) || (v.rows() != 1))
    throw std::runtime_error(name + " must be a row vector with 4 entries.");
}

template<typename Scalar>
void assert_is_Matrix4(const std::string name, const Eigen::PlainObjectBase<Scalar>& v)
{
  if (v.size() == 0)
    return;

  if ((v.cols() != 4) || (v.rows() != 4))
    throw std::runtime_error(name + " must be a 4x4 matrix.");
}



namespace py = pybind11;
