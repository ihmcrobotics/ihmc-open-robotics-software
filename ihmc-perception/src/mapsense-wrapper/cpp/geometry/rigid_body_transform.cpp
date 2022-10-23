#include "rigid_body_transform.h"
#include <utility>
#include <iostream>

RigidBodyTransform::RigidBodyTransform()
{
   matrix = Eigen::Matrix4d::Identity();
}

RigidBodyTransform::RigidBodyTransform(const RigidBodyTransform& transform) : matrix(transform.matrix)
{
   _id = transform.GetID();
}

RigidBodyTransform::RigidBodyTransform(RigidBodyTransform&& transform) : matrix(std::move(transform.matrix))
{
   _id = transform.GetID();
}

RigidBodyTransform::RigidBodyTransform(Eigen::Matrix4d mat)
{
   matrix = std::move(mat);
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation)
{

   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
   matrix.setIdentity();
   matrix.block<3, 3>(0, 0) = rotation;
   matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::SetToIdentity()
{
   matrix = Eigen::Matrix4d::Identity();
}

void RigidBodyTransform::SetToInverse()
{
   matrix.block<3, 1>(0, 3) = -matrix.block<3, 3>(0, 0).transpose() * matrix.block<3, 1>(0, 3);
   matrix.block<3, 3>(0, 0) = matrix.block<3, 3>(0, 0).transpose();
}

RigidBodyTransform RigidBodyTransform::GetInverse() const
{
   RigidBodyTransform transformToPack;
   transformToPack.matrix.block<3, 3>(0, 0) = matrix.block<3, 3>(0, 0).transpose();
   transformToPack.matrix.block<3, 1>(0, 3) = -matrix.block<3, 3>(0, 0).transpose() * matrix.block<3, 1>(0, 3);
   return transformToPack;
}

void RigidBodyTransform::SetAnglesAndTranslation(const Eigen::Vector3d& angles, const Eigen::Vector3d& translation)
{
   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) angles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) angles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) angles.z(), Eigen::Vector3d::UnitZ());
   matrix.setIdentity();
   matrix.block<3, 3>(0, 0) = rotation;
   matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::SetQuaternionAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation)
{
   matrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();
   matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::SetRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
   matrix.setIdentity();
   matrix.block<3, 3>(0, 0) = rotation;
   matrix.block<3, 1>(0, 3) = translation;
}

const Eigen::Matrix4d& RigidBodyTransform::GetMatrix() const
{
   return matrix;
}

void RigidBodyTransform::SetMatrix(const Eigen::Matrix4d& matrix)
{
   this->matrix = matrix;
}

void RigidBodyTransform::MultiplyLeft(const RigidBodyTransform& transform)
{
   matrix = transform.matrix * matrix;
}

void RigidBodyTransform::MultiplyRight(const RigidBodyTransform& transform)
{
   matrix = matrix * transform.matrix;
}

Eigen::Vector3d RigidBodyTransform::TransformVector(const Eigen::Vector3d& vector)
{
   return matrix.block<3, 3>(0, 0) * vector + matrix.block<3, 1>(0, 3);
}

void RigidBodyTransform::print()
{
   std::cout << matrix << std::endl;
}

Eigen::Vector3d RigidBodyTransform::GetTranslation() const
{
   return matrix.block<3, 1>(0, 3);
}

Eigen::Quaterniond RigidBodyTransform::GetQuaternion() const
{
   return Eigen::Quaterniond(matrix.block<3, 3>(0, 0));
}

Eigen::Matrix3d RigidBodyTransform::GetRotation() const
{
   return matrix.block<3, 3>(0, 0);
}

void RigidBodyTransform::RotateX(float rad)
{
   Rotate(rad, Eigen::Vector3d::UnitX());
}

void RigidBodyTransform::RotateY(float rad)
{
   Rotate(rad, Eigen::Vector3d::UnitY());
}

void RigidBodyTransform::RotateZ(float rad)
{
   Rotate(rad, Eigen::Vector3d::UnitZ());
}

void RigidBodyTransform::Rotate(float rad, Eigen::Vector3d axis)
{
   Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
   rotation.block<3,3>(0,0) = Eigen::AngleAxisd(rad, axis).toRotationMatrix();
   matrix =  rotation * matrix ;
}
