#pragma once

#include "Eigen/Dense"

class RigidBodyTransform
{
   private:
      Eigen::Matrix4d matrix;

      int _id;

   public:
      RigidBodyTransform();

      RigidBodyTransform(const RigidBodyTransform& transform);

      RigidBodyTransform(RigidBodyTransform&& transform);

      RigidBodyTransform(Eigen::Matrix4d matrix);

      RigidBodyTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation);

      void SetToInverse();

      void SetToIdentity();

      RigidBodyTransform GetInverse() const;

      const Eigen::Matrix4d& GetMatrix() const;

      int GetID() const {return _id;};

      void SetID(int id) {_id = id;};

      void SetMatrix(const Eigen::Matrix4d& matrix);

      void MultiplyLeft(const RigidBodyTransform& transform);

      void MultiplyRight(const RigidBodyTransform& transform);

      Eigen::Vector3d TransformVector(const Eigen::Vector3d& vector);

      void print();

      void SetRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      void SetAnglesAndTranslation(const Eigen::Vector3d& rotation, const Eigen::Vector3d& translation);

      void SetQuaternionAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation);

      Eigen::Vector3d GetTranslation() const;

      Eigen::Matrix3d GetRotation() const;

      Eigen::Quaterniond GetQuaternion() const;

      void RotateX(float angleRad);

      void RotateY(float angleRad);

      void RotateZ(float angleRad);

      void Rotate(float rad, Eigen::Vector3d axis);
};
