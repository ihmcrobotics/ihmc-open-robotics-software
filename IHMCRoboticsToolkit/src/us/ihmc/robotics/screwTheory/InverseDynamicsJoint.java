package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.kinematics.CommonJoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract interface InverseDynamicsJoint extends CommonJoint
{
   public static int maxDoF = 6;

   public abstract ReferenceFrame getFrameBeforeJoint();

   public abstract ReferenceFrame getFrameAfterJoint();

   public abstract void getJointTwist(Twist twistToPack);

   public abstract void getSuccessorTwist(Twist twistToPack);

   public abstract void getPredecessorTwist(Twist twistToPack);

   public abstract void getJointAcceleration(SpatialAccelerationVector accelerationToPack);

   public abstract void getSuccessorAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void getDesiredJointAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void getDesiredSuccessorAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void getDesiredPredecessorAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart);

   public abstract void getTauMatrix(DenseMatrix64F matrix);

   public abstract void getVelocityMatrix(DenseMatrix64F matrix, int rowStart);

   public abstract void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart);

   public abstract void setDesiredAccelerationToZero();

   public abstract void setConfiguration(DenseMatrix64F matrix, int rowStart);

   public abstract void setVelocity(DenseMatrix64F jointVelocity, int i);

   public abstract void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart);

   public abstract void setSuccessor(RigidBody successor);

   public abstract GeometricJacobian getMotionSubspace();

   public abstract void updateMotionSubspace();

   public abstract RigidBody getPredecessor();

   public abstract RigidBody getSuccessor();

   public abstract void setTorqueFromWrench(Wrench jointWrench);

   public abstract String getName();

   public abstract void updateFramesRecursively();

   public abstract int getDegreesOfFreedom();

   public abstract int getConfigurationMatrixSize();

   public abstract void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint);

   public abstract void setQddDesired(InverseDynamicsJoint originalJoint);
   
   public abstract void calculateJointStateChecksum(GenericCRC32 checksum);
   public abstract void calculateJointDesiredChecksum(GenericCRC32 checksum);
}
