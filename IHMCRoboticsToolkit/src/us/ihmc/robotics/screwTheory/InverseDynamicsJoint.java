package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.kinematics.CommonJoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract interface InverseDynamicsJoint extends CommonJoint
{
   public static int maxDoF = 6;

   public abstract ReferenceFrame getFrameBeforeJoint();

   public abstract ReferenceFrame getFrameAfterJoint();

   public abstract void packJointTwist(Twist twistToPack);

   public abstract void packSuccessorTwist(Twist twistToPack);

   public abstract void packPredecessorTwist(Twist twistToPack);

   public abstract void packJointAcceleration(SpatialAccelerationVector accelerationToPack);

   public abstract void packSuccessorAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void packDesiredJointAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void packDesiredSuccessorAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void packDesiredPredecessorAcceleration(SpatialAccelerationVector jointAcceleration);

   public abstract void packConfigurationMatrix(DenseMatrix64F matrix, int rowStart);

   public abstract void packTauMatrix(DenseMatrix64F matrix);

   public abstract void packVelocityMatrix(DenseMatrix64F matrix, int rowStart);

   public abstract void packDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart);

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
