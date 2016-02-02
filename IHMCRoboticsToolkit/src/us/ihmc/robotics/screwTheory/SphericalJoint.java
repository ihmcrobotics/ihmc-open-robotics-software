package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SphericalJoint extends AbstractInverseDynamicsJoint
{
   private final SixDoFJointReferenceFrame afterJointFrame;
   private final Quat4d jointRotation = new Quat4d();
   private final FrameVector jointAngularVelocity;
   private final FrameVector jointAngularAcceleration;
   private final FrameVector jointAngularAccelerationDesired;
   private FrameVector jointTorque;

   public SphericalJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame)
   {
      super(name, predecessor, beforeJointFrame);
      this.afterJointFrame = new SixDoFJointReferenceFrame(name, beforeJointFrame);
      this.jointAngularVelocity = new FrameVector(afterJointFrame);
      this.jointAngularAcceleration = new FrameVector(afterJointFrame);
      this.jointAngularAccelerationDesired = new FrameVector(afterJointFrame);
   }

   public ReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   public void getJointTwist(Twist twistToPack)
   {
      twistToPack.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      twistToPack.setAngularPart(jointAngularVelocity.getVector());
   }

   public void getJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      accelerationToPack.setAngularPart(jointAngularAcceleration.getVector());
   }

   public void getDesiredJointAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      jointAcceleration.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAcceleration.setAngularPart(jointAngularAccelerationDesired.getVector());
   }

   public void getTauMatrix(DenseMatrix64F matrix)
   {
      matrix.set(0, 0, jointTorque.getX());
      matrix.set(1, 0, jointTorque.getY());
      matrix.set(2, 0, jointTorque.getZ());
   }

   public void getVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointAngularVelocity.getX());
      matrix.set(rowStart + 1, 0, jointAngularVelocity.getY());
      matrix.set(rowStart + 2, 0, jointAngularVelocity.getZ());
   }

   public void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointAngularAccelerationDesired.getX());
      matrix.set(rowStart + 1, 0, jointAngularAccelerationDesired.getY());
      matrix.set(rowStart + 2, 0, jointAngularAccelerationDesired.getZ());
   }

   public void setDesiredAccelerationToZero()
   {
      jointAngularAccelerationDesired.setToZero(jointAngularAccelerationDesired.getReferenceFrame());
   }

   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;
      setMotionSubspace();

      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      this.jointTorque = new FrameVector(successorFrame);
   }

   public void updateMotionSubspace()
   {
      // empty
   }

   public void setTorqueFromWrench(Wrench jointWrench)
   {
      jointWrench.getBodyFrame().checkReferenceFrameMatch(successor.getBodyFixedFrame());
      jointWrench.getExpressedInFrame().checkReferenceFrameMatch(jointTorque.getReferenceFrame());
      jointTorque.set(jointWrench.getAngularPart());
   }

   public int getDegreesOfFreedom()
   {
      return 3;
   }

   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      this.jointAngularAccelerationDesired.setX(matrix.get(rowStart + 0, 0));
      this.jointAngularAccelerationDesired.setY(matrix.get(rowStart + 1, 0));
      this.jointAngularAccelerationDesired.setZ(matrix.get(rowStart + 2, 0));
   }

   public void setRotation(double yaw, double pitch, double roll)
   {
      RotationTools.convertYawPitchRollToQuaternion(yaw, pitch, roll, jointRotation);
      this.afterJointFrame.setRotation(jointRotation);
   }

   public void setRotation(Quat4d jointRotation)
   {
      this.jointRotation.set(jointRotation);
      this.afterJointFrame.setRotation(jointRotation);
   }

   public void setRotation(Matrix3d jointRotation)
   {
      RotationTools.convertMatrixToQuaternion(jointRotation, this.jointRotation);

      // DON'T USE THIS: the method in Quat4d is flawed and doesn't work for some rotation matrices!
//      this.jointRotation.set(jointRotation);
      this.afterJointFrame.setRotation(this.jointRotation);
   }

   public void setJointAngularVelocity(FrameVector jointAngularVelocity)
   {
      this.jointAngularVelocity.set(jointAngularVelocity);
   }

   public void setAcceleration(FrameVector jointAngularAcceleration)
   {
      this.jointAngularAcceleration.set(jointAngularAcceleration);
   }

   public void setDesiredAcceleration(FrameVector jointAngularAccelerationDesired)
   {
      this.jointAngularAccelerationDesired.set(jointAngularAccelerationDesired);
   }

   public void setJointTorque(FrameVector jointTorque)
   {
      this.jointTorque.set(jointTorque);
   }

   public void packRotation(Quat4d rotationToPack)
   {
      rotationToPack.set(jointRotation);
   }

   public void packRotation(Matrix3d rotationToPack)
   {
      rotationToPack.set(jointRotation);
   }

   public void packRotation(double[] yawPitchRoll)
   {
      RotationTools.convertQuaternionToYawPitchRoll(jointRotation, yawPitchRoll);
   }

   public void packJointTorque(FrameVector jointTorqueToPack)
   {
      jointTorqueToPack.setIncludingFrame(jointTorque);
   }

   private void setMotionSubspace()
   {
      int nDegreesOfFreedom = getDegreesOfFreedom();

      ArrayList<Twist> unitTwistsInBodyFrame = new ArrayList<Twist>();
      RigidBodyTransform identity = new RigidBodyTransform();

      ReferenceFrame[] intermediateFrames = new ReferenceFrame[nDegreesOfFreedom - 1];
      ReferenceFrame previousFrame = afterJointFrame;
      for (int i = 0; i < nDegreesOfFreedom - 1; i++) // from afterJointFrame to beforeJointFrame
      {
         int index = intermediateFrames.length - i - 1;
         ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("intermediateFrame" + index, previousFrame, identity);
         intermediateFrames[index] = frame;
         previousFrame = frame;
      }

      previousFrame = beforeJointFrame;

      for (int i = 0; i < nDegreesOfFreedom; i++) // from beforeJointFrame to afterJointFrame
      {
         ReferenceFrame frame;
         if (i < nDegreesOfFreedom - 1)
         {
            frame = intermediateFrames[i];
         }
         else
         {
            frame = afterJointFrame;
         }

         DenseMatrix64F twistMatrix = new DenseMatrix64F(Twist.SIZE, 1);
         int angularPartStartIndex = 0;
         twistMatrix.set(angularPartStartIndex + i, 0, 1.0);
         Twist twist = new Twist(frame, previousFrame, frame, twistMatrix);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      this.motionSubspace = new GeometricJacobian(this, unitTwistsInBodyFrame, afterJointFrame);
      this.motionSubspace.compute();
   }

   public void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      MatrixTools.insertQuat4dIntoEJMLVector(matrix, jointRotation, rowStart);
   }

   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      MatrixTools.extractQuat4dFromEJMLVector(jointRotation, matrix, rowStart);
      this.afterJointFrame.setRotation(jointRotation);
   }

   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      jointAngularVelocity.setX(matrix.get(rowStart + 0, 0));
      jointAngularVelocity.setY(matrix.get(rowStart + 1, 0));
      jointAngularVelocity.setZ(matrix.get(rowStart + 2, 0));
   }


   public int getConfigurationMatrixSize()
   {
      return RotationTools.QUATERNION_SIZE;
   }
   
   private SphericalJoint checkAndGetAsSphericalJoint(InverseDynamicsJoint originalJoint)
   {
      if (originalJoint instanceof SphericalJoint)
      {
         return (SphericalJoint) originalJoint;
      }
      else
      {
         throw new RuntimeException("Cannot set " + getClass().getSimpleName() + " to " + originalJoint.getClass().getSimpleName());
      }
   }
   
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
         SphericalJoint originalSphericalJoint = checkAndGetAsSphericalJoint(originalJoint);
         setRotation(originalSphericalJoint.jointRotation);
         jointAngularVelocity.set(originalSphericalJoint.jointAngularVelocity.getVector());
         jointAngularAcceleration.set(originalSphericalJoint.jointAngularAcceleration.getVector());
   }

   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      SphericalJoint originalSphericalJoint = checkAndGetAsSphericalJoint(originalJoint);
      jointAngularAccelerationDesired.set(originalSphericalJoint.jointAngularAccelerationDesired.getVector()); 
   }

   public void calculateJointStateChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointRotation);
      checksum.update(jointAngularVelocity.getVector());
      checksum.update(jointAngularAcceleration.getVector());
   }

   public void calculateJointDesiredChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointAngularAccelerationDesired.getVector());
   }
}
