package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SphericalJoint extends AbstractInverseDynamicsJoint
{
   private final FloatingInverseDynamicsJointReferenceFrame afterJointFrame;
   private final Quaternion jointRotation = new Quaternion();
   private final FrameVector jointAngularVelocity;
   private final FrameVector jointAngularAcceleration;
   private final FrameVector jointAngularAccelerationDesired;
   private FrameVector jointTorque;

   private List<Twist> unitTwists;

   public SphericalJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame)
   {
      super(name, predecessor, beforeJointFrame);
      this.afterJointFrame = new FloatingInverseDynamicsJointReferenceFrame(name, beforeJointFrame);
      this.jointAngularVelocity = new FrameVector(afterJointFrame);
      this.jointAngularAcceleration = new FrameVector(afterJointFrame);
      this.jointAngularAccelerationDesired = new FrameVector(afterJointFrame);
   }

   @Override
   public ReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   @Override
   public void getJointTwist(Twist twistToPack)
   {
      twistToPack.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      twistToPack.setAngularPart(jointAngularVelocity.getVector());
   }

   @Override
   public void getJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      accelerationToPack.setAngularPart(jointAngularAcceleration.getVector());
   }

   @Override
   public void getDesiredJointAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      jointAcceleration.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAcceleration.setAngularPart(jointAngularAccelerationDesired.getVector());
   }

   @Override
   public void getTauMatrix(DenseMatrix64F matrix)
   {
      matrix.set(0, 0, jointTorque.getX());
      matrix.set(1, 0, jointTorque.getY());
      matrix.set(2, 0, jointTorque.getZ());
   }

   @Override
   public void getVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointAngularVelocity.getX());
      matrix.set(rowStart + 1, 0, jointAngularVelocity.getY());
      matrix.set(rowStart + 2, 0, jointAngularVelocity.getZ());
   }

   @Override
   public void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointAngularAccelerationDesired.getX());
      matrix.set(rowStart + 1, 0, jointAngularAccelerationDesired.getY());
      matrix.set(rowStart + 2, 0, jointAngularAccelerationDesired.getZ());
   }

   @Override
   public void setDesiredAccelerationToZero()
   {
      jointAngularAccelerationDesired.setToZero(jointAngularAccelerationDesired.getReferenceFrame());
   }

   @Override
   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;
      setMotionSubspace();

      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      this.jointTorque = new FrameVector(successorFrame);
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void setTorqueFromWrench(Wrench jointWrench)
   {
      jointWrench.getBodyFrame().checkReferenceFrameMatch(successor.getBodyFixedFrame());
      jointWrench.getExpressedInFrame().checkReferenceFrameMatch(jointTorque.getReferenceFrame());
      jointTorque.set(jointWrench.getAngularPart());
   }

   @Override
   public int getDegreesOfFreedom()
   {
      return 3;
   }

   @Override
   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      this.jointAngularAccelerationDesired.setX(matrix.get(rowStart + 0, 0));
      this.jointAngularAccelerationDesired.setY(matrix.get(rowStart + 1, 0));
      this.jointAngularAccelerationDesired.setZ(matrix.get(rowStart + 2, 0));
   }

   public void setRotation(double yaw, double pitch, double roll)
   {
      jointRotation.setYawPitchRoll(yaw, pitch, roll);
      this.afterJointFrame.setRotation(jointRotation);
   }

   public void setRotation(QuaternionReadOnly jointRotation)
   {
      this.jointRotation.set(jointRotation);
      this.afterJointFrame.setRotation(jointRotation);
   }

   public void setRotation(RotationMatrixReadOnly jointRotation)
   {
      this.jointRotation.set(jointRotation);
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

   public void getRotation(QuaternionBasics rotationToPack)
   {
      rotationToPack.set(jointRotation);
   }

   public void getRotation(RotationMatrix rotationToPack)
   {
      rotationToPack.set(jointRotation);
   }

   public void getRotation(double[] yawPitchRollToPack)
   {
      jointRotation.getYawPitchRoll(yawPitchRollToPack);
   }

   public void getJointTorque(FrameVector jointTorqueToPack)
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

      unitTwists = Collections.unmodifiableList(unitTwistsInBodyFrame);

      this.motionSubspace = new GeometricJacobian(this, afterJointFrame);
      this.motionSubspace.compute();
   }

   @Override
   public void getUnitTwist(int dofIndex, Twist unitTwistToPack)
   {
      if (dofIndex < 0 || dofIndex >= getDegreesOfFreedom())
         throw new ArrayIndexOutOfBoundsException("Illegal index: " + dofIndex + ", was expecting dofIndex in [0, " + getDegreesOfFreedom() + "[.");
      unitTwistToPack.set(unitTwists.get(dofIndex));
   }

   @Override
   public void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      jointRotation.get(rowStart, matrix);
   }

   @Override
   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      jointRotation.set(rowStart, matrix);
      this.afterJointFrame.setRotation(jointRotation);
   }

   @Override
   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      jointAngularVelocity.setX(matrix.get(rowStart + 0, 0));
      jointAngularVelocity.setY(matrix.get(rowStart + 1, 0));
      jointAngularVelocity.setZ(matrix.get(rowStart + 2, 0));
   }

   @Override
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

   @Override
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
      SphericalJoint originalSphericalJoint = checkAndGetAsSphericalJoint(originalJoint);
      setRotation(originalSphericalJoint.jointRotation);
      jointAngularVelocity.set(originalSphericalJoint.jointAngularVelocity.getVector());
      jointAngularAcceleration.set(originalSphericalJoint.jointAngularAcceleration.getVector());
   }

   @Override
   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      SphericalJoint originalSphericalJoint = checkAndGetAsSphericalJoint(originalJoint);
      jointAngularAccelerationDesired.set(originalSphericalJoint.jointAngularAccelerationDesired.getVector());
   }

   @Override
   public void calculateJointStateChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointRotation);
      checksum.update(jointAngularVelocity.getVector());
      checksum.update(jointAngularAcceleration.getVector());
   }

   @Override
   public void calculateJointDesiredChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointAngularAccelerationDesired.getVector());
   }
}
