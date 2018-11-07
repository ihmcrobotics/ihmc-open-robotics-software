package us.ihmc.mecano.multiBodySystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class SphericalJoint extends AbstractInverseDynamicsJoint
{
   private final Quaternion jointRotation = new Quaternion();
   private final FrameVector3D jointAngularVelocity;
   private final FrameVector3D jointAngularAcceleration;
   private final FrameVector3D jointAngularAccelerationDesired;
   private FrameVector3D jointTorque;

   private List<Twist> unitTwists;

   public SphericalJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
   {
      super(name, predecessor, transformToParent);
      this.jointAngularVelocity = new FrameVector3D(afterJointFrame);
      this.jointAngularAcceleration = new FrameVector3D(afterJointFrame);
      this.jointAngularAccelerationDesired = new FrameVector3D(afterJointFrame);
   }

   @Override
   protected void updateJointTransform(RigidBodyTransform jointTransform)
   {
      jointTransform.setRotationAndZeroTranslation(jointRotation);
   }

   public QuaternionBasics getJointOrientation()
   {
      return jointRotation;
   }

   public FixedFrameVector3DBasics getJointAngularVelocity()
   {
      return jointAngularVelocity;
   }

   public FixedFrameVector3DBasics getJointAngularAcceleration()
   {
      return jointAngularAcceleration;
   }

   @Override
   public void getJointTau(int rowStart, DenseMatrix64F matrix)
   {
      matrix.set(0, 0, jointTorque.getX());
      matrix.set(1, 0, jointTorque.getY());
      matrix.set(2, 0, jointTorque.getZ());
   }

   @Override
   public void getJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      matrix.set(rowStart + 0, 0, jointAngularVelocity.getX());
      matrix.set(rowStart + 1, 0, jointAngularVelocity.getY());
      matrix.set(rowStart + 2, 0, jointAngularVelocity.getZ());
   }

   @Override
   public void getJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      matrix.set(rowStart + 0, 0, jointAngularAccelerationDesired.getX());
      matrix.set(rowStart + 1, 0, jointAngularAccelerationDesired.getY());
      matrix.set(rowStart + 2, 0, jointAngularAccelerationDesired.getZ());
   }

   @Override
   public void setJointAccelerationToZero()
   {
      jointAngularAccelerationDesired.setToZero(jointAngularAccelerationDesired.getReferenceFrame());
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      setMotionSubspace();

      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      this.jointTorque = new FrameVector3D(successorFrame);
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void setJointWrench(Wrench jointWrench)
   {
      jointWrench.getBodyFrame().checkReferenceFrameMatch(successor.getBodyFixedFrame());
      jointWrench.getReferenceFrame().checkReferenceFrameMatch(jointTorque.getReferenceFrame());
      jointTorque.set(jointWrench.getAngularPart());
   }

   @Override
   public int getDegreesOfFreedom()
   {
      return 3;
   }

   @Override
   public void setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      this.jointAngularAccelerationDesired.setX(matrix.get(rowStart + 0, 0));
      this.jointAngularAccelerationDesired.setY(matrix.get(rowStart + 1, 0));
      this.jointAngularAccelerationDesired.setZ(matrix.get(rowStart + 2, 0));
   }

   @Override
   public void setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      jointTorque.set(matrix.get(rowStart + 0), matrix.get(rowStart + 1), matrix.get(rowStart + 2));
   }

   public void setRotation(double yaw, double pitch, double roll)
   {
      jointRotation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setRotation(QuaternionReadOnly jointRotation)
   {
      this.jointRotation.set(jointRotation);
   }

   public void setRotation(RotationMatrixReadOnly jointRotation)
   {
      this.jointRotation.set(jointRotation);
   }

   public void setJointAngularVelocity(FrameVector3D jointAngularVelocity)
   {
      this.jointAngularVelocity.set(jointAngularVelocity);
   }

   public void setAcceleration(FrameVector3D jointAngularAcceleration)
   {
      this.jointAngularAcceleration.set(jointAngularAcceleration);
   }

   public void setDesiredAcceleration(FrameVector3D jointAngularAccelerationDesired)
   {
      this.jointAngularAccelerationDesired.set(jointAngularAccelerationDesired);
   }

   public void setJointTorque(FrameVector3D jointTorque)
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

   public void getJointTorque(FrameVector3D jointTorqueToPack)
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
   public void getJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      jointRotation.get(rowStart, matrix);
   }

   @Override
   public void setJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      jointRotation.set(rowStart, matrix);
   }

   @Override
   public void setJointVelocity(int rowStart, DenseMatrix64F matrix)
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

   private SphericalJoint checkAndGetAsSphericalJoint(JointBasics originalJoint)
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
}
