package us.ihmc.robotics.screwTheory;

import javax.vecmath.*;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;

public class PlanarJoint extends AbstractInverseDynamicsJoint implements FloatingInverseDynamicsJoint
{
   private final FloatingInverseDynamicsJointReferenceFrame afterJointFrame;
   private final Quat4d jointRotation = new Quat4d();
   private final Vector3d jointTranslation = new Vector3d();
   private final Twist jointTwist;
   public final SpatialAccelerationVector jointAcceleration;
   protected final SpatialAccelerationVector jointAccelerationDesired;

   private Wrench successorWrench;

   public PlanarJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame)
   {
      super(name, predecessor, beforeJointFrame);

      this.afterJointFrame = new FloatingInverseDynamicsJointReferenceFrame(name, beforeJointFrame);
      this.jointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame);
      this.jointAcceleration = new SpatialAccelerationVector(afterJointFrame, beforeJointFrame, afterJointFrame);
      this.jointAccelerationDesired = new SpatialAccelerationVector(afterJointFrame, beforeJointFrame, afterJointFrame);
   }

   @Override
   public FloatingInverseDynamicsJointReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void setDesiredAccelerationToZero()
   {
      jointAccelerationDesired.setToZero();
   }

   @Override
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
      PlanarJoint floatingJoint = checkAndGetAsPlanarJoint(originalJoint);

      setPosition(floatingJoint.jointTranslation);
      setRotation(floatingJoint.jointRotation);

      setJointTwist(floatingJoint.jointTwist);
      setAcceleration(floatingJoint.jointAcceleration);
   }

   @Override
   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      PlanarJoint floatingJoint = checkAndGetAsPlanarJoint(originalJoint);

      jointAccelerationDesired.setAngularPart(floatingJoint.jointAccelerationDesired.getAngularPart());
      jointAccelerationDesired.setLinearPart(floatingJoint.jointAccelerationDesired.getLinearPart());
   }

   @Override
   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;
      setMotionSubspace();

      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      this.successorWrench = new Wrench(successorFrame, successorFrame);
   }

   @Override
   public void getJointTwist(Twist twistToPack)
   {
      twistToPack.setToZero(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getExpressedInFrame());

      twistToPack.setAngularPartY(jointTwist.getAngularPartY());
      twistToPack.setLinearPartX(jointTwist.getLinearPartX());
      twistToPack.setLinearPartZ(jointTwist.getLinearPartZ());
   }

   @Override
   public void getJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.setToZero(jointAcceleration.getBodyFrame(), jointAcceleration.getBaseFrame(), jointAcceleration.getExpressedInFrame());

      accelerationToPack.setAngularPartY(jointAcceleration.getAngularPartY());
      accelerationToPack.setLinearPartX(jointAcceleration.getLinearPartX());
      accelerationToPack.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void getDesiredJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.setToZero(jointAccelerationDesired.getBodyFrame(), jointAccelerationDesired.getBaseFrame(), jointAccelerationDesired.getExpressedInFrame());

      accelerationToPack.setAngularPartY(jointAccelerationDesired.getAngularPartY());
      accelerationToPack.setLinearPartX(jointAccelerationDesired.getLinearPartX());
      accelerationToPack.setLinearPartZ(jointAccelerationDesired.getLinearPartZ());
   }

   @Override
   public void getTauMatrix(DenseMatrix64F matrix)
   {
      matrix.set(0, 0, successorWrench.getAngularPartY());
      matrix.set(1, 0, successorWrench.getLinearPartX());
      matrix.set(2, 0, successorWrench.getLinearPartZ());
   }

   @Override
   public void getVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointTwist.getAngularPartY());
      matrix.set(rowStart + 1, 0, jointTwist.getLinearPartX());
      matrix.set(rowStart + 2, 0, jointTwist.getLinearPartZ());
   }

   @Override
   public void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointAccelerationDesired.getAngularPartY());
      matrix.set(rowStart + 1, 0, jointAccelerationDesired.getLinearPartX());
      matrix.set(rowStart + 2, 0, jointAccelerationDesired.getLinearPartZ());
   }

   @Override
   public void setTorqueFromWrench(Wrench jointWrench)
   {
      jointWrench.getBodyFrame().checkReferenceFrameMatch(successor.getBodyFixedFrame());
      jointWrench.setToZero(successor.getBodyFixedFrame(), successorWrench.getExpressedInFrame());
      jointWrench.setAngularPartY(successorWrench.getAngularPartY());
      jointWrench.setLinearPartX(successorWrench.getLinearPartX());
      jointWrench.setLinearPartZ(successorWrench.getLinearPartZ());
      jointWrench.changeFrame(successor.getBodyFixedFrame());
   }

   @Override
   public int getDegreesOfFreedom()
   {
      return 3;
   }

   @Override
   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      jointAccelerationDesired.setToZero();
      jointAccelerationDesired.setAngularPartY(matrix.get(rowStart + 0, 0));
      jointAccelerationDesired.setLinearPartX(matrix.get(rowStart + 1, 0));
      jointAccelerationDesired.setLinearPartZ(matrix.get(rowStart + 2, 0));
   }

   @Override
   public void setJointTorque(DenseMatrix64F matrix, int rowStart)
   {
      successorWrench.setAngularPartY(matrix.get(rowStart + 0));
      successorWrench.setLinearPartX(matrix.get(rowStart + 1));
      successorWrench.setLinearPartZ(matrix.get(rowStart + 2));
   }

   public void setRotation(double yaw, double pitch, double roll)
   {
      RotationTools.convertYawPitchRollToQuaternion(0.0, pitch, 0.0, jointRotation);
      afterJointFrame.setRotation(this.jointRotation);
   }

   private final double[] yawPitchRoll = new double[3];
   public void setRotation(Quat4d jointRotation)
   {
      RotationTools.convertQuaternionToYawPitchRoll(jointRotation, yawPitchRoll);
      RotationTools.convertYawPitchRollToQuaternion(0.0, yawPitchRoll[1], 0.0, this.jointRotation);
      this.afterJointFrame.setRotation(this.jointRotation);
   }

   public void setRotation(double x, double y, double z, double w)
   {
      RotationTools.convertQuaternionToYawPitchRoll(x, y, z, w, yawPitchRoll);
      RotationTools.convertYawPitchRollToQuaternion(0.0, yawPitchRoll[1], 0.0, jointRotation);
      this.afterJointFrame.setRotation(this.jointRotation);
   }

   public void setRotation(Matrix3d jointRotation)
   {
      RotationTools.convertMatrixToYawPitchRoll(jointRotation, yawPitchRoll);
      RotationTools.convertYawPitchRollToQuaternion(0.0, yawPitchRoll[1], 0.0, this.jointRotation);

      // DON'T USE THIS: the method in Quat4d is flawed and doesn't work for some rotation matrices!
      //      this.jointRotation.set(jointRotation);

      this.afterJointFrame.setRotation(this.jointRotation);
   }

   public void setPosition(Tuple3d jointTranslation)
   {
      this.jointTranslation.set(jointTranslation.getX(), 0.0, jointTranslation.getZ());
      afterJointFrame.setTranslation(this.jointTranslation);
   }
   
   public void setPosition(double x, double y, double z)
   {
      jointTranslation.set(x, 0.0, z);
      afterJointFrame.setTranslation(this.jointTranslation);
   }

   public void setJointTwist(Twist jointTwist)
   {
      this.jointTwist.checkReferenceFramesMatch(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getExpressedInFrame());
      this.jointTwist.setAngularPartY(jointTwist.getAngularPartY());
      this.jointTwist.setLinearPartX(jointTwist.getLinearPartX());
      this.jointTwist.setLinearPartZ(jointTwist.getLinearPartZ());
   }

   public void setAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      this.jointAcceleration.checkReferenceFramesMatch(jointAcceleration);
      this.jointAcceleration.setAngularPartY(jointAcceleration.getAngularPartY());
      this.jointAcceleration.setLinearPartX(jointAcceleration.getLinearPartX());
      this.jointAcceleration.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }

   public void setDesiredAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      this.jointAccelerationDesired.checkReferenceFramesMatch(jointAcceleration);
      this.jointAccelerationDesired.setAngularPartY(jointAcceleration.getAngularPartY());
      this.jointAccelerationDesired.setLinearPartX(jointAcceleration.getLinearPartX());
      this.jointAccelerationDesired.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }
   

   public void setWrench(Wrench jointWrench)
   {
      successorWrench.getBodyFrame().checkReferenceFrameMatch(jointWrench.getBodyFrame());
      successorWrench.getExpressedInFrame().checkReferenceFrameMatch(jointWrench.getExpressedInFrame());
      successorWrench.setAngularPartY(jointWrench.getAngularPartY());
      successorWrench.setLinearPartX(jointWrench.getLinearPartX());
      successorWrench.setLinearPartZ(jointWrench.getLinearPartZ());
   }

   public void getRotation(Quat4d rotationToPack)
   {
      RotationTools.convertQuaternionToYawPitchRoll(this.jointRotation, yawPitchRoll);
      RotationTools.convertYawPitchRollToQuaternion(0.0, yawPitchRoll[1], 0.0, rotationToPack);
   }

   public void getRotation(Quat4f rotationToPack)
   {
      RotationTools.convertQuaternionToYawPitchRoll(this.jointRotation, yawPitchRoll);
      RotationTools.convertYawPitchRollToQuaternion(0.0, yawPitchRoll[1], 0.0, this.jointRotation);
      rotationToPack.set(jointRotation);
   }

   public void getRotation(Matrix3d rotationToPack)
   {
      RotationTools.convertQuaternionToYawPitchRoll(this.jointRotation, yawPitchRoll);
      RotationTools.convertYawPitchRollToMatrix(0.0, yawPitchRoll[1], 0.0, rotationToPack);
   }

   public void getRotation(double[] yawPitchRoll)
   {
      RotationTools.convertQuaternionToYawPitchRoll(this.jointRotation, this.yawPitchRoll);
      yawPitchRoll[0] = 0.0;
      yawPitchRoll[1] = this.yawPitchRoll[1];
      yawPitchRoll[2] = 0.0;
   }

   public void getTranslation(Tuple3d translationToPack)
   {
      translationToPack.setX(jointTranslation.getX());
      translationToPack.setZ(jointTranslation.getZ());
   }

   public void getTranslation(Tuple3f translationToPack)
   {
      translationToPack.set(jointTranslation);
      translationToPack.setY((float) 0.0);
   }

   public void setPositionAndRotation(RigidBodyTransform transform)
   {
      RotationTools.convertTransformToYawPitchRoll(transform, yawPitchRoll);
      RotationTools.convertYawPitchRollToQuaternion(0.0, yawPitchRoll[1], 0.0, jointRotation);

      if (!RotationTools.isQuaternionNormalized(jointRotation))
      {
         throw new AssertionError("quaternion is not normalized.  " + jointRotation);
      }

      transform.getTranslation(jointTranslation);
      jointTranslation.setY(0.0);
      this.afterJointFrame.setRotation(jointRotation);
      this.afterJointFrame.setTranslation(jointTranslation);
   }

   public Tuple3d getTranslationForReading()
   {
      return jointTranslation;
   }

   public Quat4d getRotationForReading()
   {
      return jointRotation;
   }

   public Vector3d getLinearVelocityForReading()
   {
      Vector3d linearVelocity = jointTwist.getLinearPart();
      linearVelocity.setY(0.0);

      return linearVelocity;
   }

   public Vector3d getAngularVelocityForReading()
   {
      Vector3d angularVelocity = jointTwist.getAngularPart();
      angularVelocity.setX(0.0);
      angularVelocity.setZ(0.0);

      return angularVelocity;
   }

   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocityToPack.set(0.0, jointTwist.getAngularPartY(), 0.0);
   }

   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocityToPack.set(jointTwist.getLinearPartX(), 0.0, jointTwist.getLinearPartZ());
   }

   public void getWrench(Wrench wrenchToPack)
   {
      wrenchToPack.setToZero(successorWrench.getBodyFrame(), successorWrench.getExpressedInFrame());

      wrenchToPack.setAngularPartY(successorWrench.getAngularPartY());
      wrenchToPack.setLinearPartX(successorWrench.getLinearPartX());
      wrenchToPack.setLinearPartZ(successorWrench.getLinearPartZ());
   }

   public void getLinearAcceleration(Vector3d linearAccelerationToPack)
   {
      linearAccelerationToPack.setX(jointAcceleration.getLinearPartX());
      linearAccelerationToPack.setY(0.0);
      linearAccelerationToPack.setZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      RotationTools.convertQuaternionToYawPitchRoll(jointRotation, yawPitchRoll);
      int index = rowStart;
      matrix.set(index++, 0, yawPitchRoll[1]);
      matrix.set(index++, 0, jointTranslation.getX());
      matrix.set(index++, 0, jointTranslation.getZ());
   }

   @Override
   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double qRot = matrix.get(index++, 0);
      double x = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      RotationTools.convertYawPitchRollToQuaternion(0.0, qRot, 0.0, jointRotation);
      jointTranslation.set(x, 0.0, z);
      afterJointFrame.setRotation(jointRotation);
      afterJointFrame.setTranslation(jointTranslation);
   }

   @Override
   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double qdRot = matrix.get(index++, 0);
      double xd = matrix.get(index++, 0);
      double zd = matrix.get(index++, 0);
      jointTwist.setToZero();
      jointTwist.setAngularPartY(qdRot);
      jointTwist.setLinearPart(xd, 0.0, zd);
   }

   @Override
   public void calculateJointStateChecksum(GenericCRC32 checksum)
   {
      RotationTools.convertQuaternionToYawPitchRoll(jointRotation, yawPitchRoll);
      checksum.update(yawPitchRoll[1]);
      checksum.update(jointTwist.getAngularPartY());
      checksum.update(jointAcceleration.getAngularPartY());

      checksum.update(jointTranslation.getX());
      checksum.update(jointTranslation.getY());
      checksum.update(jointTwist.getLinearPartX());
      checksum.update(jointTwist.getLinearPartZ());
      checksum.update(jointAcceleration.getLinearPartX());
      checksum.update(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void calculateJointDesiredChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointAccelerationDesired.getAngularPartY());
      checksum.update(jointAccelerationDesired.getLinearPartX());
      checksum.update(jointAccelerationDesired.getLinearPartZ());
   }

   @Override
   public int getConfigurationMatrixSize()
   {
      return getDegreesOfFreedom();
   }

   private void setMotionSubspace()
   {
      int nDegreesOfFreedom = getDegreesOfFreedom();

      ArrayList<Twist> unitTwistsInBodyFrame = new ArrayList<Twist>();
      RigidBodyTransform identity = new RigidBodyTransform();

      ReferenceFrame[] intermediateFrames = new ReferenceFrame[nDegreesOfFreedom - 1];
      ReferenceFrame previousFrame = afterJointFrame;
      for (int i = 0; i < nDegreesOfFreedom - 1; i++)    // from afterJointFrame to beforeJointFrame
      {
         int index = intermediateFrames.length - i - 1;
         ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("intermediateFrame" + index, previousFrame, identity);
         intermediateFrames[index] = frame;
         previousFrame = frame;
      }

      previousFrame = beforeJointFrame;

      for (int i = 0; i < nDegreesOfFreedom; i++)    // from beforeJointFrame to afterJointFrame
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
         int startIndex = 0;
         twistMatrix.set(startIndex + i, 0, 1.0);
         Twist twist = new Twist(frame, previousFrame, frame, twistMatrix);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      this.motionSubspace = new GeometricJacobian(this, unitTwistsInBodyFrame, afterJointFrame);
      this.motionSubspace.compute();
   }

   private PlanarJoint checkAndGetAsPlanarJoint(InverseDynamicsJoint originalJoint)
   {
      if (originalJoint instanceof PlanarJoint)
      {
         return (PlanarJoint) originalJoint;
      }
      else
      {
         throw new RuntimeException("Cannot set " + getClass().getSimpleName() + " to " + originalJoint.getClass().getSimpleName());
      }
   }
}
