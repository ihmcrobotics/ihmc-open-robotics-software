package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SixDoFJoint extends AbstractInverseDynamicsJoint implements FloatingInverseDynamicsJoint
{
   private final FloatingInverseDynamicsJointReferenceFrame afterJointFrame;
   private final Quat4d jointRotation = new Quat4d(0.0, 0.0, 0.0, 1.0);
   private final Vector3d jointTranslation = new Vector3d();
   private final Twist jointTwist;
   private final SpatialAccelerationVector jointAcceleration;
   private final SpatialAccelerationVector jointAccelerationDesired;

   private Wrench successorWrench;

   public SixDoFJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame)
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
   public void getJointTwist(Twist twistToPack)
   {
      twistToPack.set(jointTwist);
   }
   
   @Override
   public void getJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(jointAcceleration);
   }
   
   @Override
   public void getDesiredJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(jointAccelerationDesired);
   }
   
   @Override
   public void getTauMatrix(DenseMatrix64F matrix)
   {
      successorWrench.getMatrix(matrix);
   }
   
   @Override
   public void getVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      jointTwist.getMatrix(matrix, rowStart);
   }
   
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      jointTwist.getAngularPart(angularVelocityToPack);
   }
   
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      jointTwist.getLinearPart(linearVelocityToPack);
   }
   
   public Vector3d getLinearVelocityForReading()
   {
      return jointTwist.getLinearPart();
   }
   
   public Vector3d getAngularVelocityForReading()
   {
      return jointTwist.getAngularPart();
   }
   
   @Override
   public void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      jointAccelerationDesired.getMatrix(matrix, rowStart);
   }
   
   @Override
   public void setDesiredAccelerationToZero()
   {
      jointAccelerationDesired.setToZero();
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
   public void setTorqueFromWrench(Wrench jointWrench)
   {
      this.successorWrench.checkAndSet(jointWrench);
   }
   
   @Override
   public int getDegreesOfFreedom()
   {
      return 6;
   }
   
   @Override
   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      jointAccelerationDesired.set(jointAccelerationDesired.getBodyFrame(), jointAccelerationDesired.getBaseFrame(),
            jointAccelerationDesired.getExpressedInFrame(), matrix, rowStart);
   }

   @Override
   public void setJointTorque(DenseMatrix64F matrix, int rowStart)
   {
      successorWrench.set(successorWrench.getBodyFrame(), successorWrench.getExpressedInFrame(), matrix, rowStart);
   }

   public void setPositionAndRotation(RigidBodyTransform transform)
   {
      RotationTools.convertTransformToQuaternion(transform, jointRotation);
      
      if (!RotationTools.isQuaternionNormalized(jointRotation))
      {
         throw new AssertionError("quaternion is not normalized.  " + jointRotation);
      }

      transform.getTranslation(jointTranslation);
      this.afterJointFrame.setRotation(jointRotation);
      this.afterJointFrame.setTranslation(jointTranslation);
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
   public void setRotation(double x, double y, double z, double w)
   {
      this.jointRotation.set(x, y, z, w);
      this.afterJointFrame.setRotation(this.jointRotation);
   }

   public void setRotation(Matrix3d jointRotation)
   {
      RotationTools.convertMatrixToQuaternion(jointRotation, this.jointRotation);

      // DON'T USE THIS: the method in Quat4d is flawed and doesn't work for some rotation matrices!
      //      this.jointRotation.set(jointRotation);

      this.afterJointFrame.setRotation(this.jointRotation);
   }

   public void setPosition(Tuple3d jointTranslation)
   {
      this.jointTranslation.set(jointTranslation);
      this.afterJointFrame.setTranslation(this.jointTranslation);
   }

   public void setPosition(double x, double y, double z)
   {
      jointTranslation.set(x, y, z);
      this.afterJointFrame.setTranslation(this.jointTranslation);
   }

   public void setJointTwist(Twist jointTwist)
   {
      this.jointTwist.checkAndSet(jointTwist);
   }

   public void setAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      this.jointAcceleration.checkAndSet(jointAcceleration);
   }

   public void setDesiredAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      this.jointAccelerationDesired.checkAndSet(jointAcceleration);
   }

   public void setWrench(Wrench jointWrench)
   {
      this.successorWrench.checkAndSet(jointWrench);
   }

   public void getRotation(Quat4d rotationToPack)
   {
      rotationToPack.set(jointRotation);
   }

   public void getRotation(Quat4f rotationToPack)
   {
      rotationToPack.set(jointRotation);
   }

   public void getRotation(Matrix3d rotationToPack)
   {
      rotationToPack.set(jointRotation);
   }

   public void getRotation(double[] yawPitchRoll)
   {
      RotationTools.convertQuaternionToYawPitchRoll(jointRotation, yawPitchRoll);
   }

   public void getTranslation(Tuple3d translationToPack)
   {
      translationToPack.set(jointTranslation);
   }

   public void getTranslation(Tuple3f translationToPack)
   {
      translationToPack.set(jointTranslation);
   }

   public Tuple3d getTranslationForReading()
   {
      return jointTranslation;
   }
   
   public Quat4d getRotationForReading()
   {
      return jointRotation;
   }

   public void getWrench(Wrench wrenchToPack)
   {
      wrenchToPack.set(successorWrench);
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

         DenseMatrix64F twistMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
         twistMatrix.set(i, 0, 1.0);
         Twist twist = new Twist(frame, previousFrame, frame, twistMatrix);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      this.motionSubspace = new GeometricJacobian(this, unitTwistsInBodyFrame, afterJointFrame);
      this.motionSubspace.compute();
   }
   
   @Override
   public void updateMotionSubspace()
   {
      // empty
   }
   
   @Override
   public void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      MatrixTools.insertQuat4dIntoEJMLVector(matrix, jointRotation, rowStart);
      //      RotationFunctions.assertQuaternionNormalized(jointRotation, "SixDoFJoint "+name+":");
      index += RotationTools.QUATERNION_SIZE;
      matrix.set(index++, 0, jointTranslation.getX());
      matrix.set(index++, 0, jointTranslation.getY());
      matrix.set(index++, 0, jointTranslation.getZ());
   }
   
   @Override
   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      MatrixTools.extractQuat4dFromEJMLVector(jointRotation, matrix, rowStart);
      RotationTools.checkQuaternionNormalized(jointRotation);
      index += RotationTools.QUATERNION_SIZE;
      jointTranslation.setX(matrix.get(index++, 0));
      jointTranslation.setY(matrix.get(index++, 0));
      jointTranslation.setZ(matrix.get(index++, 0));
      this.afterJointFrame.setRotation(jointRotation);
      this.afterJointFrame.setTranslation(jointTranslation);
   }
   
   @Override
   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      jointTwist.set(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getExpressedInFrame(), matrix, rowStart);
   }
   
   //FIXME: FIX THIS!!!!
   /**
    * The implementation for this method generates garbage and is wrong.
    * Do not use it or fix it.
    * 
    * The implementation should be something like this:
    * <p> {@code RigidBodyTransform inverseTransformToRoot = afterJointFrame.getInverseTransformToRoot();}
    * <p> {@code inverseTransformToRoot.transform(linearVelocityInWorld);}
    * <p> {@code jointTwist.setLinearPart(linearVelocityInWorld);}
    *
    * Sylvain
    * 
    * @deprecated
    * @param linearVelocityInWorld
    */
   public void setLinearVelocityInWorld(Vector3d linearVelocityInWorld)
   {
      Twist newTwist = new Twist(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), ReferenceFrame.getWorldFrame());
      newTwist.setLinearPart(linearVelocityInWorld);
      newTwist.changeFrame(jointTwist.getExpressedInFrame());
      newTwist.setAngularPart(jointTwist.getAngularPart());
      
      jointTwist.set(newTwist);
   }
   
   @Override
   public int getConfigurationMatrixSize()
   {
      int positionSize = 3;

      return RotationTools.QUATERNION_SIZE + positionSize;
   }

   private SixDoFJoint checkAndGetAsSiXDoFJoint(InverseDynamicsJoint originalJoint)
   {
      if (originalJoint instanceof SixDoFJoint)
      {
         return (SixDoFJoint) originalJoint;
      }
      else
      {
         throw new RuntimeException("Cannot set " + getClass().getSimpleName() + " to " + originalJoint.getClass().getSimpleName());
      }
   }
   
   @Override
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
      SixDoFJoint sixDoFOriginalJoint = checkAndGetAsSiXDoFJoint(originalJoint);
      setPosition(sixDoFOriginalJoint.jointTranslation);
      setRotation(sixDoFOriginalJoint.jointRotation);

      jointTwist.setAngularPart(sixDoFOriginalJoint.jointTwist.getAngularPart());
      jointTwist.setLinearPart(sixDoFOriginalJoint.jointTwist.getLinearPart());

      jointAcceleration.setAngularPart(sixDoFOriginalJoint.jointAcceleration.getAngularPart());
      jointAcceleration.setLinearPart(sixDoFOriginalJoint.jointAcceleration.getLinearPart());
   }
   
   @Override
   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      SixDoFJoint sixDoFOriginalJoint = checkAndGetAsSiXDoFJoint(originalJoint);
      jointAccelerationDesired.setAngularPart(sixDoFOriginalJoint.jointAccelerationDesired.getAngularPart());
      jointAccelerationDesired.setLinearPart(sixDoFOriginalJoint.jointAccelerationDesired.getLinearPart());
   }
   
   @Override
   public void calculateJointStateChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointTranslation);
      checksum.update(jointRotation);
      checksum.update(jointTwist.getAngularPart());
      checksum.update(jointTwist.getLinearPart());
      checksum.update(jointAcceleration.getAngularPart());
      checksum.update(jointAcceleration.getLinearPart());
   }
   
   @Override
   public void calculateJointDesiredChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointAccelerationDesired.getAngularPart());
      checksum.update(jointAccelerationDesired.getLinearPart());
   }

   public void getLinearAcceleration(Vector3d linearAccelerationToPack)
   {
      linearAccelerationToPack.setX(jointAcceleration.getLinearPartX());
      linearAccelerationToPack.setY(jointAcceleration.getLinearPartY());
      linearAccelerationToPack.setZ(jointAcceleration.getLinearPartZ());
   }
}
