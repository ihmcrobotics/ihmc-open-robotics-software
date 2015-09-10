package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;

import javax.vecmath.Vector2d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PlanarJoint extends AbstractInverseDynamicsJoint
{
   private final PlanarJointReferenceFrame afterJointFrame;
   private double qRot;
   private double qdRot;
   private double qddRot;
   private double qddRotDesired;
   private double tauRot;

   private final FrameVector2d qTrans;
   private final FrameVector2d qdTrans;
   private final FrameVector2d qddTrans;
   private final FrameVector2d qddTransDesired;
   private final FrameVector2d tauTrans;

   public PlanarJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame)
   {
      super(name, predecessor, beforeJointFrame);
      this.afterJointFrame = new PlanarJointReferenceFrame(name, beforeJointFrame);
      this.qTrans = new FrameVector2d(beforeJointFrame);
      this.qdTrans = new FrameVector2d(afterJointFrame);
      this.qddTrans = new FrameVector2d(afterJointFrame);
      this.qddTransDesired = new FrameVector2d(afterJointFrame);
      this.tauTrans = new FrameVector2d(afterJointFrame);
   }

   public ReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   public void packJointTwist(Twist twistToPack)
   {
      twistToPack.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      twistToPack.setAngularPartZ(qdRot);
      twistToPack.setLinearPartX(qdTrans.getX());
      twistToPack.setLinearPartY(qdTrans.getY());
   }

   public void packJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      accelerationToPack.setAngularPartZ(qddRot);
      accelerationToPack.setLinearPartX(qddTrans.getX());
      accelerationToPack.setLinearPartY(qddTrans.getY());
   }

   public void packDesiredJointAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      jointAcceleration.setToZero(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAcceleration.setAngularPartZ(qddRotDesired);
      jointAcceleration.setLinearPartX(qddTransDesired.getX());
      jointAcceleration.setLinearPartY(qddTransDesired.getY());
   }

   public void packTauMatrix(DenseMatrix64F matrix)
   {
      matrix.set(0, 0, tauRot);
      matrix.set(1, 0, tauTrans.getX());
      matrix.set(2, 0, tauTrans.getY());
   }

   public void packVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, qdRot);
      matrix.set(rowStart + 1, 0, qdTrans.getX());
      matrix.set(rowStart + 2, 0, qdTrans.getY());
   }

   public void packDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, qddRotDesired);
      matrix.set(rowStart + 1, 0, qddTransDesired.getX());
      matrix.set(rowStart + 2, 0, qddTransDesired.getY());
   }

   public void setDesiredAccelerationToZero()
   {
      qddRot = 0.0;
      qddTrans.set(0.0, 0.0);
   }

   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;
      setMotionSubspace();
   }

   public void updateMotionSubspace()
   {
      // empty
   }

   public void setTorqueFromWrench(Wrench jointWrench)
   {
      jointWrench.getBodyFrame().checkReferenceFrameMatch(successor.getBodyFixedFrame());
      jointWrench.setToZero(successor.getBodyFixedFrame(), tauTrans.getReferenceFrame());
      jointWrench.setAngularPartX(tauRot);
      jointWrench.setLinearPartX(tauTrans.getX());
      jointWrench.setLinearPartY(tauTrans.getY());
      jointWrench.changeFrame(successor.getBodyFixedFrame());
   }

   public int getDegreesOfFreedom()
   {
      return 3;
   }

   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      qddRotDesired = matrix.get(rowStart + 0, 0);
      qddTransDesired.setX(matrix.get(rowStart + 1, 0));
      qddTransDesired.setY(matrix.get(rowStart + 2, 0));
   }

   public void setRotation(double qRot)
   {
      this.qRot = qRot;
      afterJointFrame.setRotationAndUpdate(qRot);
   }

   public void setTranslation(FrameVector2d qTrans)
   {
      this.qTrans.set(qTrans);
      afterJointFrame.setTranslationAndUpdate(qTrans);
   }
   
   protected void setTranslation(Vector2d qTrans)
   {
      this.qTrans.set(qTrans);
      afterJointFrame.setTranslationAndUpdate(this.qTrans);
   }

   public void setRotationalVelocity(double qdRot)
   {
      this.qdRot = qdRot;
   }

   public void setTranslationalVelocity(FrameVector2d qdTrans)
   {
      this.qdTrans.set(qdTrans);
   }
   
   protected void setTranslationalVelocity(Vector2d qdTrans)
   {
      this.qdTrans.set(qdTrans);
   }

   public void setRotationalAcceleration(double qddRot)
   {
      this.qddRot = qddRot;
   }

   public void setTranslationalAcceleration(FrameVector2d qddTrans)
   {
      this.qddTrans.set(qddTrans);
   }
   
   protected void setTranslationalAcceleration(Vector2d qddTrans)
   {
      this.qddTrans.set(qddTrans);
   }

   public void setDesiredRotationalAcceleration(double qddRotDesired)
   {
      this.qddRotDesired = qddRotDesired;
   }

   public void setDesiredTranslationalAcceleration(FrameVector2d qddTransDesired)
   {
      this.qddTransDesired.set(qddTransDesired);
   }
   
   private void setDesiredTranslationalAcceleration(Vector2d qddTransDesired)
   {
      this.qddTransDesired.set(qddTransDesired);
   }

   public void setTauRotation(double tauRot)
   {
      this.tauRot = tauRot;
   }

   public void setTauTranslation(FrameVector2d tauTrans)
   {
      this.tauTrans.set(tauTrans);
   }

   public double getRotation()
   {
      return qRot;
   }

   public void packTranslation(FrameVector2d vectorToPack)
   {
      vectorToPack.setIncludingFrame(qTrans);
   }

   public double getRotationalVelocity()
   {
      return qdRot;
   }

   public void packTranslationalVelocity(FrameVector2d vectorToPack)
   {
      vectorToPack.set(qdTrans);
   }

   public double getRotationalAcceleration()
   {
      return qddRot;
   }

   public void packTranslationalAcceleration(FrameVector2d vectorToPack)
   {
      vectorToPack.set(qddTrans);
   }

   public double getDesiredRotationalAcceleration()
   {
      return qddRotDesired;
   }

   public void packDesiredTranslationalAcceleration(FrameVector2d vectorToPack)
   {
      vectorToPack.set(qddTransDesired);
   }

   public double getTauRotation()
   {
      return tauRot;
   }

   public void packTauTranslation(FrameVector2d vectorToPack)
   {
      vectorToPack.set(tauTrans);
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

   public void packConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      matrix.set(index++, 0, qRot);
      matrix.set(index++, 0, qTrans.getX());
      matrix.set(index++, 0, qTrans.getY());
   }

   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      qRot = matrix.get(index++, 0);
      double x = matrix.get(index++, 0);
      double y = matrix.get(index++, 0);
      qTrans.set(x, y);
      afterJointFrame.setRotationAndUpdate(qRot);
      afterJointFrame.setTranslationAndUpdate(qTrans);
   }

   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      qdRot = matrix.get(index++, 0);
      double xd = matrix.get(index++, 0);
      double yd = matrix.get(index++, 0);
      qdTrans.set(xd, yd);
   }


   public int getConfigurationMatrixSize()
   {
      return getDegreesOfFreedom();
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

   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
      PlanarJoint originalPlanarJoint = checkAndGetAsPlanarJoint(originalJoint);
      setRotation(originalPlanarJoint.qRot);
      setRotationalVelocity(originalPlanarJoint.qdRot);
      setRotationalAcceleration(originalPlanarJoint.qddRot);

      // HumanoidReferenceFrames are not the same!
      setTranslation(originalPlanarJoint.qTrans.getVector());
      setTranslationalVelocity(originalPlanarJoint.qdTrans.getVector());
      setTranslationalAcceleration(originalPlanarJoint.qddTrans.getVector());
   }

   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      PlanarJoint originalPlanarJoint = checkAndGetAsPlanarJoint(originalJoint);
      
      setDesiredRotationalAcceleration(originalPlanarJoint.getDesiredRotationalAcceleration());
      setDesiredTranslationalAcceleration(originalPlanarJoint.qddTransDesired.getVector());      
      
   }

   public void calculateJointStateChecksum(GenericCRC32 checksum)
   {
      checksum.update(qRot);
      checksum.update(qdRot);
      checksum.update(qddRot);
      
      checksum.update(qTrans.getVector());
      checksum.update(qdTrans.getVector());
      checksum.update(qddTrans.getVector());
   }

   public void calculateJointDesiredChecksum(GenericCRC32 checksum)
   {
      checksum.update(qddRotDesired);
      checksum.update(qddTransDesired.getVector());
   }
}
