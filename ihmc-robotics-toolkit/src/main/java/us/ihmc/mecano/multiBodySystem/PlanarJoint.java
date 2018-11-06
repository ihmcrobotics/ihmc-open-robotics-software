package us.ihmc.mecano.multiBodySystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class PlanarJoint extends AbstractInverseDynamicsJoint implements FloatingInverseDynamicsJoint
{
   private final Quaternion jointRotation = new Quaternion();
   private final Vector3D jointTranslation = new Vector3D();
   private final Twist jointTwist;
   private final SpatialAcceleration jointAcceleration;
   private final SpatialAcceleration jointAccelerationDesired;

   private List<Twist> unitTwists;

   private Wrench successorWrench;

   public PlanarJoint(String name, RigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   public PlanarJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
   {
      super(name, predecessor, transformToParent);

      jointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAcceleration = new SpatialAcceleration(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAccelerationDesired = new SpatialAcceleration(afterJointFrame, beforeJointFrame, afterJointFrame);
   }

   @Override
   protected void updateJointTransform(RigidBodyTransform jointTransform)
   {
      jointTransform.set(jointRotation, jointTranslation);
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void setJointAccelerationToZero()
   {
      jointAccelerationDesired.setToZero();
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      setMotionSubspace();

      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      successorWrench = new Wrench(successorFrame, successorFrame);
   }

   @Override
   public void getJointTau(int rowStart, DenseMatrix64F matrix)
   {
      matrix.set(0, 0, successorWrench.getAngularPartY());
      matrix.set(1, 0, successorWrench.getLinearPartX());
      matrix.set(2, 0, successorWrench.getLinearPartZ());
   }

   @Override
   public void getJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      matrix.set(rowStart + 0, 0, jointTwist.getAngularPartY());
      matrix.set(rowStart + 1, 0, jointTwist.getLinearPartX());
      matrix.set(rowStart + 2, 0, jointTwist.getLinearPartZ());
   }

   @Override
   public void getJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      matrix.set(rowStart + 0, 0, jointAccelerationDesired.getAngularPartY());
      matrix.set(rowStart + 1, 0, jointAccelerationDesired.getLinearPartX());
      matrix.set(rowStart + 2, 0, jointAccelerationDesired.getLinearPartZ());
   }

   @Override
   public void setJointWrench(Wrench jointWrench)
   {
      jointWrench.getBodyFrame().checkReferenceFrameMatch(successor.getBodyFixedFrame());
      jointWrench.setToZero(successor.getBodyFixedFrame(), successorWrench.getReferenceFrame());
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
   public void setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      jointAccelerationDesired.setToZero();
      jointAccelerationDesired.setAngularPartY(matrix.get(rowStart + 0, 0));
      jointAccelerationDesired.setLinearPartX(matrix.get(rowStart + 1, 0));
      jointAccelerationDesired.setLinearPartZ(matrix.get(rowStart + 2, 0));
   }

   @Override
   public void setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      successorWrench.setAngularPartY(matrix.get(rowStart + 0));
      successorWrench.setLinearPartX(matrix.get(rowStart + 1));
      successorWrench.setLinearPartZ(matrix.get(rowStart + 2));
   }

   @Override
   public void setJointOrientation(QuaternionReadOnly jointRotation)
   {
      this.jointRotation.setToPitchQuaternion(jointRotation.getPitch());
   }

   @Override
   public void setJointOrientation(RotationMatrixReadOnly jointRotation)
   {
      this.jointRotation.setToPitchQuaternion(jointRotation.getPitch());
   }

   @Override
   public void setJointPosition(Tuple3DReadOnly jointTranslation)
   {
      this.jointTranslation.set(jointTranslation.getX(), 0.0, jointTranslation.getZ());
   }

   @Override
   public void setJointTwist(Twist jointTwist)
   {
      this.jointTwist.checkReferenceFrameMatch(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getReferenceFrame());
      this.jointTwist.setAngularPartY(jointTwist.getAngularPartY());
      this.jointTwist.setLinearPartX(jointTwist.getLinearPartX());
      this.jointTwist.setLinearPartZ(jointTwist.getLinearPartZ());
   }

   @Override
   public void setJointAcceleration(SpatialAcceleration jointAcceleration)
   {
      this.jointAcceleration.checkReferenceFrameMatch(jointAcceleration);
      this.jointAcceleration.setAngularPartY(jointAcceleration.getAngularPartY());
      this.jointAcceleration.setLinearPartX(jointAcceleration.getLinearPartX());
      this.jointAcceleration.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void setDesiredAcceleration(SpatialAcceleration jointAcceleration)
   {
      jointAccelerationDesired.checkReferenceFrameMatch(jointAcceleration);
      jointAccelerationDesired.setAngularPartY(jointAcceleration.getAngularPartY());
      jointAccelerationDesired.setLinearPartX(jointAcceleration.getLinearPartX());
      jointAccelerationDesired.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void setWrench(Wrench jointWrench)
   {
      successorWrench.getBodyFrame().checkReferenceFrameMatch(jointWrench.getBodyFrame());
      successorWrench.getReferenceFrame().checkReferenceFrameMatch(jointWrench.getReferenceFrame());
      successorWrench.setAngularPartY(jointWrench.getAngularPartY());
      successorWrench.setLinearPartX(jointWrench.getLinearPartX());
      successorWrench.setLinearPartZ(jointWrench.getLinearPartZ());
   }

   @Override
   public void setJointConfiguration(RigidBodyTransform transform)
   {
      jointRotation.setToPitchQuaternion(transform.getRotationMatrix().getPitch());
      jointRotation.checkIfUnitary();

      transform.getTranslation(jointTranslation);
      jointTranslation.setY(0.0);
   }

   @Override
   public void getJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      int index = rowStart;
      matrix.set(index++, 0, jointRotation.getPitch());
      matrix.set(index++, 0, jointTranslation.getX());
      matrix.set(index++, 0, jointTranslation.getZ());
   }

   @Override
   public void setJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      int index = rowStart;
      double qRot = matrix.get(index++, 0);
      double x = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      jointRotation.setToPitchQuaternion(qRot);
      jointTranslation.set(x, 0.0, z);
   }

   @Override
   public void setJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      int index = rowStart;
      double qdRot = matrix.get(index++, 0);
      double xd = matrix.get(index++, 0);
      double zd = matrix.get(index++, 0);
      jointTwist.setToZero();
      jointTwist.setAngularPartY(qdRot);
      jointTwist.getLinearPart().set(xd, 0.0, zd);
   }

   @Override
   public int getConfigurationMatrixSize()
   {
      return getDegreesOfFreedom();
   }

   private void setMotionSubspace()
   {
      int nDegreesOfFreedom = getDegreesOfFreedom();

      ArrayList<Twist> unitTwistsInBodyFrame = new ArrayList<>();
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

         DenseMatrix64F twistMatrix = new DenseMatrix64F(SpatialVector.SIZE, 1);
         int startIndex = 0;
         twistMatrix.set(startIndex + i, 0, 1.0);
         Twist twist = new Twist(frame, previousFrame, frame, twistMatrix);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      unitTwists = Collections.unmodifiableList(unitTwistsInBodyFrame);

      motionSubspace = new GeometricJacobian(this, afterJointFrame);
      motionSubspace.compute();
   }

   @Override
   public void getUnitTwist(int dofIndex, Twist unitTwistToPack)
   {
      if (dofIndex < 0 || dofIndex >= getDegreesOfFreedom())
         throw new ArrayIndexOutOfBoundsException("Illegal index: " + dofIndex + ", was expecting dofIndex in [0, " + getDegreesOfFreedom() + "[.");
      unitTwistToPack.setIncludingFrame(unitTwists.get(dofIndex));
   }

   private PlanarJoint checkAndGetAsPlanarJoint(JointBasics originalJoint)
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
