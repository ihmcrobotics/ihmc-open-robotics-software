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
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class SixDoFJoint extends AbstractInverseDynamicsJoint implements FloatingJointBasics
{
   private final Quaternion jointRotation = new Quaternion(0.0, 0.0, 0.0, 1.0);
   private final Vector3D jointTranslation = new Vector3D();
   private final Twist jointTwist;
   private final SpatialAcceleration jointAcceleration;
   private final SpatialAcceleration jointAccelerationDesired;

   private List<Twist> unitTwists;

   private Wrench successorWrench;

   public SixDoFJoint(String name, RigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   public SixDoFJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
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
   public void getJointTau(int rowStart, DenseMatrix64F matrix)
   {
      successorWrench.get(matrix);
   }

   @Override
   public void getJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      jointTwist.get(rowStart, matrix);
   }

   @Override
   public void getJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      jointAccelerationDesired.get(rowStart, matrix);
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
   public void setJointWrench(Wrench jointWrench)
   {
      successorWrench.set(jointWrench);
   }

   @Override
   public int getDegreesOfFreedom()
   {
      return 6;
   }

   @Override
   public void setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      jointAccelerationDesired.setIncludingFrame(jointAccelerationDesired.getBodyFrame(), jointAccelerationDesired.getBaseFrame(),
                                   jointAccelerationDesired.getReferenceFrame(), rowStart, matrix);
   }

   @Override
   public void setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      successorWrench.setIncludingFrame(successorWrench.getBodyFrame(), successorWrench.getReferenceFrame(), rowStart, matrix);
   }

   public void setJointConfiguration(RigidBodyTransform transform)
   {
      transform.getRotation(jointRotation);
      jointRotation.checkIfUnitary();

      transform.getTranslation(jointTranslation);
   }
   
   @Override
   public void setJointOrientation(QuaternionReadOnly jointRotation)
   {
      this.jointRotation.set(jointRotation);
   }

   @Override
   public void setJointOrientation(RotationMatrixReadOnly jointRotation)
   {
      this.jointRotation.set(jointRotation);
   }

   @Override
   public void setJointPosition(Tuple3DReadOnly jointTranslation)
   {
      this.jointTranslation.set(jointTranslation);
   }

   @Override
   public void setJointTwist(Twist jointTwist)
   {
      this.jointTwist.set(jointTwist);
   }

   @Override
   public void setJointAcceleration(SpatialAcceleration jointAcceleration)
   {
      this.jointAcceleration.set(jointAcceleration);
   }

   @Override
   public void setDesiredAcceleration(SpatialAcceleration jointAcceleration)
   {
      jointAccelerationDesired.set(jointAcceleration);
   }

   @Override
   public void setWrench(Wrench jointWrench)
   {
      successorWrench.set(jointWrench);
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

         DenseMatrix64F twistMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
         twistMatrix.set(i, 0, 1.0);
         Twist twist = new Twist(frame, previousFrame, frame, twistMatrix);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      unitTwists = Collections.unmodifiableList(unitTwistsInBodyFrame);

      motionSubspace = new GeometricJacobian(this, afterJointFrame);
      motionSubspace.compute();
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void getJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      jointRotation.get(rowStart, matrix);
      jointTranslation.get(rowStart + 4, matrix);
   }

   @Override
   public void setJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      jointRotation.set(rowStart, matrix);
      jointTranslation.set(rowStart + 4, matrix);
   }

   @Override
   public void setJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      jointTwist.setIncludingFrame(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getReferenceFrame(), rowStart, matrix);
   }

   //FIXME: FIX THIS!!!!
   /**
    * The implementation for this method generates garbage and is wrong. Do not use it or fix it.
    *
    * The implementation should be something like this:
    * <p>
    * {@code RigidBodyTransform inverseTransformToRoot = afterJointFrame.getInverseTransformToRoot();}
    * <p>
    * {@code inverseTransformToRoot.transform(linearVelocityInWorld);}
    * <p>
    * {@code jointTwist.setLinearPart(linearVelocityInWorld);}
    *
    * Sylvain
    *
    * @deprecated
    * @param linearVelocityInWorld
    */
   @Deprecated
   public void setLinearVelocityInWorld(Vector3D linearVelocityInWorld)
   {
      Twist newTwist = new Twist(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), ReferenceFrame.getWorldFrame());
      newTwist.getLinearPart().set(linearVelocityInWorld);
      newTwist.changeFrame(jointTwist.getReferenceFrame());
      newTwist.getAngularPart().set(jointTwist.getAngularPart());

      jointTwist.setIncludingFrame(newTwist);
   }

   @Override
   public int getConfigurationMatrixSize()
   {
      int positionSize = 3;

      return RotationTools.QUATERNION_SIZE + positionSize;
   }

   private SixDoFJoint checkAndGetAsSiXDoFJoint(JointBasics originalJoint)
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
}
