package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;

public class RootJointDesiredConfigurationData implements RootJointDesiredConfigurationDataReadOnly
{
   private final DenseMatrix64F desiredConfiguration = new DenseMatrix64F(7, 0);
   private final DenseMatrix64F desiredVelocity = new DenseMatrix64F(6, 0);
   private final DenseMatrix64F desiredAcceleration = new DenseMatrix64F(6, 0);

   public RootJointDesiredConfigurationData()
   {
      clear();
   }

   public void clear()
   {
      desiredConfiguration.reshape(0, 0);
      desiredVelocity.reshape(0, 0);
      desiredAcceleration.reshape(0, 0);
   }

   public void set(RootJointDesiredConfigurationDataReadOnly other)
   {
      desiredConfiguration.set(other.getDesiredConfiguration());
      desiredVelocity.set(other.getDesiredVelocity());
      desiredAcceleration.set(other.getDesiredAcceleration());
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(RootJointDesiredConfigurationData other)
   {
      if (!hasDesiredConfiguration())
         desiredConfiguration.set(other.desiredConfiguration);
      if (!hasDesiredVelocity())
         desiredVelocity.set(other.desiredVelocity);
      if (!hasDesiredAcceleration())
         desiredAcceleration.set(other.desiredAcceleration);
   }

   public void setDesiredAccelerationFromJoint(FloatingInverseDynamicsJoint sixDoFJoint)
   {
      desiredAcceleration.reshape(6, 1);
      sixDoFJoint.getDesiredAccelerationMatrix(desiredAcceleration, 0);
   }

   public void setDesiredConfiguration(FrameOrientation orientation, FramePoint position)
   {
      desiredConfiguration.reshape(7, 1);
      MatrixTools.insertQuat4dIntoEJMLVector(desiredConfiguration, orientation.getQuaternion(), 0);
      MatrixTools.insertTuple3dIntoEJMLVector(position.getPoint(), desiredConfiguration, 4);
   }

   public void setDesiredVelocity(FrameVector angularVelocity, FrameVector linearVelocity)
   {
      desiredVelocity.reshape(6, 1);
      MatrixTools.insertTuple3dIntoEJMLVector(angularVelocity.getVector(), desiredVelocity, 0);
      MatrixTools.insertTuple3dIntoEJMLVector(linearVelocity.getVector(), desiredVelocity, 3);
   }

   public void setDesiredAcceleration(FrameVector angularAcceleration, FrameVector linearAcceleration)
   {
      desiredAcceleration.reshape(6, 1);
      MatrixTools.insertTuple3dIntoEJMLVector(angularAcceleration.getVector(), desiredAcceleration, 0);
      MatrixTools.insertTuple3dIntoEJMLVector(linearAcceleration.getVector(), desiredAcceleration, 3);
   }

   public void setDesiredConfiguration(DenseMatrix64F q)
   {
      if (q.getNumRows() != 7 || q.getNumCols() != 1)
         throw new RuntimeException("Unexpected size: " + q);

      desiredConfiguration.set(q);
   }

   public void setDesiredVelocity(DenseMatrix64F qd)
   {
      if (qd.getNumRows() != 6 || qd.getNumCols() != 1)
         throw new RuntimeException("Unexpected size: " + qd);

      desiredVelocity.set(qd);
   }

   public void setDesiredAcceleration(DenseMatrix64F qdd)
   {
      if (qdd.getNumRows() != 6 || qdd.getNumCols() != 1)
         throw new RuntimeException("Unexpected size: " + qdd);

      desiredAcceleration.set(qdd);
   }

   public void setDesiredConfiguration(DenseMatrix64F q, int startIndex)
   {
      desiredConfiguration.reshape(7, 1);
      CommonOps.extract(q, startIndex, startIndex + 7, 0, 1, desiredConfiguration, 0, 0);
   }

   public void setDesiredVelocity(DenseMatrix64F qd, int startIndex)
   {
      desiredVelocity.reshape(6, 1);
      CommonOps.extract(qd, startIndex, startIndex + 6, 0, 1, desiredVelocity, 0, 0);
   }

   public void setDesiredAcceleration(DenseMatrix64F qdd, int startIndex)
   {
      desiredAcceleration.reshape(6, 1);
      CommonOps.extract(qdd, startIndex, startIndex + 6, 0, 1, desiredAcceleration, 0, 0);
   }

   public void setDesiredPosition(DenseMatrix64F q, int[] indices)
   {
      if (indices.length != 7)
         throw new RuntimeException("Unexpected number of indices: " + Arrays.toString(indices));
      desiredConfiguration.reshape(7, 1);
      for (int i = 0; i < indices.length; i++)
         desiredConfiguration.set(i, 0, q.get(indices[i], 0));
   }

   public void setDesiredVelocity(DenseMatrix64F qd, int[] indices)
   {
      if (indices.length != 6)
         throw new RuntimeException("Unexpected number of indices: " + Arrays.toString(indices));
      desiredVelocity.reshape(6, 1);
      for (int i = 0; i < indices.length; i++)
         desiredVelocity.set(i, 0, qd.get(indices[i], 0));
   }

   public void setDesiredAcceleration(DenseMatrix64F qdd, int[] indices)
   {
      if (indices.length != 6)
         throw new RuntimeException("Unexpected number of indices: " + Arrays.toString(indices));
      desiredAcceleration.reshape(6, 1);
      for (int i = 0; i < indices.length; i++)
         desiredAcceleration.set(i, 0, qdd.get(indices[i], 0));
   }

   @Override
   public boolean hasDesiredConfiguration()
   {
      return desiredConfiguration.getNumRows() != 0;
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return desiredVelocity.getNumRows() != 0;
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return desiredAcceleration.getNumRows() != 0;
   }

   @Override
   public DenseMatrix64F getDesiredConfiguration()
   {
      return desiredConfiguration;
   }

   @Override
   public DenseMatrix64F getDesiredVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public DenseMatrix64F getDesiredAcceleration()
   {
      return desiredAcceleration;
   }
}
