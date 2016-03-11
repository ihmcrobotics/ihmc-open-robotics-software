package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class RootJointDesiredConfiguration
{
   private final DenseMatrix64F desiredConfiguration = new DenseMatrix64F(7, 0);
   private final DenseMatrix64F desiredVelocity = new DenseMatrix64F(6, 0);
   private final DenseMatrix64F desiredAcceleration = new DenseMatrix64F(6, 0);

   public RootJointDesiredConfiguration()
   {
      clear();
   }

   public void clear()
   {
      desiredConfiguration.reshape(0, 0);
      desiredVelocity.reshape(0, 0);
      desiredAcceleration.reshape(0, 0);
   }

   public void set(RootJointDesiredConfiguration other)
   {
      desiredConfiguration.set(other.desiredConfiguration);
      desiredVelocity.set(other.desiredVelocity);
      desiredAcceleration.set(other.desiredAcceleration);
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(RootJointDesiredConfiguration other)
   {
      if (!hasDesiredPosition())
         desiredConfiguration.set(other.desiredConfiguration);
      if (!hasDesiredVelocity())
         desiredVelocity.set(other.desiredVelocity);
      if (!hasDesiredAcceleration())
         desiredAcceleration.set(other.desiredAcceleration);
   }

   public void setDesiredPosition(DenseMatrix64F q)
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
      desiredConfiguration.reshape(7, 0);
      CommonOps.extract(q, startIndex, startIndex + 7, 0, 1, desiredConfiguration, 0, 0);
   }

   public void setDesiredVelocity(DenseMatrix64F qd, int startIndex)
   {
      desiredVelocity.reshape(6, 0);
      CommonOps.extract(qd, startIndex, startIndex + 6, 0, 1, desiredVelocity, 0, 0);
   }

   public void setDesiredAcceleration(DenseMatrix64F qdd, int startIndex)
   {
      desiredAcceleration.reshape(6, 0);
      CommonOps.extract(qdd, startIndex, startIndex + 6, 0, 1, desiredAcceleration, 0, 0);
   }

   public void setDesiredPosition(DenseMatrix64F q, int[] indices)
   {
      if (indices.length != 7)
         throw new RuntimeException("Unexpected number of indices: " + Arrays.toString(indices));
      desiredConfiguration.reshape(7, 0);
      for (int i = 0; i < indices.length; i++)
         desiredConfiguration.set(i, 0, q.get(indices[i], 0));
   }

   public void setDesiredVelocity(DenseMatrix64F qd, int[] indices)
   {
      if (indices.length != 6)
         throw new RuntimeException("Unexpected number of indices: " + Arrays.toString(indices));
      desiredVelocity.reshape(6, 0);
      for (int i = 0; i < indices.length; i++)
         desiredVelocity.set(i, 0, qd.get(indices[i], 0));
   }

   public void setDesiredAcceleration(DenseMatrix64F qdd, int[] indices)
   {
      if (indices.length != 6)
         throw new RuntimeException("Unexpected number of indices: " + Arrays.toString(indices));
      desiredAcceleration.reshape(6, 0);
      for (int i = 0; i < indices.length; i++)
         desiredAcceleration.set(i, 0, qdd.get(indices[i], 0));
   }

   public boolean hasDesiredPosition()
   {
      return desiredConfiguration.getNumRows() != 0;
   }

   public boolean hasDesiredVelocity()
   {
      return desiredVelocity.getNumRows() != 0;
   }

   public boolean hasDesiredAcceleration()
   {
      return desiredAcceleration.getNumRows() != 0;
   }

   public DenseMatrix64F getDesiredPosition()
   {
      return desiredConfiguration;
   }

   public DenseMatrix64F getDesiredVelocity()
   {
      return desiredVelocity;
   }

   public DenseMatrix64F getDesiredAcceleration()
   {
      return desiredAcceleration;
   }
}
