package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

public interface DiscreteData
{
   void setZero(int size);
   int size();

   DenseMatrix64F get(int index);
   void add(int index, DenseMatrix64F data);
   void set(DiscreteData other);
}
