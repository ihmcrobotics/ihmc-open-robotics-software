package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

public interface DiscreteOptimizationData
{
   void set(DiscreteOptimizationData other);

   DiscreteData getControlSequence();
   DiscreteData getStateSequence();

   DenseMatrix64F getState(int index);
   DenseMatrix64F getControl(int index);

   void setState(int index, DenseMatrix64F state);
   void setControl(int index, DenseMatrix64F control);

   void setZero(DiscreteOptimizationData other);

   int size();
}
