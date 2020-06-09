package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;

public interface DiscreteOptimizationData
{
   void set(DiscreteOptimizationData other);

   DiscreteData getControlSequence();
   DiscreteData getStateSequence();

   DMatrixRMaj getState(int index);
   DMatrixRMaj getControl(int index);

   void setState(int index, DMatrixRMaj state);
   void setControl(int index, DMatrixRMaj control);

   void setZero(DiscreteOptimizationData other);

   int size();
}
