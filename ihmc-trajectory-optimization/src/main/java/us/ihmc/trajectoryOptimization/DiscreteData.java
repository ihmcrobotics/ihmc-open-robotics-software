package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;

public interface DiscreteData
{
   void setZero(int size);
   int size();

   DMatrixRMaj get(int index);
   void add(int index, DMatrixRMaj data);
   void set(DiscreteData other);
}
