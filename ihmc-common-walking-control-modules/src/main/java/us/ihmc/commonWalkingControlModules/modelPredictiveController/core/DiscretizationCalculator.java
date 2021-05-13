package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;

public interface DiscretizationCalculator
{
   void compute(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj Ad, DMatrixRMaj Bd, DMatrixRMaj Cd, double tickDuration);
}
