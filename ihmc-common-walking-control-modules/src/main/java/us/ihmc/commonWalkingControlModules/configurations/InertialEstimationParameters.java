package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;

import java.util.Set;

public interface InertialEstimationParameters
{
   InertialParameterManagerFactory.EstimatorType getTypeOfEstimatorToUse();

   Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] getParametersToEstimate();

   DMatrixRMaj getURDFParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets);

   double getBreakFrequencyForPostProcessing();
   double getBreakFrequencyForEstimateFiltering();

   double getBreakFrequencyForAccelerationCalculation();

   double getBiasCompensationWindowSizeInSeconds();

   double getProcessModelCovariance();
   double[] getProcessModelCovarianceForBody();
   double getProcessCovarianceMultiplierForWalking();


   double[] getFloatingBaseMeasurementCovariance();
   double getLegMeasurementCovariance();
   double getArmMeasurementCovariance();
   double getSpineMeasurementCovariance();

   double getNormalizedInnovationThreshold();
}
