package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public interface InertialEstimationParameters
{
   int[] getFloatingBaseJointIndices();

   SideDependentList<int[]> getLegJointIndices();

   int[] getSpineJointIndices();

   SideDependentList<int[]> getArmJointIndices();

   InertialParameterManagerFactory.EstimatorType getTypeOfEstimatorToUse();

   Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] getParametersToEstimate();

   DMatrixRMaj getURDFParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets);

   double getBreakFrequencyForPostProcessing();
   double getBreakFrequencyForEstimateFiltering();

   double getBreakFrequencyForAccelerationCalculation();

   double getBiasCompensationWindowSizeInSeconds();

   double[] getProcessModelCovarianceForBody();

   double getFloatingBaseMeasurementCovariance();
   double getLegMeasurementCovariance();
   double getArmMeasurementCovariance();
   double getSpineMeasurementCovariance();

   double getNormalizedInnovationThreshold();
}
