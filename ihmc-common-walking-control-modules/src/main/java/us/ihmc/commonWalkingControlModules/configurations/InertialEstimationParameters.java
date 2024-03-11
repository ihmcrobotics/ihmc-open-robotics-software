package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory.EstimatorType;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public interface InertialEstimationParameters
{
   int[] getFloatingBaseJointIndices();

   SideDependentList<int[]> getLegJointIndices();

   int[] getSpineJointIndices();

   SideDependentList<int[]> getArmJointIndices();

   InertialParameterManagerFactory.EstimatorType getTypeOfEstimatorToUse();

   Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] getBasisSets();

   default String[] getBasisNames()
   {
      return getTypeOfEstimatorToUse() == EstimatorType.PHYSICALLY_CONSISTENT_EKF ?
            RigidBodyInertialParametersTools.getNamesForThetaBasis() : RigidBodyInertialParametersTools.getNamesForPiBasis();
   }

   default int getNumberOfParameters()
   {
      int numberOfParameters = 0;
      for (Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption> basisSet : getBasisSets())
      {
         numberOfParameters += basisSet.size();
      }
      return numberOfParameters;
   }

   DMatrixRMaj getURDFParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets);

   double getBreakFrequencyForPostProcessing();
   double getBreakFrequencyForEstimateFiltering();

   double getBreakFrequencyForAccelerationCalculation();

   double getBiasCompensationWindowSizeInSeconds();

   double[] getProcessCovariance();

   double getFloatingBaseMeasurementCovariance();
   double getLegMeasurementCovariance();
   double getArmMeasurementCovariance();
   double getSpineMeasurementCovariance();

   double getNormalizedInnovationThreshold();
}
