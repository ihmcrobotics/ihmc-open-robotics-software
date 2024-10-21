package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertialEstimatorType;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public interface InertialEstimationParameters
{
   int[] getFloatingBaseJointIndices();

   SideDependentList<int[]> getLegJointIndices();

   int[] getSpineJointIndices();

   SideDependentList<int[]> getArmJointIndices();

   String[] getMeasurementNames();

   String[] getEstimateNames();

   InertialEstimatorType getTypeOfEstimatorToUse();

   Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] getBasisSets();

   default String[] getBasisNames()
   {
      return getTypeOfEstimatorToUse() == InertialEstimatorType.PHYSICALLY_CONSISTENT_EKF ?
            RigidBodyInertialParametersTools.getNamesForThetaBasis() :
            RigidBodyInertialParametersTools.getNamesForPiBasis();
   }

   default int getNumberOfNonEmptyBasisSets()
   {
      int numberOfNonEmptyBasisSets = 0;
      for (Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption> basisSet : getBasisSets())
      {
         if (!basisSet.isEmpty())
            numberOfNonEmptyBasisSets++;
      }
      return numberOfNonEmptyBasisSets;
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

   double[] getMaxParameterDeltaRates();
}
