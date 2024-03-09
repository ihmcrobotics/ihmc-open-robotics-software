package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public interface InertialEstimationParameters
{
   String[] getTrunkBodies();
   SideDependentList<String[]> getArmBodies();
   SideDependentList<String[]> getLegBodies();

   String[] getSpineJoints();
   SideDependentList<String[]> getArmJoints();
   SideDependentList<String[]> getLegJoints();

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
