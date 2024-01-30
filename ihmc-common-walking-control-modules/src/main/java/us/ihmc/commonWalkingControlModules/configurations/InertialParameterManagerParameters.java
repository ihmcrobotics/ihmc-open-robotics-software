package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.Set;

public interface InertialParameterManagerParameters
{
   public abstract Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] getParametersToEstimate();

   public abstract DMatrixRMaj getURDFParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets);
}
