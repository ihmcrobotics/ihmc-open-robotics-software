package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;

import java.util.Set;

public interface InertialParameterManagerParameters
{
   public abstract Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] getParametersToEstimate();

   public abstract DMatrixRMaj getURDFParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets);

   public abstract double getBreakFrequencyForPostProcessing();
   public abstract double getBreakFrequencyForEstimateFiltering();

   public abstract double getProcessModelCovariance();
   public abstract double getFloatingBaseMeasurementCovariance();
   public abstract double getLegMeasurementCovariance();
   public abstract double getArmMeasurementCovariance();
   public abstract double getSpineMeasurementCovariance();
}
