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

   /**
    * When true (and using the physically-consistent EKF), each estimated body uses the generalized
    * parameterization (Eq. (11) of Rucker and Wensing): a physically-consistent deviation from the body's
    * known nominal inertia, rather than the absolute parameterization (Eq. (1)). Intended for estimating a
    * payload added to a link whose nominal inertia is known a priori. Defaults to false (absolute).
    */
   default boolean useGeneralizedPhysicalConsistency()
   {
      return false;
   }

   /**
    * Per-body prior (co)variance for a Tikhonov "soft constraint" pseudo-measurement that pulls the estimate
    * toward {@code theta = 0} (nominal, in the generalized parameterization). Returns the diagonal of {@code
    * R_prior} for one body (length {@code PARAMETERS_PER_RIGID_BODY}, Theta ordering {@code
    * [alpha,d1,d2,d3,s12,s13,s23,t1,t2,t3]}); the filter repeats it per estimated body and, each tick after the
    * torque update, applies the soft-constraint update {@code x <- R_prior (P+R_prior)^-1 x},
    * {@code P <- R_prior (P+R_prior)^-1 P}. Make entries LARGE where you trust the data (e.g. mass) so the prior
    * is negligible, and SMALL where the parameter is weakly observable (first moment / inertia) so it is held
    * near nominal. See Simon, "Optimal State Estimation" (2006) Ch. 7 (soft constraints / pseudo-measurements).
    * Return {@code null} (default) to disable the prior entirely.
    */
   default double[] getParameterPriorVariance()
   {
      return null;
   }

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
