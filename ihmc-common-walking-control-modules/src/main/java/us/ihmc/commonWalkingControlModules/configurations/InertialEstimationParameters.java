package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.parameterEstimation.GraspRejectionMethod;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertialEstimatorType;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotics.robotSide.RobotSide;
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

   /**
    * Attachment-point prior: override the reference-body CoM offset (a point in the body-fixed frame) for the
    * named estimated body. In the generalized parameterization the strongly-observable mass (alpha) channel
    * scales mass while leaving the CoM at the reference body's CoM, so anchoring the reference CoM at a KNOWN
    * attachment point (e.g. a grasped payload at the hand/cuff) makes the estimator deposit the estimated
    * payload mass THERE rather than at the link's nominal CoM. The manager sets the ESTIMATE-model body's CoM
    * to this before the filter/baseline capture the nominal, so the write-back delta (Delta_mass * refCoM)
    * carries the payload's first moment at this point into the controller model. Return {@code null} (default)
    * for no override -- behaviour is then unchanged.
    */
   default us.ihmc.euclid.tuple3D.Vector3D getReferenceCenterOfMassOffsetOverride(String bodyName)
   {
      return null;
   }

   /**
    * Builds the moment of inertia (about the body-fixed frame origin) of a physically-consistent reference body
    * whose mass sits at the attachment CoM {@code com}: a point mass there (parallel-axis term
    * {@code m(|c|^2 I - c c^T)}) plus a small isotropic about-CoM term so the pseudo-inertia is strictly positive
    * definite (required by the log-Cholesky factorisation). Used to build the attachment-point reference
    * CONSISTENTLY for both the filter's initial pi state ({@link #getURDFParameters}) and the ESTIMATE model's
    * nominal reference, so the initial deviation is exactly zero.
    */
   static void packAttachmentReferenceMomentOfInertia(double mass,
                                                      us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly com,
                                                      us.ihmc.euclid.matrix.Matrix3D momentOfInertiaAboutOriginToPack)
   {
      double cx = com.getX(), cy = com.getY(), cz = com.getZ();
      double c2 = cx * cx + cy * cy + cz * cz;
      double eps = mass * 1.0e-3; // small isotropic about-CoM inertia (~3 cm radius) for strict positive-definiteness
      momentOfInertiaAboutOriginToPack.set(mass * (c2 - cx * cx) + eps, -mass * cx * cy, -mass * cx * cz,
                                           -mass * cx * cy, mass * (c2 - cy * cy) + eps, -mass * cy * cz,
                                           -mass * cx * cz, -mass * cy * cz, mass * (c2 - cz * cz) + eps);
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

   /**
    * CoM-load coupling (degeneracy breaker): the point (in the body's inertia frame) that the body's CoM shifts
    * TOWARD as mass is added -- i.e. the payload attachment point. When non-null, the generalized-PCEKF Tikhonov
    * prior for this body targets the first-moment channels at {@code cp * (1 - exp(-2*alpha))} instead of zero:
    * at nominal mass the CoM sits at nominal, and as the (observable) mass channel alpha grows the CoM slides
    * along the nominal->cp line, reaching cp in the heavy-load limit -- exactly the combined CoM of "nominal
    * link + point payload at cp". This ties the weakly/degenerate CoM channel to the strongly-observed mass, so
    * a given observed first moment m*c resolves to a UNIQUE (m, c) rather than the mass-inflated-at-nominal
    * degenerate solution. Return {@code null} (default) for no coupling (prior targets zero, as before). Length 3.
    */
   default double[] getCoMLoadCouplingPoint(String bodyName)
   {
      return null;
   }

   double getLegMeasurementCovariance();

   double getArmMeasurementCovariance();

   double getSpineMeasurementCovariance();

   double getNormalizedInnovationThreshold();

   double[] getMaxParameterDeltaRates();

   /**
    * Two-nub force-closure grasp rejection: when true, {@link us.ihmc.commonWalkingControlModules.parameterEstimation.InertialParameterManager}
    * builds a nub contact frame per side and subtracts the INTERNAL (squeeze) component of the grasp wrench from
    * the measurement, so the squeeze is not mis-attributed to the forearm inertial parameters. Default off.
    */
   default boolean isGraspRejectionEnabled()
   {
      return false;
   }

   /** KNOWN (Stage A, prescribed squeeze) vs JACOBIAN (Stage B, force backed out of arm torques). */
   default GraspRejectionMethod getGraspRejectionMethod()
   {
      return GraspRejectionMethod.KNOWN;
   }

   /**
    * The nub-end contact point for grasp rejection, expressed in the hand ({@code getHand(side)}) link frame
    * (frame-after-joint). The manager builds a fixed-offset nub frame there for the contact Jacobian. Return
    * {@code null} (default) to disable the nub contact for that side.
    */
   default us.ihmc.euclid.tuple3D.Vector3D getNubContactOffsetInHandFrame(RobotSide side)
   {
      return null;
   }

   /** Stage A: the prescribed squeeze magnitude lambda (N). Should match the testbed's commanded squeeze. */
   default double getKnownGraspSqueeze()
   {
      return 0.0;
   }

   /** Stage B: damping (sigma^2) for the damped-least-squares arm-Jacobian force back-out. */
   default double getGraspJacobianDampedLeastSquaresLambda()
   {
      return 1.0e-4;
   }

   /** Stage B: manipulability gate sqrt(det(J J^T)); below this the force back-out is frozen/skipped. */
   default double getGraspJacobianSingularityThreshold()
   {
      return 1.0e-3;
   }
}
