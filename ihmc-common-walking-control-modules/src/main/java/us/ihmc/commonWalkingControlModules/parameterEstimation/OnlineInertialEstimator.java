package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Any inertial estimator intended for online use inside {@link InertialParameterManager} should implement this interface.
 */
public interface OnlineInertialEstimator
{
   DMatrix calculateEstimate(DMatrix observation);

   /**
    * Re-seeds the estimator to the nominal model and the covariance it was constructed with, discarding everything
    * it has learned. Intended as a recovery action if the estimator diverges.
    */
   void reset();

   /**
    * Hold the parameters where they are: the estimator still computes its measurement residual, but does not
    * update its state or covariance. Used to observe the residual at a KNOWN parameter value so that an additive
    * measurement bias can be identified -- see {@code tareProcess} in {@link InertialParameterManager}.
    */
   void setParametersHeld(boolean parametersHeld);

   boolean isParametersHeld();

   /** The current estimate, in the Pi basis. */
   DMatrix getState();

   DMatrix getMeasurementResidual();

   void setRegressor(DMatrix regressor);

   void setTorqueFromNominal(DMatrix torqueFromNominal);

   void setTorqueFromBias(DMatrix torqueFromBias);

   void setContactJacobians(SideDependentList<DMatrixRMaj> jacobians);

   void setContactWrenches(SideDependentList<DMatrixRMaj> wrenches);

   /** Hand/nub contact Jacobians for grasp rejection (a second contact channel). See {@link #setHandContactWrenches}. */
   void setHandContactJacobians(SideDependentList<DMatrixRMaj> jacobians);

   /**
    * Hand/nub contact wrenches for grasp rejection: only the INTERNAL (squeeze / grasp-map null-space) component
    * of the two-nub grasp wrench, expressed in the same frame as the hand contact Jacobians. Subtracted from the
    * measurement exactly like the foot contacts so the squeeze is not attributed to the forearm inertial parameters.
    */
   void setHandContactWrenches(SideDependentList<DMatrixRMaj> wrenches);

   void setProcessCovariance(DMatrix processCovariance);

   void setMeasurementCovariance(DMatrix measurementCovariance);

   double getNormalizedInnovation();

   void setNormalizedInnovationThreshold(double threshold);
}
