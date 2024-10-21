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

   DMatrix getMeasurementResidual();

   void setRegressor(DMatrix regressor);

   void setTorqueFromNominal(DMatrix torqueFromNominal);

   void setTorqueFromBias(DMatrix torqueFromBias);

   void setContactJacobians(SideDependentList<DMatrixRMaj> jacobians);

   void setContactWrenches(SideDependentList<DMatrixRMaj> wrenches);

   void setProcessCovariance(DMatrix processCovariance);

   void setMeasurementCovariance(DMatrix measurementCovariance);

   double getNormalizedInnovation();

   void setNormalizedInnovationThreshold(double threshold);
}
