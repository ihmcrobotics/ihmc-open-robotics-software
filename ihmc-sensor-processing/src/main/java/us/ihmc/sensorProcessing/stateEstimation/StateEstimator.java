package us.ihmc.sensorProcessing.stateEstimation;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface StateEstimator
{
   public abstract void getEstimatedOrientation(FrameQuaternion frameOrientationToPack);
   
   public abstract void setEstimatedOrientation(FrameQuaternion estimatedOrientation);

   public abstract void getEstimatedAngularVelocity(FrameVector3D estimatedAngularVelocityToPack);

   public abstract void setEstimatedAngularVelocity(FrameVector3D estimatedAngularVelocity);
   
   public abstract void getEstimatedCoMPosition(FramePoint3D estimatedCoMPositionToPack);

   public abstract void setEstimatedCoMPosition(FramePoint3D estimatedCoMPosition);

   public abstract void getEstimatedCoMVelocity(FrameVector3D estimatedCoMVelocityToPack);

   public abstract void setEstimatedCoMVelocity(FrameVector3D estimatedCoMVelocity);

   public abstract void getEstimatedPelvisPosition(FramePoint3D estimatedPelvisPositionToPack);

   public abstract void getEstimatedPelvisLinearVelocity(FrameVector3D estimatedPelvisLinearVelocityToPack);

   public abstract DenseMatrix64F getCovariance();

   public abstract DenseMatrix64F getState();

   public abstract void setState(DenseMatrix64F x, DenseMatrix64F covariance);

   public abstract void initializeOrientationEstimateToMeasurement();
}
