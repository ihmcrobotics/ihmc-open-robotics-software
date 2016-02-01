package us.ihmc.sensorProcessing.stateEstimation;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;

public interface StateEstimator
{
   public abstract void getEstimatedOrientation(FrameOrientation frameOrientationToPack);
   
   public abstract void setEstimatedOrientation(FrameOrientation estimatedOrientation);

   public abstract void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack);

   public abstract void setEstimatedAngularVelocity(FrameVector estimatedAngularVelocity);
   
   public abstract void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack);

   public abstract void setEstimatedCoMPosition(FramePoint estimatedCoMPosition);

   public abstract void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack);

   public abstract void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity);

   public abstract void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack);

   public abstract void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack);

   public abstract DenseMatrix64F getCovariance();

   public abstract DenseMatrix64F getState();

   public abstract void setState(DenseMatrix64F x, DenseMatrix64F covariance);

   public abstract void initializeOrientationEstimateToMeasurement();
}
