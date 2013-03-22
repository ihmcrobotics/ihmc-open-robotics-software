package us.ihmc.commonWalkingControlModules.stateEstimation;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface OrientationEstimator
{
   public abstract FrameOrientation getEstimatedOrientation();

   public abstract FrameVector getEstimatedAngularVelocity();

   public abstract ControlFlowInputPort<FrameVector> getAngularAccelerationInputPort();

   public abstract DenseMatrix64F getCovariance();

   public abstract DenseMatrix64F getState();

   public abstract void setState(DenseMatrix64F x, DenseMatrix64F covariance);

   public abstract void setAngularAccelerationNoiseCovariance(DenseMatrix64F angularAccelerationNoiseCovariance);
}
