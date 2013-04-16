package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface OrientationEstimatorWithPorts extends OrientationEstimator
{
   public abstract ControlFlowInputPort<FrameVector> getDesiredAngularAccelerationInputPort();
   public abstract ControlFlowInputPort<FrameVector> getDesiredCenterOfMassAccelerationInputPort();
}
