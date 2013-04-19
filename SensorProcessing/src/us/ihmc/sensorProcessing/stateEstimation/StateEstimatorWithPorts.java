package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface StateEstimatorWithPorts extends StateEstimator
{
   public abstract ControlFlowGraph getControlFlowGraph();
   public abstract ControlFlowInputPort<FrameVector> getDesiredAngularAccelerationInputPort();
   public abstract ControlFlowInputPort<FrameVector> getDesiredCenterOfMassAccelerationInputPort();   
}
