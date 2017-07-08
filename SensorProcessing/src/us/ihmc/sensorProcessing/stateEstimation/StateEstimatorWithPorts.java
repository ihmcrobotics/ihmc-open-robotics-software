package us.ihmc.sensorProcessing.stateEstimation;

import java.util.List;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;

public interface StateEstimatorWithPorts extends StateEstimator
{
   public abstract ControlFlowGraph getControlFlowGraph();
   public abstract ControlFlowInputPort<FrameVector3D> getDesiredAngularAccelerationInputPort();
   public abstract ControlFlowInputPort<FrameVector3D> getDesiredCenterOfMassAccelerationInputPort();
   public abstract ControlFlowInputPort<List<PointPositionDataObject>> getPointPositionInputPort();
   public abstract ControlFlowInputPort<List<PointVelocityDataObject>> getPointVelocityInputPort();
}
