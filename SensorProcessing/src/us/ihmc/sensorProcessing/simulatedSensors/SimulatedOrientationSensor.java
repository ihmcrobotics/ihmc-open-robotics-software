package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Matrix3d;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptorHolder;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SimulatedOrientationSensor extends SignalCorruptorHolder<FrameOrientation> implements ControlFlowElement
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame measurementFrame;

   private final Matrix3d rotationMatrix = new Matrix3d();
   private final FrameOrientation orientation = new FrameOrientation(worldFrame);

   private final ControlFlowOutputPort<FrameOrientation> orientationOutputPort = new ControlFlowOutputPort<FrameOrientation>(this);

   public SimulatedOrientationSensor(ReferenceFrame measurementFrame)
   {
      this.measurementFrame = measurementFrame;
   }

   public void startComputation()
   {
      measurementFrame.getTransformToDesiredFrame(worldFrame).get(rotationMatrix);
      orientation.set(worldFrame, rotationMatrix);
      corrupt(orientation);
      orientationOutputPort.setData(orientation);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<FrameOrientation> getOrientationOutputPort()
   {
      return orientationOutputPort;
   }
}
