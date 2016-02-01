package us.ihmc.sensorProcessing.controlFlowPorts;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class YoFramePointControlFlowOutputPort extends ControlFlowOutputPort<FramePoint>
{
   private final YoFramePoint yoFramePoint;

   public YoFramePointControlFlowOutputPort(ControlFlowElement controlFlowElement, String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      yoFramePoint = new YoFramePoint(namePrefix, frame, registry);
      super.setData(new FramePoint(frame));
   }

   @Override
   public FramePoint getData()
   {
      yoFramePoint.getFrameTuple(super.getData());

      return super.getData();
   }

   @Override
   public void setData(FramePoint data)
   {
      yoFramePoint.set(data);
   }
}
