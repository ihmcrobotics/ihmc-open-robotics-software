package us.ihmc.sensorProcessing.controlFlowPorts;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;


public class YoFramePointControlFlowOutputPort extends ControlFlowOutputPort<FramePoint3D>
{
   private final YoFramePoint yoFramePoint;

   public YoFramePointControlFlowOutputPort(ControlFlowElement controlFlowElement, String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      yoFramePoint = new YoFramePoint(namePrefix, frame, registry);
      super.setData(new FramePoint3D(frame));
   }

   @Override
   public FramePoint3D getData()
   {
      yoFramePoint.getFrameTuple(super.getData());

      return super.getData();
   }

   @Override
   public void setData(FramePoint3D data)
   {
      yoFramePoint.set(data);
   }
}
