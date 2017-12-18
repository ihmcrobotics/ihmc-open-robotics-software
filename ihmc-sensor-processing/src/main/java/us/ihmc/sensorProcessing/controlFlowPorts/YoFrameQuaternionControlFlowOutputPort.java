package us.ihmc.sensorProcessing.controlFlowPorts;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;


public class YoFrameQuaternionControlFlowOutputPort extends ControlFlowOutputPort<FrameQuaternion>
{
   private final YoFrameQuaternion yoFrameQuaternion;

   public YoFrameQuaternionControlFlowOutputPort(ControlFlowElement controlFlowElement, String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      yoFrameQuaternion = new YoFrameQuaternion(namePrefix, frame, registry);
      super.setData(new FrameQuaternion(frame));
   }

   @Override
   public FrameQuaternion getData()
   {
      yoFrameQuaternion.getFrameOrientationIncludingFrame(super.getData());

      return super.getData();
   }

   @Override
   public void setData(FrameQuaternion data)
   {
      yoFrameQuaternion.set(data);
   }
}
