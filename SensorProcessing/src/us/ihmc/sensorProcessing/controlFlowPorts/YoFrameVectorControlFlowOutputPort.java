package us.ihmc.sensorProcessing.controlFlowPorts;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class YoFrameVectorControlFlowOutputPort extends ControlFlowOutputPort<FrameVector>
{
   private final YoFrameVector yoFrameVector;

   public YoFrameVectorControlFlowOutputPort(ControlFlowElement controlFlowElement, String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      yoFrameVector = new YoFrameVector(namePrefix, frame, registry);
      super.setData(new FrameVector(frame));
   }

   @Override
   public FrameVector getData()
   {
      yoFrameVector.getFrameTuple(super.getData());

      return super.getData();
   }

   @Override
   public void setData(FrameVector data)
   {
      yoFrameVector.set(data);
   }
}
