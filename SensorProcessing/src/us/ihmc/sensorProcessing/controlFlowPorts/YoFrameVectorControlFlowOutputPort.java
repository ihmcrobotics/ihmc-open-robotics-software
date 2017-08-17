package us.ihmc.sensorProcessing.controlFlowPorts;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;


public class YoFrameVectorControlFlowOutputPort extends ControlFlowOutputPort<FrameVector3D>
{
   private final YoFrameVector yoFrameVector;

   public YoFrameVectorControlFlowOutputPort(ControlFlowElement controlFlowElement, String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      yoFrameVector = new YoFrameVector(namePrefix, frame, registry);
      super.setData(new FrameVector3D(frame));
   }

   @Override
   public FrameVector3D getData()
   {
      yoFrameVector.getFrameTuple(super.getData());

      return super.getData();
   }

   @Override
   public void setData(FrameVector3D data)
   {
      yoFrameVector.set(data);
   }
}
