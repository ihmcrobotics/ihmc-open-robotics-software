package us.ihmc.sensorProcessing.controlFlowPorts;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;

public class YoFrameQuaternionControlFlowOutputPort extends ControlFlowOutputPort<FrameOrientation>
{
   private final YoFrameQuaternion yoFrameQuaternion;

   public YoFrameQuaternionControlFlowOutputPort(ControlFlowElement controlFlowElement, String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      yoFrameQuaternion = new YoFrameQuaternion(namePrefix, frame, registry);
      super.setData(new FrameOrientation(frame));
   }

   @Override
   public FrameOrientation getData()
   {
      yoFrameQuaternion.getFrameOrientationIncludingFrame(super.getData());

      return super.getData();
   }

   @Override
   public void setData(FrameOrientation data)
   {
      yoFrameQuaternion.set(data);
   }
}
