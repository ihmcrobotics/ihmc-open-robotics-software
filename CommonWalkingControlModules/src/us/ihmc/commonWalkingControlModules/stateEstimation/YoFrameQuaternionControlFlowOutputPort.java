package us.ihmc.commonWalkingControlModules.stateEstimation;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;

public class YoFrameQuaternionControlFlowOutputPort extends ControlFlowOutputPort<FrameOrientation>
{
   private final YoFrameQuaternion yoFrameQuaternion;

   public YoFrameQuaternionControlFlowOutputPort(ControlFlowElement controlFlowElement, String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(controlFlowElement);
      yoFrameQuaternion = new YoFrameQuaternion(namePrefix, frame, registry);
      super.setData(new FrameOrientation(frame));
   }

   @Override
   public FrameOrientation getData()
   {
      yoFrameQuaternion.get(super.getData());

      return super.getData();
   }

   @Override
   public void setData(FrameOrientation data)
   {
      yoFrameQuaternion.set(data);
   }
}
