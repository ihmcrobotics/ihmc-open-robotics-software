package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class UserDesiredHeadOrientationProvider extends DesiredHeadOrientationProvider
{
   private final DoubleYoVariable userDesiredHeadPitch, userDesiredHeadYaw;
   private final ReferenceFrame headOrientationFrame;
   
   public UserDesiredHeadOrientationProvider(ReferenceFrame headOrientationFrame, YoVariableRegistry registry)
   {
      super(headOrientationFrame);

      this.headOrientationFrame = headOrientationFrame;
      
      userDesiredHeadPitch = new DoubleYoVariable("userDesiredHeadPitch", registry);
      userDesiredHeadYaw = new DoubleYoVariable("userDesiredHeadYaw", registry);
   }

   public double getDesiredExtendedNeckPitchJointAngle()
   {
      return userDesiredHeadPitch.getDoubleValue();
   }
   
   public boolean isNewHeadOrientationInformationAvailable()
   {
      return true;
   }

   public FrameOrientation getDesiredHeadOrientation()
   {      
      FrameOrientation frameOrientation = new FrameOrientation(headOrientationFrame, userDesiredHeadYaw.getDoubleValue(), 0.0, 0.0);
      return frameOrientation;
   }
}
