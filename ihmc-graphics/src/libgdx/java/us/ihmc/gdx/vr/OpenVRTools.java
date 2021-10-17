package us.ihmc.gdx.vr;

import org.lwjgl.openvr.VR;
import us.ihmc.robotics.robotSide.RobotSide;

public class OpenVRTools
{
   public static int toOpenVRSide(RobotSide side)
   {
      if (side == RobotSide.LEFT)
         return VR.EVREye_Eye_Left;
      else
         return VR.EVREye_Eye_Right;
   }
}
