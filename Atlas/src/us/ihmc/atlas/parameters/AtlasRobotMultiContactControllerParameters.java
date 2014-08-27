package us.ihmc.atlas.parameters;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.neck_ry;


public class AtlasRobotMultiContactControllerParameters extends AtlasWalkingControllerParameters
{
   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[] {jointNames[neck_ry]}; 
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[] {jointNames[back_bkz], jointNames[back_bkx], jointNames[back_bky]};
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return null;
   }

}
