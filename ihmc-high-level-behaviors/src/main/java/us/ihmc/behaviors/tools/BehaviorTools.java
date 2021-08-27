package us.ihmc.behaviors.tools;

import us.ihmc.euclid.geometry.Pose3D;

public class BehaviorTools
{
   public static Pose3D createNaNPose()
   {
      Pose3D nanPose = new Pose3D();
      nanPose.setToNaN();
      return nanPose;
   }
}
