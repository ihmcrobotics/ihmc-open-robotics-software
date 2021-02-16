package us.ihmc.atlas.ros;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.ui.video.JavaFXROS2VideoViewer;

public class RealsenseD435VideoViewer
{
   public static void main(String[] args)
   {
      new JavaFXROS2VideoViewer(ROS2Tools.D435_VIDEO, 640, 480);
   }
}
