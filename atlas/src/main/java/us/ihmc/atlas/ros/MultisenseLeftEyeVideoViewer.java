package us.ihmc.atlas.ros;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.javafx.video.JavaFXROS2VideoViewer;

public class MultisenseLeftEyeVideoViewer
{
   public static void main(String[] args)
   {
      new JavaFXROS2VideoViewer(ROS2Tools.VIDEO, 1024, 544);
   }
}
