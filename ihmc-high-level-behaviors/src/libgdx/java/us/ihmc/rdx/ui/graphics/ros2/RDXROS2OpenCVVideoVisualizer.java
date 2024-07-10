package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;

public abstract class RDXROS2OpenCVVideoVisualizer<T> extends RDXROS2SingleTopicVisualizer<T>
{
   private final RDXOpenCVVideoVisualizer openCVVideoVisualizer;

   public RDXROS2OpenCVVideoVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);

      openCVVideoVisualizer = new RDXOpenCVVideoVisualizer(title, panelName, flipY);
   }

   public RDXOpenCVVideoVisualizer getOpenCVVideoVisualizer()
   {
      return openCVVideoVisualizer;
   }
}
