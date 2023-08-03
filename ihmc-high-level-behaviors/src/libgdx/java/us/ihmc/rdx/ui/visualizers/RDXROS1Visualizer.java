package us.ihmc.rdx.ui.visualizers;

import us.ihmc.utilities.ros.RosNodeInterface;

@Deprecated
public abstract class RDXROS1Visualizer extends RDXVisualizer implements RDXROS1VisualizerInterface
{
   private boolean currentlySubscribed = false;

   public RDXROS1Visualizer(String title)
   {
      super(title + " (ROS 1)");
   }

   public abstract void subscribe(RosNodeInterface ros1Node);

   public abstract void unsubscribe(RosNodeInterface ros1Node);

   public void updateSubscribers(RosNodeInterface ros1Node)
   {
      boolean active = isActive();
      if (active != currentlySubscribed)
      {
         if (active)
         {
            subscribe(ros1Node);
         }
         else
         {
            unsubscribe(ros1Node);
         }
      }
      currentlySubscribed = active;
   }
}
