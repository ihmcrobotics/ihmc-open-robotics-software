package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2MultiBodyGraphic extends RDXROS2SingleTopicVisualizer
{
   private final RDXMultiBodyGraphic multiBodyGraphic;
   private final ROS2Topic<?> topic;

   public RDXROS2MultiBodyGraphic(String title, ROS2Topic<?> topic)
   {
      super(title);
      multiBodyGraphic = new RDXMultiBodyGraphic(title);
      this.topic = topic;
   }

   @Override
   public void renderImGuiWidgets()
   {

   }

   @Override
   public ROS2Topic<?> getTopic()
   {
      return topic;
   }

   public RDXMultiBodyGraphic getMultiBodyGraphic()
   {
      return multiBodyGraphic;
   }
}
