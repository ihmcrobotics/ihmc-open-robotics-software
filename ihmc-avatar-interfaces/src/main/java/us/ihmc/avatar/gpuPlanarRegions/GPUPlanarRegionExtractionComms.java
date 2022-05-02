package us.ihmc.avatar.gpuPlanarRegions;

import controller_msgs.msg.dds.StoredPropertySetMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class GPUPlanarRegionExtractionComms
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("gpu_planar_region_extraction");
   public static final ROS2Topic<StoredPropertySetMessage> PARAMETERS = BASE_TOPIC.withType(StoredPropertySetMessage.class).withSuffix("parameters");
   public static final ROS2Topic<StoredPropertySetMessage> CONVEX_HULL_FACTORY_PARAMETERS
         = BASE_TOPIC.withType(StoredPropertySetMessage.class).withSuffix("convex_hull_factory_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> POLYGONIZER_PARAMETERS
         = BASE_TOPIC.withType(StoredPropertySetMessage.class).withSuffix("polygonizer_parameters");

   public static final String DEBUG_EXTRACTION_IMAGE = "/gpu_planar_region_extraction/debug_extraction_image";

}
