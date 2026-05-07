package us.ihmc.communication;

import us.ihmc.jros2.ROS2Topic;

public final class ActiveMappingAPI
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";
   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().prependedWith(IHMC_TOPIC_PREFIX);

   public static final String ACTIVE_MAPPING_MODULE_NAME = "active_mapping";

   public static final ROS2Topic<?> ACTIVE_MAPPING_FOOTSTEP_PLAN_OUTPUT = IHMC_ROOT.appendedWith(ACTIVE_MAPPING_MODULE_NAME)
                                                                                   .appendedWith("footstep_plan")
                                                                                   .appendedWith("output");


}
