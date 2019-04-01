package us.ihmc.communication;

import com.google.common.base.CaseFormat;

public class ROS2ModuleIdentifier
{
   public static final String NODE_NAME_PREFIX = "ihmc_";

   private final String nodeName;
   private final String topicQualifier;

   public ROS2ModuleIdentifier(Class<?> clazz, String topicQualifier)
   {
      this(deriveNodeName(clazz), topicQualifier);
   }

   public ROS2ModuleIdentifier(String nodeName, String topicQualifier)
   {
      this.nodeName = nodeName;
      this.topicQualifier = topicQualifier;
   }

   public static String deriveNodeName(Class<?> clazz)
   {
      return NODE_NAME_PREFIX + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, clazz.getSimpleName());
   }

   public String getNodeName()
   {
      return nodeName;
   }

   public String getTopicQualifier()
   {
      return topicQualifier;
   }
}
