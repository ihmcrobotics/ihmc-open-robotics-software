package us.ihmc.communication;

import com.google.common.base.CaseFormat;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;

public class ROS2ModuleIdentifier
{
   public static final String NODE_NAME_PREFIX = "ihmc_";

   private final String moduleNodeName;
   private final String moduleTopicQualifier;

   public ROS2ModuleIdentifier(Class<?> clazz, String moduleTopicQualifier)
   {
      this(deriveModuleNodeName(clazz), moduleTopicQualifier);
   }

   public ROS2ModuleIdentifier(String moduleNodeName, String moduleTopicQualifier)
   {
      this.moduleNodeName = moduleNodeName;
      this.moduleTopicQualifier = moduleTopicQualifier;
   }

   public ROS2TopicQualifier deriveIOTopicQualifierForSubscriber(String localNodeName)
   {
      return localNodeName.endsWith(moduleNodeName) ? ROS2TopicQualifier.INPUT : ROS2TopicQualifier.OUTPUT;
   }

   public ROS2TopicQualifier deriveIOTopicQualifierForPublisher(String localNodeName)
   {
      return localNodeName.endsWith(moduleNodeName) ? ROS2TopicQualifier.OUTPUT : ROS2TopicQualifier.INPUT;
   }

   /**
    * Note: Only used in some cases and is not a general or required convention
    */
   public static String deriveModuleNodeName(Class<?> clazz)
   {
      return NODE_NAME_PREFIX + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, clazz.getSimpleName());
   }

   public String getNodeName()
   {
      return moduleNodeName;
   }

   public String getModuleTopicQualifier()
   {
      return moduleTopicQualifier;
   }

   public ROS2ModuleIdentifier qualifyMore(String moduleTopicQualifierPostfix)
   {
      return new ROS2ModuleIdentifier(moduleNodeName, moduleTopicQualifier + moduleTopicQualifierPostfix);
   }
}
