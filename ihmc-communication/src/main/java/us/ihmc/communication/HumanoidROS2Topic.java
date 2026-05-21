package us.ihmc.communication;

import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Topic;

import java.util.Objects;

/**
 * IHMC structured ROS 2 topic builder matching the legacy ihmc-ros2-library {@code ROS2Topic} API.
 * <p>
 * Topic names are assembled from fixed slots (not plain append):
 * {@code prefix + robotName + moduleName + ioQualifier + typeName + suffix}
 * <p>
 * Example: {@code /ihmc/atlas/toolbox/ik/input/arm_trajectory}
 */
public class HumanoidROS2Topic<T extends ROS2Message<T>> extends ROS2Topic<T>
{
   public static final String INPUT = "input";
   public static final String OUTPUT = "output";

   private final String prefix;
   private final String robotName;
   private final String moduleName;
   private final String ioQualifier;
   private final String typeName;
   private final String suffix;

   public HumanoidROS2Topic()
   {
      this("", "", "", "", "", "", null);
   }

   private HumanoidROS2Topic(String prefix,
                             String robotName,
                             String moduleName,
                             String ioQualifier,
                             String typeName,
                             String suffix,
                             Class<T> messageType)
   {
      super(assembleName(prefix, robotName, moduleName, ioQualifier, typeName, suffix), messageType);
      this.prefix = prefix;
      this.robotName = robotName;
      this.moduleName = moduleName;
      this.ioQualifier = ioQualifier;
      this.typeName = typeName;
      this.suffix = suffix;
   }

   private static String assembleName(String prefix,
                                      String robotName,
                                      String moduleName,
                                      String ioQualifier,
                                      String typeName,
                                      String suffix)
   {
      return prefix + robotName + moduleName + ioQualifier + typeName + suffix;
   }

   private HumanoidROS2Topic<T> copyIfNotEqual(String prefix,
                                               String robotName,
                                               String moduleName,
                                               String ioQualifier,
                                               String typeName,
                                               String suffix,
                                               Class<T> messageType)
   {
      if (Objects.equals(this.prefix, prefix) && Objects.equals(this.robotName, robotName) && Objects.equals(this.moduleName, moduleName)
          && Objects.equals(this.ioQualifier, ioQualifier) && Objects.equals(this.typeName, typeName) && Objects.equals(this.suffix, suffix)
          && Objects.equals(getType(), messageType))
      {
         return this;
      }

      return new HumanoidROS2Topic<>(prefix, robotName, moduleName, ioQualifier, typeName, suffix, messageType);
   }

   public HumanoidROS2Topic<T> withPrefix(String prefix)
   {
      return copyIfNotEqual(HumanoidROS2TopicNameTools.processTopicNamePart(prefix),
                            robotName,
                            moduleName,
                            ioQualifier,
                            typeName,
                            suffix,
                            getType());
   }

   public HumanoidROS2Topic<T> withRobot(String robotName)
   {
      return copyIfNotEqual(prefix,
                            HumanoidROS2TopicNameTools.processTopicNamePart(robotName),
                            moduleName,
                            ioQualifier,
                            typeName,
                            suffix,
                            getType());
   }

   public HumanoidROS2Topic<T> withModule(String moduleName)
   {
      return copyIfNotEqual(prefix,
                            robotName,
                            HumanoidROS2TopicNameTools.processTopicNamePart(moduleName),
                            ioQualifier,
                            typeName,
                            suffix,
                            getType());
   }

   public HumanoidROS2Topic<T> withInput()
   {
      return withIOQualifier(INPUT);
   }

   public HumanoidROS2Topic<T> withOutput()
   {
      return withIOQualifier(OUTPUT);
   }

   public HumanoidROS2Topic<T> withIOQualifier(String ioQualifier)
   {
      return copyIfNotEqual(prefix,
                            robotName,
                            moduleName,
                            HumanoidROS2TopicNameTools.processTopicNamePart(ioQualifier),
                            typeName,
                            suffix,
                            getType());
   }

   public HumanoidROS2Topic<T> withTypeName()
   {
      if (getType() == null)
         throw new RuntimeException("This topic does not have a type yet. Cannot add type name");

      return copyIfNotEqual(prefix,
                            robotName,
                            moduleName,
                            ioQualifier,
                            HumanoidROS2TopicNameTools.messageTypeToTopicNamePart(getType()),
                            suffix,
                            getType());
   }

   public HumanoidROS2Topic<T> clearTypeName()
   {
      return copyIfNotEqual(prefix, robotName, moduleName, ioQualifier, "", suffix, getType());
   }

   public HumanoidROS2Topic<T> withSuffix(String suffix)
   {
      return copyIfNotEqual(prefix,
                            robotName,
                            moduleName,
                            ioQualifier,
                            typeName,
                            HumanoidROS2TopicNameTools.processTopicNamePart(suffix),
                            getType());
   }

   public <U extends ROS2Message<U>> HumanoidROS2Topic<U> withType(Class<U> messageType)
   {
      return withTypeName(messageType);
   }

   public <U extends ROS2Message<U>> HumanoidROS2Topic<U> withTypeName(Class<U> messageType)
   {
      if (messageType == null)
         throw new RuntimeException("Cannot change the type of a topic to null");

      if (getType() != null && !Objects.equals(messageType, getType()))
         throw new RuntimeException("Cannot change the type of a topic after it's already been set");

      @SuppressWarnings("unchecked")
      HumanoidROS2Topic<U> typedTopic = (HumanoidROS2Topic<U>) copyIfNotEqual(prefix,
                                                                               robotName,
                                                                               moduleName,
                                                                               ioQualifier,
                                                                               HumanoidROS2TopicNameTools.messageTypeToTopicNamePart(messageType),
                                                                               suffix,
                                                                               (Class<T>) messageType);
      return typedTopic;
   }

   /**
    * Fill empty slots from another topic (same semantics as legacy {@code withTopic}).
    */
   public HumanoidROS2Topic<T> withTopic(HumanoidROS2Topic<?> topic)
   {
      String newPrefix = takeSecondIfFirstEmpty(prefix, topic.prefix);
      String newRobotName = takeSecondIfFirstEmpty(robotName, topic.robotName);
      String newModuleName = takeSecondIfFirstEmpty(moduleName, topic.moduleName);
      String newIOQualifier = takeSecondIfFirstEmpty(ioQualifier, topic.ioQualifier);
      String newTypeName = takeSecondIfFirstEmpty(typeName, topic.typeName);
      String newSuffix = takeSecondIfFirstEmpty(suffix, topic.suffix);

      if (topic.getType() != null && !topic.getType().equals(getType()))
         throw new RuntimeException("Cannot change the type of a topic with the withTopic method");

      return copyIfNotEqual(newPrefix, newRobotName, newModuleName, newIOQualifier, newTypeName, newSuffix, getType());
   }

   private static String takeSecondIfFirstEmpty(String first, String second)
   {
      return first.isEmpty() ? second : first;
   }

   public String getPrefix()
   {
      return prefix;
   }

   public String getRobotName()
   {
      return robotName;
   }

   public String getModuleName()
   {
      return moduleName;
   }

   public String getIOQualifier()
   {
      return ioQualifier;
   }

   public String getTypeNamePart()
   {
      return typeName;
   }

   public String getSuffixPart()
   {
      return suffix;
   }

   @Override
   public String toString()
   {
      return getName();
   }
}
