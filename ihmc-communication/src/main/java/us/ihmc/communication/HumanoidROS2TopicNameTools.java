package us.ihmc.communication;

import org.apache.commons.lang3.StringUtils;

/**
 * Topic name helpers ported from ihmc-ros2-library {@code ROS2TopicNameTools}.
 */
public final class HumanoidROS2TopicNameTools
{
   private HumanoidROS2TopicNameTools()
   {
   }

   public static String removeMessageOrPacketSuffixFromTypeName(Class<?> messageType)
   {
      return removeMessageOrPacketSuffixFromTypeName(messageType.getSimpleName());
   }

   public static String removeMessageOrPacketSuffixFromTypeName(String simpleTypeName)
   {
      String suffixRemoved = simpleTypeName;
      suffixRemoved = StringUtils.removeEnd(suffixRemoved, "Packet");
      suffixRemoved = StringUtils.removeEnd(suffixRemoved, "Message");
      return suffixRemoved;
   }

   public static String toROSTopicFormat(String pascalCased)
   {
      if (pascalCased == null)
         return null;

      pascalCased = pascalCased.trim();

      if (pascalCased.isEmpty())
         return pascalCased;

      if (pascalCased.length() == 1)
         return pascalCased.toLowerCase();

      StringBuilder stringBuilder = new StringBuilder();

      boolean isNewWord = true;
      boolean isPreviousUpper = false;

      for (int charIndex = 0; charIndex < pascalCased.length(); charIndex++)
      {
         boolean isCharUpper = Character.isUpperCase(pascalCased.charAt(charIndex));

         if (charIndex == 0 || !isCharUpper)
            isNewWord = false;
         else if (!isPreviousUpper)
            isNewWord = true;
         else
         {
            int nextIndex = charIndex + 1;
            boolean isNextUpper = nextIndex == pascalCased.length() || Character.isUpperCase(pascalCased.charAt(nextIndex));
            isNewWord = !isNextUpper;
         }

         isPreviousUpper = isCharUpper;

         if (isNewWord)
            stringBuilder.append("_");

         stringBuilder.append(Character.toLowerCase(pascalCased.charAt(charIndex)));
      }
      return stringBuilder.toString();
   }

   public static String messageTypeToTopicNamePart(Class<?> messageType)
   {
      if (messageType == null)
         return "";

      String messageTypePart = removeMessageOrPacketSuffixFromTypeName(messageType);
      return processTopicNamePart(toROSTopicFormat(messageTypePart));
   }

   public static String processTopicNamePart(String string)
   {
      if (string == null)
         return "";

      string = string.trim().toLowerCase();

      if (string.isEmpty())
         return "";

      if (!string.startsWith("/"))
         string = "/" + string;

      return string;
   }
}
