package us.ihmc.rdx.ui.tools;

import us.ihmc.log.LogTools;
import us.ihmc.pubsub.attributes.CommonAttributes;

public class PubSubStatsTools
{
   public static String getHumanReadableByteSize(long numberOfBytes)
   {
      if (numberOfBytes < 1000)
         return "%d B".formatted(numberOfBytes);
      else if (numberOfBytes < 1000000)
         return "%.2f kB".formatted(numberOfBytes / 1000.0);
      else
         return "%.2f MB".formatted(numberOfBytes / 1000000.0);
   }

   public static String getHumanReadableBitSize(long numberOfBytes)
   {
      long numberOfBits = numberOfBytes * 8;

      if (numberOfBits < 1000)
         return "%d b".formatted(numberOfBits);
      else if (numberOfBits < 1000000)
         return "%.2f kb".formatted(numberOfBits / 1000.0);
      else
         return "%.2f Mb".formatted(numberOfBits / 1000000.0);
   }

   public static void printLargePayloadWarning(CommonAttributes attributes, int payloadSize)
   {
      LogTools.warn(1, "Message payload is high for topic: %s Type: %s Size: %s"
                          .formatted(attributes.getHumanReadableTopicName(),
                                     attributes.getHumanReadableTopicDataTypeName(),
                                     getHumanReadableByteSize(payloadSize)));
   }
}
