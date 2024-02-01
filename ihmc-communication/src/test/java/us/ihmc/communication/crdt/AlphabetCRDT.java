package us.ihmc.communication.crdt;

import test_msgs.msg.dds.StampedAlphabet;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.log.LogTools;

import java.time.Instant;

public class AlphabetCRDT extends LatestModificationCRDTAlgorithm<Alphabet>
{
   public AlphabetCRDT(Alphabet data, StampedAlphabet inputMessage, StampedAlphabet outputMessage)
   {
      super(data::getDataIsModified,
            data::clearDataModified,
            instant -> packMessage(instant, data, outputMessage),
            () -> MessageTools.toInstant(inputMessage.getLastModified()),
            () -> copyMessageToData(inputMessage, data));
   }

   private static void packMessage(Instant lastModified, Alphabet data, StampedAlphabet message)
   {
      LogTools.info("Packing message: {} modified: {}", data.getAlphabet(), lastModified);
      message.getAlphabet().setLength(0);
      message.getAlphabet().append(data.getAlphabet());
      MessageTools.toMessage(lastModified, message.getLastModified());
   }

   private static void copyMessageToData(StampedAlphabet message, Alphabet data)
   {
      String previous = data.getAlphabet();
      data.setAlphabet(message.getAlphabetAsString());
      LogTools.info("Copying message: message {} -> {} -> {}", message.getAlphabetAsString(), previous, data.getAlphabet());
   }
}
