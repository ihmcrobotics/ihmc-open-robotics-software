package us.ihmc.communication.crdt;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import test_msgs.msg.dds.StampedAlphabet;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

public class AlphabetCRDTTest
{
   @Test
   /**
    * In this test, we assume that without sleeping, actions happen instantaneously.
    * We should not try to assert anything based on race conditions that are just
    * thread scheduling on the local CPU.
    */
   public void testAlphabetCRDT()
   {
      // Synchronize the update of the ground truth, so we have something to assert
      Object groundTruthSyncObject = new Object();
      StringBuilder groundTruth = new StringBuilder();

      assertGroundTruth("ground truth before anything", "", groundTruth, groundTruthSyncObject);

      // Simulate messaging
      Object messagingSyncObject = new Object();
      StampedAlphabet aToBMessage = new StampedAlphabet();
      StampedAlphabet bToAMessage = new StampedAlphabet();

      Alphabet processAData = new Alphabet();
      Alphabet processBData = new Alphabet();

      // Thread is entity B
      ThreadTools.startAThread(() ->
      {
         AlphabetCRDT processBCRDT = new AlphabetCRDT(processBData, aToBMessage, bToAMessage);

         updateCRDT(messagingSyncObject, processBCRDT);

         assertAlphabet("process B after update", "", processBData);

         addLetter(groundTruthSyncObject, processBData, groundTruth, "a");
         assertAlphabet("process A data after letter added in process B", "", processAData);
         assertAlphabet("process B data after letter added in process B", "a", processBData);
         assertGroundTruth("ground truth data after letter added in process B", "a", groundTruth, groundTruthSyncObject);

         updateCRDT(messagingSyncObject, processBCRDT);

         ThreadTools.sleep(200);

         assertAlphabet("process A data", "abcd", processAData);
         assertAlphabet("process B data", "a", processBData);
         assertGroundTruth("ground truth data", "abcd", groundTruth, groundTruthSyncObject);
         updateCRDT(messagingSyncObject, processBCRDT);
         assertAlphabet("process A data", "abcd", processAData);
         assertAlphabet("process B data", "abcd", processBData);
         assertGroundTruth("ground truth data", "abcd", groundTruth, groundTruthSyncObject);


         addLetter(groundTruthSyncObject, processBData, groundTruth, "e");
         addLetter(groundTruthSyncObject, processBData, groundTruth, "f");
         updateCRDT(messagingSyncObject, processBCRDT);


      }, "EntityBThread");

      AlphabetCRDT processACRDT = new AlphabetCRDT(processAData, bToAMessage, aToBMessage);

      ThreadTools.sleep(50);

      assertAlphabet("process A data", "", processAData);
      assertAlphabet("process B data", "a", processBData);
      assertGroundTruth("ground truth data", "a", groundTruth, groundTruthSyncObject);

      updateCRDT(messagingSyncObject, processACRDT);

      assertAlphabet("process A data", "a", processAData);
      assertAlphabet("process B data", "a", processBData);
      assertGroundTruth("ground truth data", "a", groundTruth, groundTruthSyncObject);

      addLetter(groundTruthSyncObject, processAData, groundTruth, "b");
      assertAlphabet("process A data", "ab", processAData);
      assertAlphabet("process B data", "a", processBData);
      assertGroundTruth("ground truth data", "ab", groundTruth, groundTruthSyncObject);

      updateCRDT(messagingSyncObject, processACRDT);

      ThreadTools.sleep(100);

      addLetter(groundTruthSyncObject, processAData, groundTruth, "c");
      addLetter(groundTruthSyncObject, processAData, groundTruth, "d");
      updateCRDT(messagingSyncObject, processACRDT);

      ThreadTools.sleep(100);

      assertAlphabet("process A data", "abcd", processAData);
      assertAlphabet("process B data", "abcdef", processBData);
      assertGroundTruth("ground truth data", "abcdef", groundTruth, groundTruthSyncObject);
      updateCRDT(messagingSyncObject, processACRDT);
      assertAlphabet("process A data", "abcdef", processAData);
      assertAlphabet("process B data", "abcdef", processBData);
      assertGroundTruth("ground truth data", "abcdef", groundTruth, groundTruthSyncObject);
   }

   private static void updateCRDT(Object messageSyncObject, LatestModificationCRDTAlgorithm<Alphabet> crdtAlgorithm)
   {
      synchronized (messageSyncObject)
      {
         crdtAlgorithm.update();
      }
   }

   private static void addLetter(Object groundTruthSyncObject, Alphabet localAlphabet, StringBuilder groundTruthAlphabet, String letter)
   {
      localAlphabet.addLetter(letter);
      localAlphabet.markDataModified();
      synchronized (groundTruthSyncObject)
      {
         groundTruthAlphabet.append(letter);
      }
   }

   private void assertAlphabet(String debugMessage, String expected, Alphabet alphabet)
   {
      String alphabetString = alphabet.getAlphabet();
      LogTools.info("Expecting %s: %s Actual: %s".formatted(debugMessage, expected, alphabetString));
      Assertions.assertEquals(expected, alphabetString);
   }

   private void assertGroundTruth(String debugMessage, String expected, StringBuilder groundTruthAlphabet, Object groundTruthSyncObject)
   {
      synchronized (groundTruthSyncObject)
      {
         String groundTruthAlphabetString = groundTruthAlphabet.toString();
         LogTools.info("Expecting %s: %s Actual: %s".formatted(debugMessage, expected, groundTruthAlphabetString));
         Assertions.assertEquals(expected, groundTruthAlphabetString);
      }
   }
}
