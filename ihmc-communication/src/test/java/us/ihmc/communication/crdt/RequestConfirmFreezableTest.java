package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.ConfirmableRequestMessage;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class RequestConfirmFreezableTest
{
   @Test
   public void testOfflineFreezing()
   {
      conductOfflineFreezeTest(30);
      conductOfflineFreezeTest(100);
      conductOfflineFreezeTest(1);
   }

   private void conductOfflineFreezeTest(int maxFreezeDuration)
   {
      RequestConfirmFreezable operatorNode = createOperatorNode(maxFreezeDuration);
      RequestConfirmFreezable robotNode = createRobotNode(maxFreezeDuration);

      Assertions.assertFalse(operatorNode.isFrozen());
      Assertions.assertFalse(robotNode.isFrozen());

      // Basic local freeze and unfreeze

      operatorNode.freeze();
      Assertions.assertTrue(operatorNode.isFrozen());

      robotNode.freeze();
      Assertions.assertTrue(robotNode.isFrozen());

      operatorNode.unfreeze();
      Assertions.assertFalse(operatorNode.isFrozen());

      robotNode.unfreeze();
      Assertions.assertFalse(robotNode.isFrozen());

      // Test local timeouts

      operatorNode.freeze();
      for (int i = 0; i < maxFreezeDuration; i++)
      {
         Assertions.assertTrue(operatorNode.isFrozen());
         operatorNode.getCRDTInfo().startNextUpdate();
      }
      Assertions.assertFalse(operatorNode.isFrozen());

      robotNode.freeze();
      for (int i = 0; i < maxFreezeDuration; i++)
      {
         Assertions.assertTrue(robotNode.isFrozen());
         robotNode.getCRDTInfo().startNextUpdate();
      }
      Assertions.assertFalse(robotNode.isFrozen());
   }

   @Test
   public void testOrderedSynchronization()
   {
      int maxFreezeDuration = 30;
      RequestConfirmFreezable operatorNode = createOperatorNode(maxFreezeDuration);
      RequestConfirmFreezable robotNode = createRobotNode(maxFreezeDuration);

      ConfirmableRequestMessage message = new ConfirmableRequestMessage();

      // Test nothing weird in unfrozen operation

      for (int i = 0; i < 1000; i++)
      {
         operatorNode.toMessage(message);
         operatorNode.getCRDTInfo().startNextUpdate();
         robotNode.fromMessage(message);

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());
         Assertions.assertEquals(0, operatorNode.getNextRequestID());
         Assertions.assertEquals(0, robotNode.getNextRequestID());

         robotNode.toMessage(message);
         robotNode.getCRDTInfo().startNextUpdate();
         operatorNode.fromMessage(message);

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());
         Assertions.assertEquals(0, operatorNode.getNextRequestID());
         Assertions.assertEquals(0, robotNode.getNextRequestID());
      }

      for (int i = 0; i < 1000; i++)
      {
         operatorNode.freeze();

         operatorNode.toMessage(message);
         operatorNode.getCRDTInfo().startNextUpdate();

         Assertions.assertEquals(1, message.getRequestNumbers().size());
         Assertions.assertEquals(i, message.getRequestNumbers().get(0));
         Assertions.assertEquals(0, message.getConfirmationNumbers().size());

         robotNode.fromMessage(message);

         Assertions.assertTrue(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());
         Assertions.assertEquals(i + 1, operatorNode.getNextRequestID());
         Assertions.assertEquals(0, robotNode.getNextRequestID());

         robotNode.toMessage(message);
         robotNode.getCRDTInfo().startNextUpdate();

         Assertions.assertEquals(Math.min(maxFreezeDuration, i + 1), message.getConfirmationNumbers().size());

         for (int j = 0; j < message.getConfirmationNumbers().size(); j++)
         {
            if (i < maxFreezeDuration)
            {
               Assertions.assertEquals(j, message.getConfirmationNumbers().get(j));
            }
            else
            {
               Assertions.assertEquals(j + (i - maxFreezeDuration + 1), message.getConfirmationNumbers().get(j));
            }
         }

         Assertions.assertEquals(0, message.getRequestNumbers().size());

         operatorNode.fromMessage(message);

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());
         Assertions.assertEquals(i + 1, operatorNode.getNextRequestID());
         Assertions.assertEquals(0, robotNode.getNextRequestID());
      }
   }

   @Test
   public void testUnorderedSynchronization()
   {
      int maxFreezeDuration = 30;
      RequestConfirmFreezable operatorNode = createOperatorNode(maxFreezeDuration);
      RequestConfirmFreezable robotNode = createRobotNode(maxFreezeDuration);

      ConfirmableRequestMessage operatorToRobotMessage = new ConfirmableRequestMessage();
      ConfirmableRequestMessage robotToOperatorMessage = new ConfirmableRequestMessage();

      for (int i = 0; i < 1000; i++)
      {
         robotNode.toMessage(robotToOperatorMessage);
         robotNode.getCRDTInfo().startNextUpdate();
         operatorNode.toMessage(operatorToRobotMessage);
         operatorNode.getCRDTInfo().startNextUpdate();

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());

         robotNode.fromMessage(operatorToRobotMessage);
         operatorNode.fromMessage(robotToOperatorMessage);

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());
      }

      for (int i = 0; i < 1000; i++)
      {
         robotNode.toMessage(robotToOperatorMessage);
         robotNode.getCRDTInfo().startNextUpdate();
         operatorNode.toMessage(operatorToRobotMessage);
         operatorNode.getCRDTInfo().startNextUpdate();
         robotNode.fromMessage(operatorToRobotMessage);

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());

         operatorNode.fromMessage(robotToOperatorMessage);

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());
      }

      for (int i = 0; i < 1000; i++)
      {
         robotNode.toMessage(robotToOperatorMessage);
         robotNode.getCRDTInfo().startNextUpdate();
         robotNode.fromMessage(operatorToRobotMessage);
         operatorNode.toMessage(operatorToRobotMessage);
         operatorNode.getCRDTInfo().startNextUpdate();
         operatorNode.fromMessage(robotToOperatorMessage);

         Assertions.assertFalse(operatorNode.isFrozen());
         Assertions.assertFalse(robotNode.isFrozen());
      }

      for (int i = 0; i < 1000; i++)
      {
         operatorNode.freeze();

         for (int j = 0; j < 5; j++)
         {
            operatorNode.toMessage(operatorToRobotMessage);
            operatorNode.getCRDTInfo().startNextUpdate();
         }

         for (int j = 0; j < 3; j++)
         {
            robotNode.fromMessage(operatorToRobotMessage);
         }

         Assertions.assertTrue(operatorNode.isFrozen());

         for (int j = 0; j < 5; j++)
         {
            robotNode.toMessage(robotToOperatorMessage);
            robotNode.getCRDTInfo().startNextUpdate();
         }

         Assertions.assertTrue(operatorNode.isFrozen());

         operatorNode.fromMessage(robotToOperatorMessage);

         Assertions.assertFalse(operatorNode.isFrozen());
      }
   }

   @Test
   public void testCrazyStuff()
   {
      int maxFreezeDuration = 15;
      RequestConfirmFreezable operatorNode = createOperatorNode(maxFreezeDuration);
      RequestConfirmFreezable robotNode = createRobotNode(maxFreezeDuration);

      ConfirmableRequestMessage operatorToRobotMessage = new ConfirmableRequestMessage();
      ConfirmableRequestMessage robotToOperatorMessage = new ConfirmableRequestMessage();

      for (int i = 0; i < 1000; i++)
      {
         for (int j = 0; j < 7; j++)
         {
            operatorNode.freeze();
         }

         for (int j = 0; j < 9; j++)
         {
            operatorNode.toMessage(operatorToRobotMessage);
            operatorNode.getCRDTInfo().startNextUpdate();

            if (j % 3 == 0)
               operatorNode.freeze();
         }

         for (int j = 0; j < 3; j++)
         {
            robotNode.fromMessage(operatorToRobotMessage);
         }

         Assertions.assertTrue(operatorNode.isFrozen());

         for (int j = 0; j < 5; j++)
         {
            robotNode.toMessage(robotToOperatorMessage);
            robotNode.getCRDTInfo().startNextUpdate();

            robotNode.freeze();
         }

         Assertions.assertTrue(operatorNode.isFrozen());

         operatorNode.fromMessage(robotToOperatorMessage);

         Assertions.assertFalse(operatorNode.isFrozen());
      }
   }

   private RequestConfirmFreezable createOperatorNode(int maxFreezeDuration)
   {
      return createNode(ROS2ActorDesignation.OPERATOR, maxFreezeDuration);
   }

   private RequestConfirmFreezable createRobotNode(int maxFreezeDuration)
   {
      return createNode(ROS2ActorDesignation.ROBOT, maxFreezeDuration);
   }

   private RequestConfirmFreezable createNode(ROS2ActorDesignation actorDesignation, int maxFreezeDuration)
   {
      CRDTInfo crdtInfo = new CRDTInfo(actorDesignation, maxFreezeDuration);
      return new RequestConfirmFreezable(crdtInfo);
   }
}
