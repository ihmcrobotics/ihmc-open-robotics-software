package us.ihmc.communication.crdt;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class RequestConfirmFreezableTest
{
   @Test
   public void test()
   {
      int maxFreezeDuration = 30;
      RequestConfirmFreezable operatorNode = createOperatorNode(maxFreezeDuration);
      RequestConfirmFreezable robotNode = createRobotNode(maxFreezeDuration);

      Assertions.assertFalse(operatorNode.isFrozen());
      Assertions.assertFalse(robotNode.isFrozen());

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
