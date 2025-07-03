package us.ihmc.avatar.abilityHand;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.psyonicros2.AbilityHandInterface;
import us.ihmc.psyonicros2.AbilityHandROS2API;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

public class ROS2AbilityHandStatus
{
   private final float[] currentPosition =  new float[AbilityHandInterface.ACTUATOR_COUNT];
   public ROS2AbilityHandStatus(ROS2Node ros2Node, String robotName, RobotSide handSide)
   {
      ROS2Tools.createVolatileCallbackSubscription(ros2Node, AbilityHandROS2API.STATE_TOPIC, abilityHandState ->
      {
         System.arraycopy(abilityHandState.getActuatorPositions(), 0, currentPosition, 0, AbilityHandInterface.ACTUATOR_COUNT);
      });
   }

   public float[] getCurrentPosition()
   {
      return currentPosition;
   }
}
