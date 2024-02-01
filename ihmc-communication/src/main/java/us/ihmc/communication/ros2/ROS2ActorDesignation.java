package us.ihmc.communication.ros2;

/**
 * An enum for the designation of an actor, usually, the
 * robot or operator, for the purposes of ROS 2 communication.
 *
 * See {@link ROS2IOTopicQualifier}.
 * See {@link CRDTActorDesignation}.
 */
public enum ROS2ActorDesignation
{
   ROBOT(ROS2IOTopicQualifier.COMMAND),
   OPERATOR(ROS2IOTopicQualifier.STATUS);

   private final ROS2IOTopicQualifier incomingQualifier;
   private final ROS2IOTopicQualifier outgoingQualifier;

   ROS2ActorDesignation(ROS2IOTopicQualifier incomingQualifier)
   {
      this.incomingQualifier = incomingQualifier;
      this.outgoingQualifier = incomingQualifier.getOpposite();
   }

   public ROS2IOTopicQualifier getIncomingQualifier()
   {
      return incomingQualifier;
   }

   public ROS2IOTopicQualifier getOutgoingQualifier()
   {
      return outgoingQualifier;
   }
}
