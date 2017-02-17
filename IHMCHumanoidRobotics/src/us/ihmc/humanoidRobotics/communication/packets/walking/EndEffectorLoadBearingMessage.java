package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "This message commands the controller to start loading an end effector that was unloaded to support the robot weight. "
      + " One application is making a foot loadbearing."
      + " When the robot is performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the robot to switch back to double support."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/end_effector_load_bearing")
public class EndEffectorLoadBearingMessage extends Packet<EndEffectorLoadBearingMessage>
{
   public enum EndEffector
   {
      @RosEnumValueDocumentation(documentation = "In this case, the user also needs to provide the robotSide."
            + " If in the air, the corresponding foot will enter first a vertical ground approach transition and eventually touch the ground and switch to loadbearing."
            + " Then the robot is ready to walk."
            + " It is preferable to request a foot to switch to load bearing when it is aready close to the ground.")
      FOOT,
      @RosEnumValueDocumentation(documentation = "In this case, the user also needs to provide the robotSide."
            + " It is preferable to request a hand to switch to load bearing when it is aready close to the ground.")
      HAND;

      public static final EndEffector[] values = values();

      public boolean isRobotSideNeeded()
      {
         switch (this)
         {
         case FOOT:
         case HAND:
            return true;
         default:
            throw new RuntimeException("Should not get there.");
         }
      }
   }

   public enum LoadBearingRequest
   {
      @RosEnumValueDocumentation(documentation = "Request to load the given end-effector.")
      LOAD,
      @RosEnumValueDocumentation(documentation = "Request to unload the given end-effector.")
      UNLOAD;

      public static final LoadBearingRequest[] values = values();
   }

   @RosExportedField(documentation = "Needed to identify a side dependent end-effector.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Specifies which end-effector should be loaded/unloaded.")
   public EndEffector endEffector;
   @RosExportedField(documentation = "Wether the end-effector should be loaded or unloaded.")
   public LoadBearingRequest request;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public EndEffectorLoadBearingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public EndEffectorLoadBearingMessage(Random random)
   {
      robotSide = RandomTools.generateRandomEnum(random, RobotSide.class);
      endEffector = RandomTools.generateRandomEnum(random, EndEffector.class);
      request = RandomTools.generateRandomEnum(random, LoadBearingRequest.class);
   }
   /**
    * Create a message to request one end-effector to switch to load bearing.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide refers to the side of the end-effector if necessary.
    * @param endEffector refers to the end-effector that will switch to load bearing.
    */
   public EndEffectorLoadBearingMessage(RobotSide robotSide, EndEffector endEffector, LoadBearingRequest request)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.endEffector = endEffector;
      this.request = request;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public EndEffector getEndEffector()
   {
      return endEffector;
   }

   public LoadBearingRequest getRequest()
   {
      return request;
   }

   @Override
   public boolean epsilonEquals(EndEffectorLoadBearingMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      if (endEffector != other.endEffector)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "End effector load bearing: end-effector = " + endEffector + ", robotSide = " + robotSide;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateEndEffectorLoadBearingMessage(this);
   }
}
