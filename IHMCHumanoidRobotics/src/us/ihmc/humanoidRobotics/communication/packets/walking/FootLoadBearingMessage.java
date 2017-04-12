package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "This message commands the controller to start loading a foot that was unloaded to support the robot weight. "
      + " When the robot is performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the robot to switch back to double support."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/foot_load_bearing")
public class FootLoadBearingMessage extends Packet<FootLoadBearingMessage>
{
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
   @RosExportedField(documentation = "Wether the end-effector should be loaded or unloaded.")
   public LoadBearingRequest request;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootLoadBearingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public FootLoadBearingMessage(Random random)
   {
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
      request = RandomNumbers.nextEnum(random, LoadBearingRequest.class);
   }

   /**
    * Create a message to request one end-effector to switch to load bearing.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide refers to the side of the end-effector if necessary.
    */
   public FootLoadBearingMessage(RobotSide robotSide, LoadBearingRequest request)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.request = request;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public LoadBearingRequest getRequest()
   {
      return request;
   }

   @Override
   public boolean epsilonEquals(FootLoadBearingMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "Foot load bearing:"
            + "\nrobotSide = " + robotSide
            + "\nrequest = " + request;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootLoadBearingMessage(this);
   }
}
