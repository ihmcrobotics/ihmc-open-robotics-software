package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "The message commands the controller to bring the given part of the body back to a default configuration called 'home'."
      + " It is useful to get back to a safe configuration before walking.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/go_home")
public class GoHomeMessage extends Packet<GoHomeMessage>
{
   public enum BodyPart
   {
      @RosEnumValueDocumentation(documentation = "Request the chest to go back to a straight up configuration.")
      ARM,
      @RosEnumValueDocumentation(documentation = "Request the arm to go to a preconfigured home configuration that is elbow lightly flexed, forearm pointing forward, and upper pointing downward.")
      CHEST,
      @RosEnumValueDocumentation(documentation = "Request the pelvis to go back to between the feet, zero pitch and roll, and headed in the same direction as the feet.")
      PELVIS;

      public static final BodyPart[] values = values();

      public boolean isRobotSideNeeded()
      {
         switch (this)
         {
         case ARM:
            return true;
         case CHEST:
         case PELVIS:
            return false;
         default:
            throw new RuntimeException("Should not get there.");
         }
      }
   }

   @RosExportedField(documentation = "Specifies the part of the body the user wants to move back to it home configuration.")
   public BodyPart bodyPart;
   @RosExportedField(documentation = "Needed to identify a side dependent end-effector.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "How long the trajectory will spline from the current desired to the home configuration.")
   public double trajectoryTime;

   public GoHomeMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public GoHomeMessage(Random random)
   {
      bodyPart = RandomTools.generateRandomEnum(random, BodyPart.class);
      robotSide = RandomTools.generateRandomEnum(random, RobotSide.class);
      trajectoryTime = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public GoHomeMessage(BodyPart bodyPart, double trajectoryTime)
   {
      checkRobotSide(bodyPart);
      this.bodyPart = bodyPart;
      this.trajectoryTime = trajectoryTime;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public GoHomeMessage(BodyPart bodyPart, RobotSide robotSide, double trajectoryTime)
   {
      if (robotSide == null)
         checkRobotSide(bodyPart);
      this.bodyPart = bodyPart;
      this.robotSide = robotSide;
      this.trajectoryTime = trajectoryTime;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   private void checkRobotSide(BodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
   }

   public BodyPart getBodyPart()
   {
      return bodyPart;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public boolean epsilonEquals(GoHomeMessage other, double epsilon)
   {
      if (bodyPart != other.bodyPart)
         return false;
      if (robotSide != other.robotSide)
         return false;
      return true;
   }
   
   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateGoHomeMessage(this);
   }
}
