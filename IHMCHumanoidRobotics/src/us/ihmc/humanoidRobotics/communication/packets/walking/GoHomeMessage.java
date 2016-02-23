package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.DocumentedEnum;

@ClassDocumentation("The message commands the controller to bring the given part of the body back to a default configuration called 'home'."
      + " It is useful to get back to a safe configuration before walking.")
public class GoHomeMessage extends IHMCRosApiMessage<GoHomeMessage>
{
   public enum BodyPart implements DocumentedEnum<BodyPart>
   {
      ARM, CHEST, PELVIS;

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

      @Override
      public String getDocumentation(BodyPart var)
      {
         switch (var)
         {
         case CHEST:
            return "Request the chest to go back to a straight up configuration.";
         case ARM:
            return "Request the arm to go to a preconfigured home configuration that is elbow lightly flexed, forearm pointing forward, and upper pointing downward.";
         case PELVIS:
            return "Request the pelvis to go back to between the feet, zero pitch and roll, and headed in the same direction as the feet.";

         default:
            return "No documentation available.";
         }
      }

      @Override
      public BodyPart[] getDocumentedValues()
      {
         return values();
      }
   }

   @FieldDocumentation("Specifies the part of the body the user wants to move back to it home configuration.")
   public BodyPart bodyPart;
   @FieldDocumentation("Needed to identify a side dependent end-effector.")
   public RobotSide robotSide;
   @FieldDocumentation("How long the trajectory will spline from the current desired to the home configuration.")
   public double trajectoryTime;

   public GoHomeMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public GoHomeMessage(BodyPart bodyPart, RobotSide robotSide, double trajectoryTime)
   {
      this.bodyPart = bodyPart;
      this.robotSide = robotSide;
      this.trajectoryTime = trajectoryTime;
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
}
