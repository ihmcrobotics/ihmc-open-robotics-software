package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.DocumentedEnum;

@ClassDocumentation("This message commands the controller to start loading an end effector that was unloaded to support the robot weight. "
      + "The only application at the moment is making a foot loadbearing."
      + "When the robot is performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the robot to switch back to double support.")
public class EndEffectorLoadBearingMessage extends IHMCRosApiPacket<EndEffectorLoadBearingMessage>
{
   public enum EndEffector implements DocumentedEnum<EndEffector>
   {
      FOOT;

      public static final EndEffector[] values = values();

      public boolean isRobotSideNeeded()
      {
         switch (this)
         {
         case FOOT:
            return true;
         default:
            throw new RuntimeException("Should not get there.");
         }
      }

      @Override
      public String getDocumentation(EndEffector var)
      {
         switch (var)
         {
         case FOOT:
            return "In this case, the user also needs to provide the robotSide."
                  + " If in the air, the corresponding foot will enter first a vertical ground approach transition and eventually touch the ground and switch to loadbearing."
                  + " Then the robot is ready to walk."
                  + " It is preferable to request a foot to switch to load bearing when it is aready close to the ground.";

         default:
            return "No documentation available.";
         }
      }

      @Override
      public EndEffector[] getDocumentedValues()
      {
         return values();
      }
   }

   @FieldDocumentation("At the moment, since only the feet can be requested to switch to loadbearing, the robotSide has to be provided."
         + " It refers to which of the two feet should switch to loadbearing.")
   public RobotSide robotSide;
   @FieldDocumentation("At the moment, only the feet can be requested to switch to loadbearing so this needs to be equal to EndEffector.FOOT."
         + " When the option for another end-effector will be available, the corresponding enum value for that end-effector will be added.")
   public EndEffector endEffector;

   /**
    * Empty constructor for serialization.
    */
   public EndEffectorLoadBearingMessage()
   {
   }

   public EndEffectorLoadBearingMessage(RobotSide robotSide, EndEffector endEffector)
   {
      this.robotSide = robotSide;
      this.endEffector = endEffector;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public EndEffector getEndEffector()
   {
      return endEffector;
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
}
