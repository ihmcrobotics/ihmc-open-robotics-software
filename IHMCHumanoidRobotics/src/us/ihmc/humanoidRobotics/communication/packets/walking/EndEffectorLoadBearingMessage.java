package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.DocumentedEnum;

@ClassDocumentation("This message commands the controller to start loading an end effector that was unloaded to support the robot weight. "
      + " One application is making a foot loadbearing."
      + " When the robot is performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the robot to switch back to double support."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.")
public class EndEffectorLoadBearingMessage extends IHMCRosApiMessage<EndEffectorLoadBearingMessage>
{
   public enum EndEffector implements DocumentedEnum<EndEffector>
   {
      FOOT, HAND;

      public static final EndEffector[] values = values();

      public boolean isRobotSideNeeded()
      {
         switch (this)
         {
         case FOOT:
            return true;
         case HAND:
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

         case HAND:
            return "In this case, the user also needs to provide the robotSide."
                  + " It is preferable to request a hand to switch to load bearing when it is aready close to the ground.";

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

   public enum LoadBearingRequest implements DocumentedEnum<LoadBearingRequest>
   {
      LOAD, UNLOAD;

      public static final LoadBearingRequest[] values = values();

      @Override
      public String getDocumentation(LoadBearingRequest var)
      {
         switch (var)
         {
         case LOAD:
            return "Request to load the given end-effector.";
         case UNLOAD:
            return "Request to unload the given end-effector.";
         default:
            return "No documentation available.";
         }
      }

      @Override
      public LoadBearingRequest[] getDocumentedValues()
      {
         return values();
      }
   }

   @FieldDocumentation("At the moment, since only the feet can be requested to switch to loadbearing, the robotSide has to be provided."
         + " It refers to which of the two feet should switch to loadbearing.")
   public RobotSide robotSide;
   @FieldDocumentation("Spcifies which end-effector should be loaded/unloaded.")
   public EndEffector endEffector;
   @FieldDocumentation("Wether the end-effector should be loaded or unloaded.")
   public LoadBearingRequest request;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public EndEffectorLoadBearingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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
}
