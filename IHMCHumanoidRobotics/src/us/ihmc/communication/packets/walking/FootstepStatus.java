package us.ihmc.communication.packets.walking;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.tools.DocumentedEnum;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * User: Matt
 * Date: 1/18/13
 */
@ClassDocumentation("This message gives the status of the current footstep from the controller as well as the position\n"
                                  + "and orientation of the footstep in world cooredinates. ")
public class FootstepStatus extends IHMCRosApiPacket<FootstepStatus>
{
   public enum Status implements DocumentedEnum<Status>
   {
      STARTED, COMPLETED;
      
      @Override
      public String getDocumentation(Status var)
      {
         switch (var)
         {
         case STARTED:
            return "execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state";
         case COMPLETED:
            return "a footstep is completed";

         default:
            return "no documentation available";
         }
      }

      @Override
      public Status[] getDocumentedValues()
      {
         return values();
      }
   }

   public Status status;
   @FieldDocumentation("footstepIndex starts at 0 and monotonically increases with each completed footstep in a given\n"
                                     + "FootstepDataListMessage.")
   public int footstepIndex;

   public RobotSide robotSide;
   @FieldDocumentation("actualFootPositionInWorld gives the position of where the foot actually landed as opposed\n"
                                     + "to the desired position sent to the controller")
   public Point3d actualFootPositionInWorld;
   @FieldDocumentation("actualFootOrientationInWorld gives the orientation the foot is actually in as opposed to"
                                     + "the desired orientation sent to the controller\n")
   public Quat4d actualFootOrientationInWorld;
   @FieldDocumentation("isDoneWalking will be set to true when in double-support and there are no footsteps queued. If\n"
                                     + "isDoneWalking is set to true, the rest of the fields in this packet should be ignored")
   public boolean isDoneWalking;

   public FootstepStatus()
   {
   }

   public FootstepStatus(Status status, int footstepIndex)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.isDoneWalking = false;
      this.actualFootPositionInWorld = null;
      this.actualFootOrientationInWorld = null;
      this.robotSide = null;
   }

   public FootstepStatus(Status status, int footstepIndex, Point3d actualFootPositionInWorld, Quat4d actualFootOrientationInWorld)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.isDoneWalking = false;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;
      
      this.robotSide = null;
   }
   
   public FootstepStatus(Status status, int footstepIndex, Point3d actualFootPositionInWorld, Quat4d actualFootOrientationInWorld,RobotSide robotSide)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.isDoneWalking = false;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;
      this.robotSide = robotSide;
   }

   public static FootstepStatus createWalkingIsDonePacket()
   {
      FootstepStatus footstepStatus = new FootstepStatus();
      footstepStatus.isDoneWalking = true;
      footstepStatus.footstepIndex = 0;
      footstepStatus.status = null;
      footstepStatus.actualFootPositionInWorld = null;
      footstepStatus.actualFootOrientationInWorld = null;
      footstepStatus.robotSide = null;
      
      return footstepStatus;
   }

   public boolean isDoneWalking()
   {
      return isDoneWalking;
   }

   public Status getStatus()
   {
      return status;
   }

   public int getFootstepIndex()
   {
      return footstepIndex;
   }

   public String toString()
   {
      return "FootstepStatus{" + status + ", index: " + footstepIndex + "}";
   }

   public Point3d getActualFootPositionInWorld()
   {
      if (actualFootPositionInWorld != null)
         return actualFootPositionInWorld;
      return null;
   }

   public Quat4d getActualFootOrientationInWorld()
   {
      if (actualFootOrientationInWorld != null)
         return actualFootOrientationInWorld;
      return null;
   }

   public RobotSide getRobotSide()
   {
         return robotSide;
   }
   
   @Override
   public boolean equals(Object other)
   {
      if (other instanceof FootstepStatus)
      {
         FootstepStatus otherFoostepStatus = (FootstepStatus) other;
         boolean sameStatus = otherFoostepStatus.getStatus() == getStatus();
         boolean sameIndex = otherFoostepStatus.getFootstepIndex() == getFootstepIndex();

         return sameStatus && sameIndex;
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(FootstepStatus other, double epsilon)
   {
      return this.equals(other);
   }

   public FootstepStatus(Random random)
   {
      this.status = Status.values()[random.nextInt(Status.values().length)];
      this.footstepIndex = RandomTools.generateRandomIntWithEdgeCases(random, 0.1);
   }
}
