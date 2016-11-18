package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * User: Matt
 * Date: 1/18/13
 */
@RosMessagePacket(documentation = "This message gives the status of the current footstep from the controller as well as the position\n"
                                  + "and orientation of the footstep in world cooredinates. ",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/output/footstep_status")
public class FootstepStatus extends StatusPacket<FootstepStatus>
{
   public enum Status
   {
      @RosEnumValueDocumentation(documentation = "execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state")
      STARTED,
      @RosEnumValueDocumentation(documentation = "a footstep is completed")
      COMPLETED
   }

   @RosExportedField(documentation = "The current footstep status enum value.")
   public Status status;
   @RosExportedField(documentation = "footstepIndex starts at 0 and monotonically increases with each completed footstep in a given\n"
                                     + "FootstepDataListMessage.")
   public int footstepIndex;

   @RosExportedField(documentation = "The robot side (left or right) that this footstep status correlates to.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "desiredFootPositionInWorld gives the position of the desired position sent to the controller as opposed\n"
         + "to where the foot actually landed")
   public Point3d desiredFootPositionInWorld;
   @RosExportedField(documentation = "desiredFootOrientationInWorld gives the desired orientation of the foot sent to the controller as opposed to\n"
         + "the orientation where the foot actually is")
   public Quat4d desiredFootOrientationInWorld;
   @RosExportedField(documentation = "actualFootPositionInWorld gives the position of where the foot actually landed as opposed\n"
                                     + "to the desired position sent to the controller")
   public Point3d actualFootPositionInWorld;
   @RosExportedField(documentation = "actualFootOrientationInWorld gives the orientation the foot is actually in as opposed to\n"
                                     + "the desired orientation sent to the controller")
   public Quat4d actualFootOrientationInWorld;

   public FootstepStatus()
   {
   }

   public FootstepStatus(Status status, int footstepIndex)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = null;
      this.desiredFootOrientationInWorld = null;
      this.actualFootPositionInWorld = null;
      this.actualFootOrientationInWorld = null;
      this.robotSide = null;
   }

   public FootstepStatus(Status status, int footstepIndex, Point3d actualFootPositionInWorld, Quat4d actualFootOrientationInWorld)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = null;
      this.desiredFootOrientationInWorld = null;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;

      this.robotSide = null;
   }

   public FootstepStatus(Status status, int footstepIndex, Point3d actualFootPositionInWorld, Quat4d actualFootOrientationInWorld, RobotSide robotSide)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = null;
      this.desiredFootOrientationInWorld = null;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;
      this.robotSide = robotSide;
   }

   public FootstepStatus(Status status, int footstepIndex, Point3d desiredFootPositionInWorld, Quat4d desiredFootOrientationInWorld,
         Point3d actualFootPositionInWorld, Quat4d actualFootOrientationInWorld, RobotSide robotSide)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = desiredFootPositionInWorld;
      this.desiredFootOrientationInWorld = desiredFootOrientationInWorld;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;
      this.robotSide = robotSide;
   }

   @Override
   public void set(FootstepStatus other)
   {
      status = other.status;
      footstepIndex = other.footstepIndex;
      robotSide = other.robotSide;

      if (desiredFootPositionInWorld == null)
         desiredFootPositionInWorld = new Point3d();
      if (desiredFootOrientationInWorld == null)
         desiredFootOrientationInWorld = new Quat4d();

      if (other.desiredFootPositionInWorld == null)
         desiredFootPositionInWorld.set(Double.NaN, Double.NaN, Double.NaN);
      else
         desiredFootPositionInWorld.set(other.desiredFootPositionInWorld);

      if (other.desiredFootOrientationInWorld == null)
         desiredFootOrientationInWorld.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      else
         desiredFootOrientationInWorld.set(other.desiredFootOrientationInWorld);

      if (actualFootPositionInWorld == null)
         actualFootPositionInWorld = new Point3d();
      if (actualFootOrientationInWorld == null)
         actualFootOrientationInWorld = new Quat4d();

      if (other.actualFootPositionInWorld == null)
         actualFootPositionInWorld.set(Double.NaN, Double.NaN, Double.NaN);
      else
         actualFootPositionInWorld.set(other.actualFootPositionInWorld);

      if (other.actualFootOrientationInWorld == null)
         actualFootOrientationInWorld.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      else
         actualFootOrientationInWorld.set(other.actualFootOrientationInWorld);
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

   public Point3d getDesiredFootPositionInWorld()
   {
      if (desiredFootPositionInWorld != null)
         return desiredFootPositionInWorld;
      return null;
   }

   public Quat4d getDesiredFootOrientationInWorld()
   {
      if (desiredFootOrientationInWorld != null)
         return desiredFootOrientationInWorld;
      return null;
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
