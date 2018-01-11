package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FramePose3D;

public class WalkOverTerrainGoalPacket extends Packet<WalkOverTerrainGoalPacket>
{
   public FramePose3D goalPose;

   public WalkOverTerrainGoalPacket()
   {
   }

   public WalkOverTerrainGoalPacket(FramePose3D goalPose)
   {
      this.goalPose = goalPose;
   }

   @Override
   public boolean epsilonEquals(WalkOverTerrainGoalPacket other, double epsilon)
   {
      if(other == null)
         return false;

      return goalPose.epsilonEquals(other.goalPose, epsilon);
   }
}
