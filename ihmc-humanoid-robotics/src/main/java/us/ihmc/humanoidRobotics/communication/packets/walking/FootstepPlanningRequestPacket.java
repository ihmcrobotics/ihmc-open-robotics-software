package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.robotics.geometry.RotationTools;

public class FootstepPlanningRequestPacket extends Packet<FootstepPlanningRequestPacket>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   public static final byte FOOTSTEP_PLANNER_TYPE_PLANAR_REGION_BIPEDAL = 0;
   public static final byte FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP = 1;
   public static final byte FOOTSTEP_PLANNER_TYPE_A_STAR = 2;
   public static final byte FOOTSTEP_PLANNER_TYPE_SIMPLE_BODY_PATH = 3;
   public static final byte FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR = 4;

   public static final int NO_PLAN_ID = -1;

   public byte initialStanceRobotSide;
   public Point3D32 stanceFootPositionInWorld;
   public Quaternion32 stanceFootOrientationInWorld;
   public Point3D32 goalPositionInWorld;
   public Quaternion32 goalOrientationInWorld;
   public byte requestedFootstepPlannerType;
   public double timeout;
   public PlanarRegionsListMessage planarRegionsListMessage;

   public int plannerRequestId = NO_PLAN_ID;

   public FootstepPlanningRequestPacket()
   {
      // empty constructor for serialization
   }

   @Override
   public void set(FootstepPlanningRequestPacket other)
   {
      initialStanceRobotSide = other.initialStanceRobotSide;
      stanceFootPositionInWorld = new Point3D32(other.stanceFootPositionInWorld);
      stanceFootOrientationInWorld = new Quaternion32(other.stanceFootOrientationInWorld);
      goalPositionInWorld = new Point3D32(other.goalPositionInWorld);
      goalOrientationInWorld = new Quaternion32(other.goalOrientationInWorld);
      requestedFootstepPlannerType = other.requestedFootstepPlannerType;
      timeout = other.timeout;
      planarRegionsListMessage = new PlanarRegionsListMessage();
      planarRegionsListMessage.set(other.planarRegionsListMessage);
      plannerRequestId = other.plannerRequestId;

      setPacketInformation(other);
   }

   public void setTimeout(double timeout)
   {
      this.timeout = timeout;
   }

   public double getTimeout()
   {
      return timeout;
   }

   public void setPlanarRegionsListMessage(PlanarRegionsListMessage planarRegionsListMessage)
   {
      this.planarRegionsListMessage = planarRegionsListMessage;
   }

   public void setPlannerRequestId(int planId)
   {
      this.plannerRequestId = planId;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningRequestPacket other, double epsilon)
   {
      if (initialStanceRobotSide != other.initialStanceRobotSide)
         return false;
      if (!stanceFootPositionInWorld.epsilonEquals(other.stanceFootPositionInWorld, (float) epsilon))
         return false;
      if (!RotationTools.quaternionEpsilonEquals(stanceFootOrientationInWorld, other.stanceFootOrientationInWorld, (float) epsilon))
         return false;
      if (!goalPositionInWorld.epsilonEquals(other.goalPositionInWorld, (float) epsilon))
         return false;
      if (!RotationTools.quaternionEpsilonEquals(goalOrientationInWorld, other.goalOrientationInWorld, (float) epsilon))
         return false;
      if (this.requestedFootstepPlannerType != other.requestedFootstepPlannerType)
         return false;
      if (plannerRequestId != other.plannerRequestId)
         return false;
      if (planarRegionsListMessage != null && other.planarRegionsListMessage != null
            && planarRegionsListMessage.epsilonEquals(other.planarRegionsListMessage, epsilon))
         return false;
      return true;
   }

}
