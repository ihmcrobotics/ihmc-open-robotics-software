package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class FootstepPlannerRequest
{
   private int requestId;

   private RobotSide requestedInitialStanceSide;
   private final SideDependentList<Pose3D> startFootPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> goalFootPoses = new SideDependentList<>(new Pose3D(), new Pose3D());

   private boolean planBodyPath;
   private double goalDistanceProximity;
   private double goalYawProximity;
   private double timeout;
   private double horizonLength;
   private PlanarRegionsList planarRegionsList;
   private boolean assumeFlatGround;
   private final ArrayList<Pose3DReadOnly> bodyPathWaypoints = new ArrayList<>();

   public FootstepPlannerRequest()
   {
      clear();
   }

   private void clear()
   {
      requestId = -1;
      requestedInitialStanceSide = RobotSide.LEFT;
      startFootPoses.forEach(Pose3D::setToNaN);
      goalFootPoses.forEach(Pose3D::setToNaN);
      planBodyPath = false;
      goalDistanceProximity = -1.0;
      goalYawProximity = -1.0;
      timeout = 5.0;
      horizonLength = Double.MAX_VALUE;
      planarRegionsList = null;
      assumeFlatGround = false;
      bodyPathWaypoints.clear();
   }

   public void setRequestId(int requestId)
   {
      this.requestId = requestId;
   }

   public void setRequestedInitialStanceSide(RobotSide requestedInitialStanceSide)
   {
      this.requestedInitialStanceSide = requestedInitialStanceSide;
   }

   public void setStartFootPoses(double idealStepWidth, Pose3DReadOnly midFootStartPose)
   {
      for (RobotSide side : RobotSide.values)
      {
         startFootPoses.get(side).set(midFootStartPose);
         startFootPoses.get(side).appendTranslation(0.0, 0.5 * side.negateIfRightSide(idealStepWidth), 0.0);
      }
   }

   public void setStartFootPoses(Pose3DReadOnly leftFootPose, Pose3DReadOnly rightFootPose)
   {
      this.startFootPoses.get(RobotSide.LEFT).set(leftFootPose);
      this.startFootPoses.get(RobotSide.RIGHT).set(rightFootPose);
   }

   public void setStartFootPose(RobotSide side, Pose3DReadOnly footPose)
   {
      this.startFootPoses.get(side).set(footPose);
   }

   public void setStartFootPose(RobotSide side, Tuple3DReadOnly stanceFootPosition, Orientation3DReadOnly stanceFootOrientation)
   {
      this.startFootPoses.get(side).set(stanceFootPosition, stanceFootOrientation);
   }

   public void setGoalFootPoses(Pose3DReadOnly leftFootPose, Pose3DReadOnly rightFootPose)
   {
      this.goalFootPoses.get(RobotSide.LEFT).set(leftFootPose);
      this.goalFootPoses.get(RobotSide.RIGHT).set(rightFootPose);
   }

   public void setGoalFootPoses(double idealStepWidth, Pose3DReadOnly midFootGoalPose)
   {
      for (RobotSide side : RobotSide.values)
      {
         goalFootPoses.get(side).set(midFootGoalPose);
         goalFootPoses.get(side).appendTranslation(0.0, 0.5 * side.negateIfRightSide(idealStepWidth), 0.0);
      }
   }

   public void setGoalFootPose(RobotSide side, Pose3DReadOnly goalPose)
   {
      this.goalFootPoses.get(side).set(goalPose);
   }

   public void setGoalFootPose(RobotSide side, Tuple3DReadOnly goalPosition, Orientation3DReadOnly goalOrientation)
   {
      this.goalFootPoses.get(side).set(goalPosition, goalOrientation);
   }

   public void setPlanBodyPath(boolean planBodyPath)
   {
      this.planBodyPath = planBodyPath;
   }

   public void setGoalDistanceProximity(double goalDistanceProximity)
   {
      this.goalDistanceProximity = goalDistanceProximity;
   }

   public void setGoalYawProximity(double goalYawProximity)
   {
      this.goalYawProximity = goalYawProximity;
   }

   public void setTimeout(double timeout)
   {
      this.timeout = timeout;
   }

   public void setHorizonLength(double horizonLength)
   {
      this.horizonLength = horizonLength;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setAssumeFlatGround(boolean assumeFlatGround)
   {
      this.assumeFlatGround = assumeFlatGround;
   }

   public int getRequestId()
   {
      return requestId;
   }

   public RobotSide getRequestedInitialStanceSide()
   {
      return requestedInitialStanceSide;
   }

   public SideDependentList<Pose3D> getStartFootPoses()
   {
      return startFootPoses;
   }

   public SideDependentList<Pose3D> getGoalFootPoses()
   {
      return goalFootPoses;
   }

   public boolean getPlanBodyPath()
   {
      return planBodyPath;
   }

   public double getGoalDistanceProximity()
   {
      return goalDistanceProximity;
   }

   public double getGoalYawProximity()
   {
      return goalYawProximity;
   }

   public double getTimeout()
   {
      return timeout;
   }

   public double getHorizonLength()
   {
      return horizonLength;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public boolean getAssumeFlatGround()
   {
      return assumeFlatGround;
   }

   public ArrayList<Pose3DReadOnly> getBodyPathWaypoints()
   {
      return bodyPathWaypoints;
   }

   public void setFromPacket(FootstepPlanningRequestPacket requestPacket)
   {
      clear();

      setRequestId(requestPacket.getPlannerRequestId());
      RobotSide requestedInitialStanceSide = RobotSide.fromByte(requestPacket.getRequestedInitialStanceSide());
      if (requestedInitialStanceSide != null)
         setRequestedInitialStanceSide(requestedInitialStanceSide);
      setStartFootPose(RobotSide.LEFT, requestPacket.getStartLeftFootPose());
      setStartFootPose(RobotSide.RIGHT, requestPacket.getStartRightFootPose());
      setGoalFootPose(RobotSide.LEFT, requestPacket.getGoalLeftFootPose());
      setGoalFootPose(RobotSide.RIGHT, requestPacket.getGoalRightFootPose());

      setPlanBodyPath(FootstepPlannerType.fromByte(requestPacket.getRequestedFootstepPlannerType()).plansPath());
      setGoalDistanceProximity(requestPacket.getGoalDistanceProximity());
      setGoalYawProximity(requestPacket.getGoalYawProximity());
      setTimeout(requestPacket.getTimeout());
      if(requestPacket.getHorizonLength() > 0.0)
         setHorizonLength(requestPacket.getHorizonLength());
      setAssumeFlatGround(requestPacket.getAssumeFlatGround());

      for (int i = 0; i < requestPacket.getBodyPathWaypoints().size(); i++)
      {
         bodyPathWaypoints.add(new Pose3D(requestPacket.getBodyPathWaypoints().get(i)));
      }

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(requestPacket.getPlanarRegionsListMessage());
      setPlanarRegionsList(planarRegionsList);
   }

   public void setPacket(FootstepPlanningRequestPacket requestPacket)
   {
      requestPacket.setPlannerRequestId(getRequestId());

      requestPacket.setRequestedInitialStanceSide(getRequestedInitialStanceSide().toByte());
      requestPacket.getStartLeftFootPose().set(getStartFootPoses().get(RobotSide.LEFT));
      requestPacket.getStartRightFootPose().set(getStartFootPoses().get(RobotSide.RIGHT));
      requestPacket.getGoalLeftFootPose().set(getGoalFootPoses().get(RobotSide.LEFT));
      requestPacket.getGoalRightFootPose().set(getGoalFootPoses().get(RobotSide.RIGHT));

      requestPacket.setRequestedFootstepPlannerType((getPlanBodyPath() ? FootstepPlannerType.VIS_GRAPH_WITH_A_STAR : FootstepPlannerType.A_STAR).toByte());
      requestPacket.setGoalDistanceProximity(getGoalDistanceProximity());
      requestPacket.setGoalYawProximity(getGoalYawProximity());
      requestPacket.setTimeout(getTimeout());
      requestPacket.setHorizonLength(getHorizonLength());
      requestPacket.setAssumeFlatGround(getAssumeFlatGround());

      requestPacket.getBodyPathWaypoints().clear();
      for (int i = 0; i < bodyPathWaypoints.size(); i++)
      {
         requestPacket.getBodyPathWaypoints().add().set(bodyPathWaypoints.get(i));
      }

      if(getPlanarRegionsList() != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(getPlanarRegionsList());
         requestPacket.getPlanarRegionsListMessage().set(planarRegionsListMessage);
      }
   }

   public void set(FootstepPlannerRequest other)
   {
      clear();

      this.requestId = other.requestId;

      this.requestedInitialStanceSide = other.requestedInitialStanceSide;
      this.startFootPoses.get(RobotSide.LEFT).set(other.startFootPoses.get(RobotSide.LEFT));
      this.startFootPoses.get(RobotSide.RIGHT).set(other.startFootPoses.get(RobotSide.RIGHT));
      this.goalFootPoses.get(RobotSide.LEFT).set(other.goalFootPoses.get(RobotSide.LEFT));
      this.goalFootPoses.get(RobotSide.RIGHT).set(other.goalFootPoses.get(RobotSide.RIGHT));

      this.planBodyPath = other.planBodyPath;
      this.goalDistanceProximity = other.goalDistanceProximity;
      this.goalYawProximity = other.goalYawProximity;
      this.timeout = other.timeout;
      this.horizonLength = other.horizonLength;
      this.assumeFlatGround = other.assumeFlatGround;

      if(other.planarRegionsList != null)
      {
         this.planarRegionsList = other.planarRegionsList.copy();
      }

      for (int i = 0; i < other.bodyPathWaypoints.size(); i++)
      {
         this.bodyPathWaypoints.add(new Pose3D(other.bodyPathWaypoints.get(i)));
      }
   }
}
