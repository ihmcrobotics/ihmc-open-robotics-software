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

import java.util.ArrayList;

public class FootstepPlannerRequest
{
   private int requestId;

   private RobotSide initialStanceSide;
   private final Pose3D stanceFootPose = new Pose3D();
   private final Pose3D goalPose = new Pose3D();
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
      initialStanceSide = null;
      stanceFootPose.setToNaN();
      goalPose.setToNaN();
      planBodyPath = false;
      goalDistanceProximity = -1.0;
      goalYawProximity = -1.0;
      timeout = 5.0;
      horizonLength = Double.POSITIVE_INFINITY;
      planarRegionsList = null;
      assumeFlatGround = false;
      bodyPathWaypoints.clear();
   }

   public void setRequestId(int requestId)
   {
      this.requestId = requestId;
   }

   public void setInitialStanceSide(RobotSide initialStanceSide)
   {
      this.initialStanceSide = initialStanceSide;
   }

   public void setInitialStancePose(Pose3DReadOnly stanceFootPose)
   {
      this.stanceFootPose.set(stanceFootPose);
   }

   public void setInitialStancePose(Tuple3DReadOnly stanceFootPosition, Orientation3DReadOnly stanceFootOrientation)
   {
      this.stanceFootPose.set(stanceFootPosition, stanceFootOrientation);
   }

   public void setGoalPose(Pose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setGoalPose(Tuple3DReadOnly goalPosition, Orientation3DReadOnly goalOrientation)
   {
      this.goalPose.set(goalPosition, goalOrientation);
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

   public RobotSide getInitialStanceSide()
   {
      return initialStanceSide;
   }

   public Pose3D getStanceFootPose()
   {
      return stanceFootPose;
   }

   public Pose3D getGoalPose()
   {
      return goalPose;
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
      setInitialStanceSide(RobotSide.fromByte(requestPacket.getInitialStanceRobotSide()));
      setInitialStancePose(requestPacket.getStanceFootPositionInWorld(), requestPacket.getStanceFootOrientationInWorld());
      setGoalPose(requestPacket.getGoalPositionInWorld(), requestPacket.getGoalOrientationInWorld());
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
      requestPacket.setInitialStanceRobotSide(getInitialStanceSide().toByte());
      requestPacket.getStanceFootPositionInWorld().set(getStanceFootPose().getPosition());
      requestPacket.getStanceFootOrientationInWorld().set(getStanceFootPose().getOrientation());
      requestPacket.getGoalPositionInWorld().set(getGoalPose().getPosition());
      requestPacket.getGoalOrientationInWorld().set(getGoalPose().getOrientation());
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

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(getPlanarRegionsList());
      requestPacket.getPlanarRegionsListMessage().set(planarRegionsListMessage);
   }

   public void set(FootstepPlannerRequest other)
   {
      clear();

      this.requestId = other.requestId;
      this.initialStanceSide = other.initialStanceSide;
      this.stanceFootPose.set(other.stanceFootPose);
      this.goalPose.set(other.goalPose);
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
