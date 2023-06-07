package us.ihmc.footstepPlanning;

import perception_msgs.msg.dds.HeightMapMessage;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.ArrayList;

public class FootstepPlannerRequest
{
   /**
    * Unique ID associated with the plan
    */
   private int requestId;

   /**
    * Requested initial stance side. If this is null the planner will choose an initial stance side
    */
   private RobotSide requestedInitialStanceSide;

   /**
    * The starting left and right footstep poses
    */
   private final SideDependentList<Pose3D> startFootPoses = new SideDependentList<>(side -> new Pose3D());

   /**
    * The goal left and right footstep poses.
    */
   private final SideDependentList<Pose3D> goalFootPoses = new SideDependentList<>(side -> new Pose3D());

   /**
    * If true, the planner will snap the provided goal steps. Otherwise the provided poses will be trusted as valid footholds.
    */
   private boolean snapGoalSteps;

   /**
    * If {@link #snapGoalSteps} is true and the goal steps can't be snapped, this specifies whether to abort or go ahead and plan.
    */
   private boolean abortIfGoalStepSnappingFails;

   /**
    * If plan_body_path is true and the planner fails, this specifies whether to abort or use a straight-line body path
    */
   private boolean abortIfBodyPathPlannerFails;

   /**
    * If true, the planner will plan a body path. If false, it will try to follow a straight line to the goal.
    */
   private boolean planBodyPath;

   /**
    * If true, the planner will plan steps. If false, it will only return the body path.
    */
   private boolean planFootsteps;

   /**
    * If true, does A* search. If false, a simple turn-walk-turn path is returned with no checks on step feasibility.
    */
   private boolean performAStarSearch;

   /**
    * (beta) This specifies an acceptable xy distance that is acceptable to terminate planning. Set to non-positive value to disable.
    */
   private double goalDistanceProximity;

   /**
    * (beta) This specifies an acceptable orientation distance that is acceptable to terminate planning. Set to non-positive value to disable.
    */
   private double goalYawProximity;

   /**
    * Planner timeout in seconds. If {@link #maximumIterations} is set also, the planner terminates whenever either is reached
    */
   private double timeout;

   /**
    * Maximum iterations. Set to a non-positive number to disable. If {@link #timeout} is also set, the planner terminates whener either is reached.
    */
   private int maximumIterations;

   /**
    * Maximum lookahead distance along the body path. Is only used when {@link #planBodyPath} is true
    */
   private double horizonLength;

   /**
    * Planar regions. May be null or empty to enable flat ground mode.
    */
   private PlanarRegionsList planarRegionsList;

   /**
    * Fallback planar regions. May be null or empty to enable flat ground mode.
    */
   private PlanarRegionsList fallbackPlanarRegionsList;

   /**
    * Height map. May be null to enable flat ground mode.
    */
   private HeightMapData heightMapData;

   /**
    * If true, will ignore planar regions and plan on flat ground.
    * Note that collision checks will still be performed if enabled, such as {@link FootstepPlannerParametersReadOnly#checkForBodyBoxCollisions}
    */
   private boolean assumeFlatGround;

   /**
    * Externally provided body path waypoints. If this is non-empty and {@link #planBodyPath} is false, the planner will follow this path.
    */
   private final ArrayList<Pose3DReadOnly> bodyPathWaypoints = new ArrayList<>();

   /**
    * Period of time in seconds the planner will publish it's status. If this is a non-positive number no status is published until it's completed.
    */
   private double statusPublishPeriod;

   /**
    * Requested swing planner. Sets swing parameters to avoid collisions
    */
   private SwingPlannerType swingPlannerType = SwingPlannerType.NONE;

   /**
    * Reference footstep plan. When provided, the planner will try to match this plan by having the cost function's minima be at these steps.
    */
   private FootstepPlan referencePlan = null;

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
      snapGoalSteps = true;
      abortIfGoalStepSnappingFails = false;
      abortIfBodyPathPlannerFails = false;
      planBodyPath = false;
      planFootsteps = true;
      performAStarSearch = true;
      goalDistanceProximity = -1.0;
      goalYawProximity = -1.0;
      timeout = 5.0;
      maximumIterations = -1;
      horizonLength = Double.MAX_VALUE;
      planarRegionsList = null;
      fallbackPlanarRegionsList = null;
      heightMapData = null;
      assumeFlatGround = false;
      bodyPathWaypoints.clear();
      statusPublishPeriod = 1.0;
      swingPlannerType = SwingPlannerType.NONE;
      referencePlan = null;
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

   public void setSnapGoalSteps(boolean snapGoalSteps)
   {
      this.snapGoalSteps = snapGoalSteps;
   }

   public void setAbortIfGoalStepSnappingFails(boolean abortIfGoalStepSnappingFails)
   {
      this.abortIfGoalStepSnappingFails = abortIfGoalStepSnappingFails;
   }

   public void setAbortIfBodyPathPlannerFails(boolean abortIfBodyPathPlannerFails)
   {
      this.abortIfBodyPathPlannerFails = abortIfBodyPathPlannerFails;
   }

   public void setPlanBodyPath(boolean planBodyPath)
   {
      this.planBodyPath = planBodyPath;
   }

   public void setPlanFootsteps(boolean planFootsteps)
   {
      this.planFootsteps = planFootsteps;
   }

   public void setPerformAStarSearch(boolean performAStarSearch)
   {
      this.performAStarSearch = performAStarSearch;
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

   public void setMaximumIterations(int maximumIterations)
   {
      this.maximumIterations = maximumIterations;
   }

   public void setHorizonLength(double horizonLength)
   {
      this.horizonLength = horizonLength;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setFallbackPlanarRegionsList(PlanarRegionsList fallbackPlanarRegionsList)
   {
      this.fallbackPlanarRegionsList = fallbackPlanarRegionsList;
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   public void setAssumeFlatGround(boolean assumeFlatGround)
   {
      this.assumeFlatGround = assumeFlatGround;
   }

   public void setStatusPublishPeriod(double statusPublishPeriod)
   {
      this.statusPublishPeriod = statusPublishPeriod;
   }

   public void setSwingPlannerType(SwingPlannerType swingPlannerType)
   {
      this.swingPlannerType = swingPlannerType;
   }

   public void setReferencePlan(FootstepPlanReadOnly referencePlan)
   {
      if (referencePlan == null)
         this.referencePlan = null;
      else
         this.referencePlan = new FootstepPlan(referencePlan);
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

   public boolean getSnapGoalSteps()
   {
      return snapGoalSteps;
   }

   public boolean getAbortIfGoalStepSnappingFails()
   {
      return abortIfGoalStepSnappingFails;
   }

   public boolean getAbortIfBodyPathPlannerFails()
   {
      return abortIfBodyPathPlannerFails;
   }

   public boolean getPlanBodyPath()
   {
      return planBodyPath;
   }

   public boolean getPlanFootsteps()
   {
      return planFootsteps;
   }

   public boolean getPerformAStarSearch()
   {
      return performAStarSearch;
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

   public int getMaximumIterations()
   {
      return maximumIterations;
   }

   public double getHorizonLength()
   {
      return horizonLength;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public PlanarRegionsList getFallbackPlanarRegionsList()
   {
      return fallbackPlanarRegionsList;
   }

   public HeightMapData getHeightMapData()
   {
      return heightMapData;
   }

   public boolean getAssumeFlatGround()
   {
      return assumeFlatGround;
   }

   public ArrayList<Pose3DReadOnly> getBodyPathWaypoints()
   {
      return bodyPathWaypoints;
   }

   public double getStatusPublishPeriod()
   {
      return statusPublishPeriod;
   }

   public SwingPlannerType getSwingPlannerType()
   {
      return swingPlannerType;
   }

   public FootstepPlan getReferencePlan()
   {
      return referencePlan;
   }

   public boolean hasReferenceFootstepPlan()
   {
      return referencePlan != null;
   }

   // TODO add ROS field if needed. probably should be added to be loggable

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
      setSnapGoalSteps(requestPacket.getSnapGoalSteps());
      setAbortIfGoalStepSnappingFails(requestPacket.getAbortIfGoalStepSnappingFails());
      setAbortIfBodyPathPlannerFails(requestPacket.getAbortIfBodyPathPlannerFails());
      setPlanBodyPath(requestPacket.getPlanBodyPath());
      setPlanFootsteps(requestPacket.getPlanFootsteps());
      setPerformAStarSearch(requestPacket.getPerformAStarSearch());
      setGoalDistanceProximity(requestPacket.getGoalDistanceProximity());
      setGoalYawProximity(requestPacket.getGoalYawProximity());
      setTimeout(requestPacket.getTimeout());
      setMaximumIterations(requestPacket.getMaxIterations());
      if(requestPacket.getHorizonLength() > 0.0)
         setHorizonLength(requestPacket.getHorizonLength());
      setAssumeFlatGround(requestPacket.getAssumeFlatGround());
      setStatusPublishPeriod(requestPacket.getStatusPublishPeriod());
      setReferencePlan(FootstepDataMessageConverter.convertToFootstepPlan(requestPacket.getReferencePlan()));

      SwingPlannerType swingPlannerType = SwingPlannerType.fromByte(requestPacket.getRequestedSwingPlanner());
      if (swingPlannerType != null)
         setSwingPlannerType(swingPlannerType);

      for (int i = 0; i < requestPacket.getBodyPathWaypoints().size(); i++)
      {
         bodyPathWaypoints.add(new Pose3D(requestPacket.getBodyPathWaypoints().get(i)));
      }

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(requestPacket.getPlanarRegionsListMessage());
      setPlanarRegionsList(planarRegionsList);
      PlanarRegionsList fallbackPlanarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(requestPacket.getFallbackPlanarRegionsListMessage());
      setFallbackPlanarRegionsList(fallbackPlanarRegionsList);
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(requestPacket.getHeightMapMessage());
      if (!heightMapData.isEmpty())
         setHeightMapData(heightMapData);
      else
         setHeightMapData(null);
   }

   public void setPacket(FootstepPlanningRequestPacket requestPacket)
   {
      requestPacket.setPlannerRequestId(getRequestId());

      requestPacket.setRequestedInitialStanceSide(getRequestedInitialStanceSide().toByte());
      requestPacket.getStartLeftFootPose().set(getStartFootPoses().get(RobotSide.LEFT));
      requestPacket.getStartRightFootPose().set(getStartFootPoses().get(RobotSide.RIGHT));
      requestPacket.getGoalLeftFootPose().set(getGoalFootPoses().get(RobotSide.LEFT));
      requestPacket.getGoalRightFootPose().set(getGoalFootPoses().get(RobotSide.RIGHT));
      requestPacket.setSnapGoalSteps(getSnapGoalSteps());
      requestPacket.setAbortIfGoalStepSnappingFails(getAbortIfGoalStepSnappingFails());
      requestPacket.setAbortIfBodyPathPlannerFails(getAbortIfBodyPathPlannerFails());
      requestPacket.setPlanBodyPath(getPlanBodyPath());
      requestPacket.setPlanFootsteps(getPlanFootsteps());
      requestPacket.setPerformAStarSearch(getPerformAStarSearch());
      requestPacket.setGoalDistanceProximity(getGoalDistanceProximity());
      requestPacket.setGoalYawProximity(getGoalYawProximity());
      requestPacket.setTimeout(getTimeout());
      requestPacket.setMaxIterations(getMaximumIterations());
      requestPacket.setHorizonLength(getHorizonLength());
      requestPacket.setAssumeFlatGround(getAssumeFlatGround());
      requestPacket.setStatusPublishPeriod(getStatusPublishPeriod());
      requestPacket.setRequestedSwingPlanner(getSwingPlannerType().toByte());

      requestPacket.getBodyPathWaypoints().clear();
      for (int i = 0; i < bodyPathWaypoints.size(); i++)
      {
         requestPacket.getBodyPathWaypoints().add().set(bodyPathWaypoints.get(i));
      }

      if (getPlanarRegionsList() != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(getPlanarRegionsList());
         requestPacket.getPlanarRegionsListMessage().set(planarRegionsListMessage);
      }
      if (getFallbackPlanarRegionsList() != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(getFallbackPlanarRegionsList());
         requestPacket.getFallbackPlanarRegionsListMessage().set(planarRegionsListMessage);
      }

      if (getHeightMapData() != null)
      {
         HeightMapMessage heightMapMessage = HeightMapMessageTools.toMessage(getHeightMapData());
         requestPacket.getHeightMapMessage().set(heightMapMessage);
      }

      if (referencePlan != null && !referencePlan.isEmpty())
      {
         requestPacket.getReferencePlan().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(referencePlan, -1.0, -1.0));
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
      this.snapGoalSteps = other.snapGoalSteps;
      this.abortIfGoalStepSnappingFails = other.abortIfGoalStepSnappingFails;
      this.abortIfBodyPathPlannerFails = other.abortIfBodyPathPlannerFails;

      this.planBodyPath = other.planBodyPath;
      this.planFootsteps = other.planFootsteps;
      this.performAStarSearch = other.performAStarSearch;
      this.goalDistanceProximity = other.goalDistanceProximity;
      this.goalYawProximity = other.goalYawProximity;
      this.timeout = other.timeout;
      this.maximumIterations = other.maximumIterations;
      this.horizonLength = other.horizonLength;
      this.assumeFlatGround = other.assumeFlatGround;
      this.statusPublishPeriod = other.statusPublishPeriod;
      this.swingPlannerType = other.swingPlannerType;

      if(other.planarRegionsList != null)
      {
         this.planarRegionsList = other.planarRegionsList.copy();
      }
      if(other.fallbackPlanarRegionsList != null)
      {
         this.fallbackPlanarRegionsList = other.fallbackPlanarRegionsList.copy();
      }

      for (int i = 0; i < other.bodyPathWaypoints.size(); i++)
      {
         this.bodyPathWaypoints.add(new Pose3D(other.bodyPathWaypoints.get(i)));
      }

      // todo should be a copy
      this.heightMapData = other.heightMapData;

      if (other.referencePlan != null)
         this.referencePlan = new FootstepPlan(other.referencePlan);
   }
}
