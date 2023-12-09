package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class MonteCarloFootstepPlannerRequest
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
    * Height Map image in OpenCV format
    */
   private TerrainMapData terrainMapData;

   /**
    * Sensor origin that defines the center of the height map
    */
   private Point2D sensorOrigin;

   public MonteCarloFootstepPlannerRequest()
   {
      clear();
   }

   private void clear()
   {
      requestId = -1;
      requestedInitialStanceSide = RobotSide.LEFT;
      startFootPoses.forEach(Pose3D::setToNaN);
      goalFootPoses.forEach(Pose3D::setToNaN);
      goalDistanceProximity = -1.0;
      goalYawProximity = -1.0;
      timeout = 5.0;
      maximumIterations = -1;
      sensorOrigin = new Point2D();
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

   public void setTerrainMapData(TerrainMapData terrainMap)
   {
      this.terrainMapData = terrainMap;
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }

   public void setSensorOrigin(double originX, double originY)
   {
      this.sensorOrigin.set(originX, originY);
   }

   public Point2D getSensorOrigin()
   {
      return sensorOrigin;
   }

   public void setPacket(MonteCarloFootstepPlannerRequest requestPacket)
   {
      requestPacket.setRequestedInitialStanceSide(RobotSide.fromByte(getRequestedInitialStanceSide().toByte()));
      requestPacket.setGoalDistanceProximity(getGoalDistanceProximity());
      requestPacket.setGoalYawProximity(getGoalYawProximity());
      requestPacket.setTimeout(getTimeout());
   }

   public void set(MonteCarloFootstepPlannerRequest other)
   {
      clear();

      this.requestId = other.requestId;

      this.requestedInitialStanceSide = other.requestedInitialStanceSide;
      this.startFootPoses.get(RobotSide.LEFT).set(other.startFootPoses.get(RobotSide.LEFT));
      this.startFootPoses.get(RobotSide.RIGHT).set(other.startFootPoses.get(RobotSide.RIGHT));
      this.goalFootPoses.get(RobotSide.LEFT).set(other.goalFootPoses.get(RobotSide.LEFT));
      this.goalFootPoses.get(RobotSide.RIGHT).set(other.goalFootPoses.get(RobotSide.RIGHT));
      this.goalDistanceProximity = other.goalDistanceProximity;
      this.goalYawProximity = other.goalYawProximity;
      this.timeout = other.timeout;
      this.maximumIterations = other.maximumIterations;
   }
}
