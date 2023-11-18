package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.activeMapping.ActivePlanarMappingRemoteTask;
import us.ihmc.behaviors.activeMapping.ContinuousWalkingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloWaypointAgent;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlanningWorld;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanningTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.ros2.ROS2Node;

public class HumanoidActivePerceptionModule
{
   /* For storing world and agent states when active mapping module is disabled */
   private MonteCarloPlanningWorld world;
   private MonteCarloWaypointAgent agent;

   /* For displaying occupancy grid from the active mapping module. */
   private final Mat gridColor = new Mat();

   private ActivePlanarMappingRemoteTask activePlaneMappingRemoteThread;
   private ContinuousPlannerSchedulingTask continuousPlannerSchedulingThread;

   private PerceptionConfigurationParameters perceptionConfigurationParameters;
   private final ContinuousWalkingParameters continuousPlanningParameters;

   public HumanoidActivePerceptionModule(PerceptionConfigurationParameters perceptionConfigurationParameters,
                                         ContinuousWalkingParameters continuousWalkingParameters)
   {
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
      this.continuousPlanningParameters = continuousWalkingParameters;
   }

   public void initializeActivePlaneMappingTask(String robotName, DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames, ROS2Node ros2Node)
   {
      LogTools.info("Initializing Active Mapping Process");
      activePlaneMappingRemoteThread = new ActivePlanarMappingRemoteTask(robotName, robotModel,
                                                                         continuousPlanningParameters,
                                                                         PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                                         PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE,
                                                                         ros2Node, referenceFrames, () -> {}, true);
   }

   public void initializeContinuousElevationMappingTask(DRCRobotModel robotModel, ROS2Node ros2Node, HumanoidReferenceFrames referenceFrames)
   {
      continuousPlannerSchedulingThread = new ContinuousPlannerSchedulingTask(robotModel, ros2Node, referenceFrames, continuousPlanningParameters);
   }

   public void update(ReferenceFrame sensorFrame, boolean display)
   {
      if (activePlaneMappingRemoteThread == null)
      {
         int gridX = ContinuousPlanningTools.getIndexFromCoordinates(sensorFrame.getTransformToWorldFrame().getTranslationX(),
                                                                     perceptionConfigurationParameters.getOccupancyGridResolution(),
                                                                     70);
         int gridY = ContinuousPlanningTools.getIndexFromCoordinates(sensorFrame.getTransformToWorldFrame().getTranslationY(),
                                                                     perceptionConfigurationParameters.getOccupancyGridResolution(),
                                                                     70);

         agent.changeStateTo(gridX, gridY);
         agent.measure(world);

         if (display)
         {
            MonteCarloPlannerTools.plotWorld(world, gridColor);
            MonteCarloPlannerTools.plotAgent(agent, gridColor);
            MonteCarloPlannerTools.plotRangeScan(agent.getScanPoints(), gridColor);

            PerceptionDebugTools.display("Monte Carlo Planner World", gridColor, 1, 1400);
         }
      }
   }

   public void initializeOccupancyGrid(int depthHeight, int depthWidth, int gridHeight, int gridWidth)
   {
      if (activePlaneMappingRemoteThread != null)
      {
         LogTools.warn("Initializing Occupancy Grid from Active Mapping Remote Process");

         world = activePlaneMappingRemoteThread.getActiveMappingModule().getPlanner().getWorld();
         agent = activePlaneMappingRemoteThread.getActiveMappingModule().getPlanner().getAgent();
      }
      else
      {
         LogTools.warn("Initializing Occupancy Grid from Scratch");

         this.world = new MonteCarloPlanningWorld(0, gridHeight, gridWidth);
         this.agent = new MonteCarloWaypointAgent(new Point2D());
      }
   }

   public Mat getOccupancyGrid()
   {
      return world.getGrid();
   }

   public void destroy()
   {
      if (activePlaneMappingRemoteThread != null)
         activePlaneMappingRemoteThread.destroy();

      if (continuousPlannerSchedulingThread != null)
         continuousPlannerSchedulingThread.destroy();
   }

   public ContinuousPlannerSchedulingTask getContinuousMappingRemoteThread()
   {
      return continuousPlannerSchedulingThread;
   }
}
