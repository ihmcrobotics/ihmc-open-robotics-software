package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.activeMapping.ActiveMappingRemoteTask;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlanningAgent;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlanningWorld;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.ros2.ROS2Node;

public class HumanoidActivePerceptionModule
{
   /* For storing world and agent states when active mapping module is disabled */
   private MonteCarloPlanningWorld world;
   private MonteCarloPlanningAgent agent;

   /* For displaying occupancy grid from the active mapping module. */
   private final Mat gridColor = new Mat();

   private ActiveMappingRemoteTask activeMappingRemoteThread;

   private PerceptionConfigurationParameters perceptionConfigurationParameters;

   public HumanoidActivePerceptionModule(PerceptionConfigurationParameters perceptionConfigurationParameters)
   {
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
   }

   public void initializeActiveMappingProcess(String robotName, DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames, ROS2Node ros2Node)
   {
      LogTools.info("Initializing Active Mapping Process");
      activeMappingRemoteThread = new ActiveMappingRemoteTask(robotName,
                                                              robotModel,
                                                              PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                              PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE,
                                                              ros2Node,
                                                              referenceFrames,
                                                              () ->
                                                              {
                                                              },
                                                              true);
   }

   public void update(ReferenceFrame sensorFrame, boolean display)
   {
      if (activeMappingRemoteThread == null)
      {
         int gridX = ActiveMappingTools.getIndexFromCoordinates(sensorFrame.getTransformToWorldFrame().getTranslationX(),
                                                                perceptionConfigurationParameters.getOccupancyGridResolution(),
                                                                70);
         int gridY = ActiveMappingTools.getIndexFromCoordinates(sensorFrame.getTransformToWorldFrame().getTranslationY(),
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
      if (activeMappingRemoteThread != null)
      {
         LogTools.warn("Initializing Occupancy Grid from Active Mapping Remote Process");

         world = activeMappingRemoteThread.getActiveMappingModule().getPlanner().getWorld();
         agent = activeMappingRemoteThread.getActiveMappingModule().getPlanner().getAgent();
      }
      else
      {
         LogTools.warn("Initializing Occupancy Grid from Scratch");

         world = new MonteCarloPlanningWorld(0, gridHeight, gridWidth);
         agent = new MonteCarloPlanningAgent(new Point2D());
      }
   }

   public Mat getOccupancyGrid()
   {
      return world.getGrid();
   }

   public void destroy()
   {
      if (activeMappingRemoteThread != null)
         activeMappingRemoteThread.destroy();
   }
}
