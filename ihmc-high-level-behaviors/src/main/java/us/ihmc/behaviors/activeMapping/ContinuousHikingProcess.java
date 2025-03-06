package us.ihmc.behaviors.activeMapping;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.StandAloneRealsenseProcess;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class ContinuousHikingProcess
{
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final ContinuousPlannerSchedulingTask continuousPlannerSchedulingTask;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private final StandAloneRealsenseProcess standAloneRealsenseProcess;

   public ContinuousHikingProcess(DRCRobotModel robotModel)
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("nadia_terrain_perception_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Node);

      RepeatingTaskThread robotUpdateThread = new RepeatingTaskThread("SyncedRobotUpdate", syncedRobot::update).setFrequencyLimit(30.0);
      robotUpdateThread.startRepeating();

      // Add Continuous Hiking Parameters to be between the UI and this process
      ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.CONTINUOUS_HIKING_PARAMETERS, continuousHikingParameters);

      // Add Monte Carlo Footstep Planner Parameters to be between the UI and this process
      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.MONTE_CARLO_PLANNER_PARAMETERS, monteCarloPlannerParameters);

      // Add A* Footstep Planner Parameters to be between the UI and this process
      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters("ForContinuousWalking");
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.FOOTSTEP_PLANNING_PARAMETERS, footstepPlannerParameters);

      // Add Swing Planner Parameters to be synced between the UI and this process
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.SWING_PLANNING_PARAMETERS, swingPlannerParameters);

      HeightMapParameters heightMapParameters = RapidHeightMapManager.getHeightMapParameters();
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.HEIGHT_MAP_PARAMETERS, heightMapParameters);

      ContinuousHikingLogger continuousHikingLogger = new ContinuousHikingLogger();
      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Node, robotModel.getSimpleRobotName());

      standAloneRealsenseProcess = new StandAloneRealsenseProcess(ros2Node, ros2Helper, syncedRobot, controllerFootstepQueueMonitor);

      continuousPlannerSchedulingTask = new ContinuousPlannerSchedulingTask(robotModel,
                                                                            ros2Node,
                                                                            syncedRobot,
                                                                            syncedRobot.getReferenceFrames(),
                                                                            controllerFootstepQueueMonitor,
                                                                            continuousHikingLogger,
                                                                            continuousHikingParameters,
                                                                            monteCarloPlannerParameters,
                                                                            footstepPlannerParameters,
                                                                            swingPlannerParameters);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));

      // Add initial delay to get things going in the right order
      executorService.scheduleAtFixedRate(this::update, 8500, 100, TimeUnit.MILLISECONDS);
   }

   public void update()
   {
      ros2PropertySetGroup.update();

      if (standAloneRealsenseProcess.getHeightMapManager() == null)
      {
         return;
      }

      if (standAloneRealsenseProcess.getLatestHeightMapData() != null && standAloneRealsenseProcess.getLatestTerrainMapData() != null)
      {
         continuousPlannerSchedulingTask.setLatestHeightMapData(standAloneRealsenseProcess.getLatestHeightMapData());
         continuousPlannerSchedulingTask.setTerrainMapData(standAloneRealsenseProcess.getLatestTerrainMapData());
      }
   }

   public void destroy()
   {
      continuousPlannerSchedulingTask.destroy();
      standAloneRealsenseProcess.destroy();
   }
}
