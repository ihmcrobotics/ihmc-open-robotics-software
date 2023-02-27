package us.ihmc.avatar.simulationStarter;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.commons.FormattingTools;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;
import us.ihmc.robotEnvironmentAwareness.RemoteLidarBasedREAUILauncher;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public abstract class DRCSimulationTools
{
   @SuppressWarnings({"hiding", "unchecked"})
   public static <T extends DRCStartingLocation, Enum> void startSimulationWithGraphicSelector(SimulationStarterInterface simulationStarter,
                                                                                               Class<?> operatorInterfaceClass,
                                                                                               String[] operatorInterfaceArgs,
                                                                                               T... possibleStartingLocations)
   {
      List<Modules> modulesToStart = new ArrayList<>();
      DRCStartingLocation startingLocation = showSelectorWithStartingLocation(modulesToStart, possibleStartingLocations);

      startSimulation(simulationStarter, operatorInterfaceClass, operatorInterfaceArgs, startingLocation, modulesToStart);
   }

   @SuppressWarnings({"hiding"})
   public static <T extends DRCStartingLocation, Enum> void startSimulation(SimulationStarterInterface simulationStarter,
                                                                            Class<?> operatorInterfaceClass,
                                                                            String[] operatorInterfaceArgs,
                                                                            DRCStartingLocation startingLocation,
                                                                            List<Modules> modulesToStart)
   {
      if (startingLocation != null)
         simulationStarter.setStartingLocation(startingLocation);

      if (modulesToStart.isEmpty())
         return;

      boolean automaticallyStartSimulation = true;
      HumanoidNetworkProcessorParameters networkProcessorParameters = createNetworkProcessorParameters(modulesToStart);

      if (modulesToStart.contains(Modules.SIMULATION))
         simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);

      if (modulesToStart.contains(Modules.OPERATOR_INTERFACE))
      {
         if (modulesToStart.contains(Modules.SIMULATION))
            startOpertorInterfaceUsingProcessSpawner(operatorInterfaceClass, operatorInterfaceArgs);
         else
            startOpertorInterface(operatorInterfaceClass, operatorInterfaceArgs);
      }

      if (modulesToStart.contains(Modules.BEHAVIOR_VISUALIZER))
         simulationStarter.startBehaviorVisualizer();

      boolean startREAModule = modulesToStart.contains(Modules.REA_MODULE);
      boolean startREAUI = modulesToStart.contains(Modules.REA_UI);

      if (startREAModule && startREAUI)
         new JavaProcessSpawner(true, true).spawn(LidarBasedREAStandaloneLauncher.class);
      else if (startREAUI)
         new JavaProcessSpawner(true, true).spawn(RemoteLidarBasedREAUILauncher.class);
   }

   public static HumanoidNetworkProcessorParameters createNetworkProcessorParameters(List<Modules> modulesToStart)
   {
      HumanoidNetworkProcessorParameters networkProcessorParameters;
      if (modulesToStart.contains(Modules.NETWORK_PROCESSOR))
      {
         networkProcessorParameters = new HumanoidNetworkProcessorParameters();
         networkProcessorParameters.setUseBehaviorModule(modulesToStart.contains(Modules.BEHAVIOR_MODULE), modulesToStart.contains(Modules.BEHAVIOR_MODULE));
         networkProcessorParameters.setUseSensorModule(modulesToStart.contains(Modules.SENSOR_MODULE));
         networkProcessorParameters.setUseZeroPoseRobotConfigurationPublisherModule(modulesToStart.contains(Modules.ZERO_POSE_PRODUCER));
         networkProcessorParameters.setUseROSModule(modulesToStart.contains(Modules.ROS_MODULE));
         networkProcessorParameters.setUseKinematicsToolboxModule(modulesToStart.contains(Modules.KINEMATICS_TOOLBOX));
         networkProcessorParameters.setUseKinematicsStreamingToolboxModule(modulesToStart.contains(Modules.KINEMATICS_TOOLBOX));
         networkProcessorParameters.setUseFootstepPlanningToolboxModule(modulesToStart.contains(Modules.FOOTSTEP_PLANNING_TOOLBOX));
         networkProcessorParameters.setUseWholeBodyTrajectoryToolboxModule(modulesToStart.contains(Modules.WHOLE_BODY_TRAJECTORY_TOOLBOX));
         networkProcessorParameters.setUseKinematicsPlanningToolboxModule(modulesToStart.contains(Modules.KINEMATICS_PLANNING_TOOLBOX));
         boolean startREAModule = modulesToStart.contains(Modules.REA_MODULE) && !modulesToStart.contains(Modules.REA_UI);
         networkProcessorParameters.setUseRobotEnvironmentAwerenessModule(startREAModule);
         networkProcessorParameters.setUseBipedalSupportPlanarRegionPublisherModule(modulesToStart.contains(Modules.SENSOR_MODULE));
         networkProcessorParameters.setUseFiducialDetectorToolboxModule(modulesToStart.contains(Modules.FIDUCIAL_DETECTOR));
         networkProcessorParameters.setUseObjectDetectorToolboxModule(modulesToStart.contains(Modules.OBJECT_DETECTOR));
         networkProcessorParameters.setUseDirectionalControlModule(modulesToStart.contains(Modules.DIRECTIONAL_CONTROL_TOOLBOX));
      }
      else
      {
         networkProcessorParameters = null;
      }
      return networkProcessorParameters;
   }

   @SuppressWarnings({"hiding", "unchecked"})
   public static <T extends DRCStartingLocation, Enum> DRCStartingLocation showSelectorWithStartingLocation(List<Modules> modulesToStartListToPack,
                                                                                                            T... possibleStartingLocations)
   {
      Platform.setImplicitExit(false);
      JavaFXMissingTools.startup();

      return JavaFXMissingTools.runAndWait(() ->
      {
         SimulationSelectorStageController controller;
         try
         {
            FXMLLoader loader = new FXMLLoader(DRCSimulationTools.class.getResource("SimulationSelectorStage.fxml"));
            loader.load();
            controller = loader.getController();
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }

         T startingLocation = controller.showAndWait(possibleStartingLocations);
         modulesToStartListToPack.addAll(controller.getModulesToStart());
         return startingLocation;
      });
   }

   /**
    * Creates and starts the operator interface. The operator interface needs the simulation and
    * network processor to work properly, if started before any of these it will simply hang and wait
    * for these two to start. Use {@link #spawnOperatorInterfaceInDifferentProcess} to either start the
    * operator interface in the same process or a different one. Note that if started in a different
    * process the debug mode will not work.
    */
   public static void startOpertorInterfaceUsingProcessSpawner(Class<?> operatorInterfaceClass, String[] operatorInterfaceArgs)
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true, true);
      if (operatorInterfaceClass == null)
         return;
      spawner.spawn(operatorInterfaceClass, operatorInterfaceArgs);

   }

   public static void startOpertorInterface(Class<?> operatorInterfaceClass, String[] operatorInterfaceArgs)
   {
      if (operatorInterfaceClass == null)
         return;

      try
      {
         Method mainMethod = operatorInterfaceClass.getDeclaredMethod("main", String[].class);
         Object args[] = {operatorInterfaceArgs};
         mainMethod.invoke(null, args);
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         e.printStackTrace();
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         e.printStackTrace();
      }
   }

   public enum Modules
   {
      SIMULATION,
      OPERATOR_INTERFACE,
      BEHAVIOR_VISUALIZER,
      NETWORK_PROCESSOR,
      SENSOR_MODULE,
      ROS_MODULE,
      BEHAVIOR_MODULE,
      ZERO_POSE_PRODUCER,
      REA_MODULE,
      REA_UI,
      KINEMATICS_TOOLBOX,
      KINEMATICS_PLANNING_TOOLBOX,
      FOOTSTEP_PLANNING_TOOLBOX,
      WHOLE_BODY_TRAJECTORY_TOOLBOX,
      FIDUCIAL_DETECTOR,
      OBJECT_DETECTOR,
      DIRECTIONAL_CONTROL_TOOLBOX;

      public String getPropertyNameForEnable()
      {
         return "enable" + FormattingTools.underscoredToCamelCase(toString(), true);
      }

      public String getPropertyNameForSelected()
      {
         return "select" + FormattingTools.underscoredToCamelCase(toString(), true);
      }

      public boolean isAlwaysEnabled()
      {
         if (this == SIMULATION || this == OPERATOR_INTERFACE || this == BEHAVIOR_VISUALIZER)
            return true;
         else
            return false;
      }

      public boolean getDefaultValueForEnable()
      {
         return true;
      }

      public boolean getDefaultValueForSelected()
      {
         if (this == SIMULATION || this == OPERATOR_INTERFACE || this == NETWORK_PROCESSOR || this == SENSOR_MODULE)
            return true;
         else
            return false;
      }

      public String getName()
      {
         return FormattingTools.underscoredToCamelCase(toString(), true);
      }
   }
}
