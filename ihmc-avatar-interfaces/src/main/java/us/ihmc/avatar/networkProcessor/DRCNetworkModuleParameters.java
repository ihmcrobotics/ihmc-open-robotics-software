package us.ihmc.avatar.networkProcessor;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.communication.net.LocalObjectCommunicator;

public class DRCNetworkModuleParameters
{
   private boolean useNetworkProcessor = true;
   private boolean useSensorModule;
   private boolean useSimulatedSensors;
   private boolean useBehaviorModule;
   private boolean useBehaviorVisualizer;
   private boolean useRosModule;
   private boolean useZeroPoseRobotConfigurationPublisher;
   private boolean useMocapModule;
   private boolean runAutomaticDiagnostic;
   private boolean useWholeBodyTrajectoryToolbox;
   private boolean useWholeBodyTrajectoryToolboxVisualizer;
   private boolean useKinematicsToolbox;
   private boolean useKinematicsToolboxVisualizer;
   private boolean useKinematicsPlanningToolbox;
   private boolean useKinematicsStreamingToolbox;
   /**
    * When provided, the toolbox is started on a separate process to improve real-time performance by
    * isolating the toolbox. The class provided has to come with a main method.
    */
   private Class<? extends KinematicsStreamingToolboxModule> kinematicsStreamingToolboxLauncherClass;
   private boolean useFootstepPlanningToolbox;
   private boolean useFootstepPostProcessingToolbox;
   private boolean useFootstepPlanningToolboxVisualizer;
   private boolean useFootstepPostProcessingToolboxVisualizer;
   private boolean useTextToSpeechModule;
   private boolean useRobotEnvironmentAwarenessModule;
   private boolean useHeightQuadTreeToolbox;
   private boolean useBipedalSupportPlanarRegionPublisher;
   private boolean useWalkingPreviewToolbox;
   private boolean useAutoREAStateUpdater;

   private LocalObjectCommunicator simulatedSensorCommunicator;

   private URI rosUri;

   private double timeToWaitBeforeStartingDiagnostics = Double.NaN;

   public boolean isSensorModuleEnabled()
   {
      return useSensorModule;
   }

   public boolean isBehaviorModuleEnabled()
   {
      return useBehaviorModule;
   }

   public boolean isBehaviorVisualizerEnabled()
   {
      return useBehaviorVisualizer;
   }

   public boolean isWholeBodyTrajectoryToolboxEnabled()
   {
      return useWholeBodyTrajectoryToolbox;
   }

   public boolean isKinematicsToolboxEnabled()
   {
      return useKinematicsToolbox;
   }

   public boolean isKinematicsPlanningToolboxEnabled()
   {
      return useKinematicsPlanningToolbox;
   }

   public boolean isKinematicsStreamingToolboxEnabled()
   {
      return useKinematicsStreamingToolbox;
   }

   public Class<? extends KinematicsStreamingToolboxModule> getKinematicsStreamingToolboxLauncherClass()
   {
      return kinematicsStreamingToolboxLauncherClass;
   }

   public boolean isFootstepPlanningToolboxEnabled()
   {
      return useFootstepPlanningToolbox;
   }

   public boolean isFootstepPostProcessingToolboxEnabled()
   {
      return useFootstepPostProcessingToolbox;
   }

   public boolean isKinematicsToolboxVisualizerEnabled()
   {
      return useKinematicsToolboxVisualizer;
   }

   public boolean isWholeBodyTrajectoryToolboxVisualizerEnabled()
   {
      return useWholeBodyTrajectoryToolboxVisualizer;
   }

   public boolean isFootstepPlanningToolboxVisualizerEnabled()
   {
      return useFootstepPlanningToolboxVisualizer;
   }

   public boolean isFootstepPostProcessingToolboxVisualizerEnabled()
   {
      return useFootstepPostProcessingToolboxVisualizer;
   }

   public boolean isSimulatedSensorsEnabled()
   {
      return useSimulatedSensors;
   }

   public boolean isRobotEnvironmentAwerenessModuleEnabled()
   {
      return useRobotEnvironmentAwarenessModule;
   }

   public boolean isBipedalSupportPlanarRegionPublisherEnabled()
   {
      return useBipedalSupportPlanarRegionPublisher;
   }

   public boolean isWalkingPreviewToolboxEnabled()
   {
      return useWalkingPreviewToolbox;
   }

   public boolean isAutoREAStateUpdaterEnabled()
   {
      return useAutoREAStateUpdater;
   }

   public boolean isHeightQuadTreeToolboxEnabled()
   {
      return useHeightQuadTreeToolbox;
   }

   public void enableZeroPoseRobotConfigurationPublisherModule(boolean b)
   {
      useZeroPoseRobotConfigurationPublisher = b;
   }

   public void enableSensorModule(boolean b)
   {
      useSensorModule = b;
   }

   public void enableBehaviorModule(boolean b)
   {
      useBehaviorModule = b;
   }

   public void enableBehaviorVisualizer(boolean useBehaviorVisualizer)
   {
      this.useBehaviorVisualizer = useBehaviorVisualizer;
   }

   public void enableKinematicsToolbox(boolean useKinematicsToolbox)
   {
      this.useKinematicsToolbox = useKinematicsToolbox;
   }

   public void enableKinematicsPlanningToolbox(boolean useKinematicsPlanningToolbox)
   {
      this.useKinematicsPlanningToolbox = useKinematicsPlanningToolbox;
   }

   public void enableKinematicsStreamingToolbox(boolean useKinematicsStreamingToolbox)
   {
      enableKinematicsStreamingToolbox(useKinematicsStreamingToolbox, null);
   }

   public void enableKinematicsStreamingToolbox(boolean useKinematicsStreamingToolbox,
                                                Class<? extends KinematicsStreamingToolboxModule> kinematicsStreamingToolboxLauncherClass)
   {
      this.useKinematicsStreamingToolbox = useKinematicsStreamingToolbox;
      this.kinematicsStreamingToolboxLauncherClass = kinematicsStreamingToolboxLauncherClass;
   }

   public void enableWholeBodyTrajectoryToolbox(boolean useConstrainedWholeBodyPlanningToolbox)
   {
      this.useWholeBodyTrajectoryToolbox = useConstrainedWholeBodyPlanningToolbox;
   }

   public void enableFootstepPlanningToolbox(boolean useFootstepPlanningToolbox)
   {
      this.useFootstepPlanningToolbox = useFootstepPlanningToolbox;
   }

   public void enableFootstepPostProcessingToolbox(boolean useFootstepPostProcessingToolbox)
   {
      this.useFootstepPostProcessingToolbox = useFootstepPostProcessingToolbox;
   }

   public void enableKinematicsToolboxVisualizer(boolean useKinematicsToolboxVisualizer)
   {
      this.useKinematicsToolboxVisualizer = useKinematicsToolboxVisualizer;
   }

   public void enableConstrainedWholeBodyPlanningToolboxVisualizer(boolean useConstrainedWholeBodyPlanningToolboxVisualizer)
   {
      this.useWholeBodyTrajectoryToolboxVisualizer = useConstrainedWholeBodyPlanningToolboxVisualizer;
   }

   public void enableFootstepPlanningToolboxVisualizer(boolean useFootstepPlanningToolboxVisualizer)
   {
      this.useFootstepPlanningToolboxVisualizer = useFootstepPlanningToolboxVisualizer;
   }

   public void enableFootstepPostProcessingToolboxVisualizer(boolean useFootstepPostProcessingToolboxVisualizer)
   {
      this.useFootstepPostProcessingToolboxVisualizer = useFootstepPostProcessingToolboxVisualizer;
   }

   public void enableWalkingPreviewToolbox(boolean useWalkingPreviewToolbox)
   {
      this.useWalkingPreviewToolbox = useWalkingPreviewToolbox;
   }

   public void enableAutoREAStateUpdater(boolean useAutoREAStateUpdater)
   {
      this.useAutoREAStateUpdater = useAutoREAStateUpdater;
   }

   public void enableRobotEnvironmentAwerenessModule(boolean enable)
   {
      this.useRobotEnvironmentAwarenessModule = enable;
   }

   public void enableBipedalSupportPlanarRegionPublisher(boolean enable)
   {
      this.useBipedalSupportPlanarRegionPublisher = enable;
   }

   public void enableHeightQuadTreeToolbox(boolean useHeightQuadTreeToolbox)
   {
      this.useHeightQuadTreeToolbox = useHeightQuadTreeToolbox;
   }

   public void enableRosModule(boolean b)
   {
      useRosModule = b;
      if (useRosModule && rosUri == null)
         try
         {
            rosUri = new URI("http://localhost:11311");
         }
         catch (URISyntaxException e)
         {
            e.printStackTrace();
         }
   }

   public boolean isRosModuleEnabled()
   {
      return useRosModule;
   }

   public void enableAutomaticDiagnostic(boolean runAutomaticDiagnostic, double timeToWaitBeforeStartingDiagnostics)
   {
      this.runAutomaticDiagnostic = runAutomaticDiagnostic;
      this.timeToWaitBeforeStartingDiagnostics = timeToWaitBeforeStartingDiagnostics;
      if (runAutomaticDiagnostic)
      {
         enableBehaviorModule(true);
      }
   }

   public boolean isAutomaticDiagnosticEnabled()
   {
      return runAutomaticDiagnostic;
   }

   public boolean isZeroPoseRobotConfigurationPublisherEnabled()
   {
      return useZeroPoseRobotConfigurationPublisher;
   }

   public void enableNetworkProcessor(boolean useNetworkProcessor)
   {
      this.useNetworkProcessor = useNetworkProcessor;
   }

   public boolean isNetworkProcessorEnabled()
   {
      return useNetworkProcessor;
   }

   public void enableMocapModule(boolean enableMocapModule)
   {
      this.useMocapModule = enableMocapModule;
   }

   public boolean isMocapModuleEnabled()
   {
      return this.useMocapModule;
   }

   public void enableTextToSpeechModule(boolean useTextToSpeechModule)
   {
      this.useTextToSpeechModule = useTextToSpeechModule;
   }

   public boolean isTextToSpeechModuleEnabled()
   {
      return useTextToSpeechModule;
   }

   public void setRosUri(URI rosURI)
   {
      rosUri = rosURI;
   }

   public URI getRosUri()
   {
      return rosUri;
   }

   public double getTimeToWaitBeforeStartingDiagnostics()
   {
      return timeToWaitBeforeStartingDiagnostics;
   }

   public void setSimulatedSensorCommunicator(LocalObjectCommunicator simulatedSensorCommunicator)
   {
      this.simulatedSensorCommunicator = simulatedSensorCommunicator;
      useSensorModule = true;
      useSimulatedSensors = true;
   }

   public LocalObjectCommunicator getSimulatedSensorCommunicator()
   {
      return simulatedSensorCommunicator;
   }

   @Override
   public String toString()
   {
      return "DRCNetworkModuleParameters [useSensorModule=" + useSensorModule + "\n useSimulatedSensors=" + useSimulatedSensors + "\n useBehaviorModule="
            + useBehaviorModule + "\n useBehaviorVisualizer=" + useBehaviorVisualizer + "\n useRosModule=" + useRosModule + "\n simulatedSensorCommunicator="
            + simulatedSensorCommunicator + "\n rosUri=" + rosUri + "]";
   }
}
