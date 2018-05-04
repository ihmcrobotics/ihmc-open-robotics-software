package us.ihmc.avatar.networkProcessor;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;

import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;

public class DRCNetworkModuleParameters
{
   private boolean useNetworkProcessor = true;
   private boolean useController;
   private boolean useSensorModule;
   private boolean useSimulatedSensors;
   private boolean useUiModule;
   private boolean useBehaviorModule;
   private boolean useBehaviorVisualizer;
   private boolean useHandModule;
   private boolean usePerceptionModule;
   private boolean useRosModule;
   private boolean useROSAPICommunicator;
   private boolean useZeroPoseRobotConfigurationPublisher;
   private boolean useMocapModule;
   private boolean useLocalControllerCommunicator;
   private boolean runAutomaticDiagnostic;
   private boolean useMultisenseManualTestModule;
   private boolean useDrillDetectionModule;
   private boolean useConstrainedWholeBodyPlanningToolbox;
   private boolean useConstrainedWholeBodyPlanningToolboxVisualizer;
   private boolean useKinematicsToolbox;
   private boolean useFootstepPlanningToolbox;
   private boolean useKinematicsToolboxVisualizer;
   private boolean useFootstepPlanningToolboxVisualizer;
   private boolean useTextToSpeechModule;
   private boolean useRobotEnvironmentAwarenessModule;
   private boolean useHeightQuadTreeToolbox;
   private boolean useRemoteObjectDetectionFeedback;
   private boolean useLidarScanLogger;
   private boolean filterControllerInputMessages;
   private boolean enableJoystickBasedStepping;

   private LocalObjectCommunicator simulatedSensorCommunicator;

   private HashMap<NetworkPorts, PacketDestination> extraIntraProcessCommunicatorPorts = new HashMap<NetworkPorts, PacketDestination>();
   private URI rosUri;

   private double timeToWaitBeforeStartingDiagnostics = Double.NaN;

   public boolean isSensorModuleEnabled()
   {
      return useSensorModule;
   }

   public boolean isPerceptionModuleEnabled()
   {
      return usePerceptionModule;
   }

   public boolean isBehaviorModuleEnabled()
   {
      return useBehaviorModule;
   }

   public boolean isBehaviorVisualizerEnabled()
   {
      return useBehaviorVisualizer;
   }
   
   public boolean isConstrainedWholeBodyPlanningToolboxEnabled()
   {
      return useConstrainedWholeBodyPlanningToolbox;
   }

   public boolean isKinematicsToolboxEnabled()
   {
      return useKinematicsToolbox;
   }

   public boolean isFootstepPlanningToolboxEnabled()
   {
      return useFootstepPlanningToolbox;
   }

   public boolean isKinematicsToolboxVisualizerEnabled()
   {
      return useKinematicsToolboxVisualizer;
   }
   
   public boolean isConstrainedWholeBodyToolboxVisualizerEnabled()
   {
      return useConstrainedWholeBodyPlanningToolboxVisualizer;
   }

   public boolean isFootstepPlanningToolboxVisualizerEnabled()
   {
      return useFootstepPlanningToolboxVisualizer;
   }

   public boolean isHandModuleEnabled()
   {
      return useHandModule;
   }

   public boolean isSimulatedSensorsEnabled()
   {
      return useSimulatedSensors;
   }

   public boolean isUiModuleEnabled()
   {
      return useUiModule;
   }

   public boolean isMultisenseManualTestModuleEnabled()
   {
      return useMultisenseManualTestModule;
   }

   public boolean isRobotEnvironmentAwerenessModuleEnabled()
   {
      return useRobotEnvironmentAwarenessModule;
   }

   public boolean isHeightQuadTreeToolboxEnabled()
   {
      return useHeightQuadTreeToolbox;
   }

   public boolean isLidarScanLoggerEnabled()
   {
      return useLidarScanLogger;
   }

   public void enableMultisenseManualTestModule(boolean b)
   {
      useMultisenseManualTestModule = b;
   }

   public void enableZeroPoseRobotConfigurationPublisherModule(boolean b)
   {
      useZeroPoseRobotConfigurationPublisher = b;
   }

   public void enableSensorModule(boolean b)
   {
      useSensorModule = b;
      if (b)
         useController = true;
   }

   public void enableUiModule(boolean b)
   {
      useUiModule = b;
   }

   public void enableBehaviorModule(boolean b)
   {
      useBehaviorModule = b;
      if (b)
         useController = true;
   }

   public void enableBehaviorVisualizer(boolean useBehaviorVisualizer)
   {
      this.useBehaviorVisualizer = useBehaviorVisualizer;
   }

   public void enableKinematicsToolbox(boolean useKinematicsToolbox)
   {
      this.useKinematicsToolbox = useKinematicsToolbox;
   }
   
   public void enableWholeBodyTrajectoryToolbox(boolean useConstrainedWholeBodyPlanningToolbox)
   {
      this.useConstrainedWholeBodyPlanningToolbox = useConstrainedWholeBodyPlanningToolbox;
   }
   
   public void enableFootstepPlanningToolbox(boolean useFootstepPlanningToolbox)
   {
      this.useFootstepPlanningToolbox = useFootstepPlanningToolbox;
   }

   public void enableKinematicsToolboxVisualizer(boolean useKinematicsToolboxVisualizer)
   {
      this.useKinematicsToolboxVisualizer = useKinematicsToolboxVisualizer;
   }
   
   public void enableConstrainedWholeBodyPlanningToolboxVisualizer(boolean useConstrainedWholeBodyPlanningToolboxVisualizer)
   {
      this.useConstrainedWholeBodyPlanningToolboxVisualizer = useConstrainedWholeBodyPlanningToolboxVisualizer;
   }

   public void enableFootstepPlanningToolboxVisualizer(boolean useFootstepPlanningToolboxVisualizer)
   {
      this.useFootstepPlanningToolboxVisualizer = useFootstepPlanningToolboxVisualizer;
   }

   public void enableHandModule(boolean b)
   {
      useHandModule = b;
   }

   public void enablePerceptionModule(boolean b)
   {
      usePerceptionModule = b;
      if (b)
         useController = true;
   }

   public void enableRobotEnvironmentAwerenessModule(boolean enable)
   {
      this.useRobotEnvironmentAwarenessModule = enable;
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

   public void enableLocalControllerCommunicator(boolean useLocalControllerCommunicator)
   {
      this.useLocalControllerCommunicator = useLocalControllerCommunicator;
      if(useLocalControllerCommunicator)
      {
         this.useController = true;
      }
   }

   public boolean isLocalControllerCommunicatorEnabled()
   {
      return useLocalControllerCommunicator;
   }

   public boolean isZeroPoseRobotConfigurationPublisherEnabled()
   {
      return useZeroPoseRobotConfigurationPublisher;
   }

   public void enableROSAPICommunicator(boolean useROSAPICommunicator)
   {
      this.useROSAPICommunicator = useROSAPICommunicator;
   }

   public boolean isROSAPICommunicatorEnabled()
   {
      return useROSAPICommunicator;
   }

   public void enableControllerCommunicator(boolean useControllerCommunicator)
   {
      this.useController = useControllerCommunicator;
   }

   public boolean isControllerCommunicatorEnabled()
   {
      return useController;
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

   public boolean isDrillDetectionModuleEnabled()
   {
      return useDrillDetectionModule;
   }

   public boolean isRemoteObjectDetectionFeedbackEnabled()
   {
      return useRemoteObjectDetectionFeedback;
   }

   public void enableLidarScanLogger(boolean enableLidarScanLogger)
   {
      this.useLidarScanLogger = enableLidarScanLogger;
   }

   public void setUseRemoteObjectDetectionFeedbackEnabled(boolean useRemoteObjectDetectionFeedback)
   {
      this.useRemoteObjectDetectionFeedback = useRemoteObjectDetectionFeedback;
   }

   public void setDrillDetectionModuleEnabled(boolean b)
   {
      useDrillDetectionModule = b;
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
      useController = true;
   }

   public LocalObjectCommunicator getSimulatedSensorCommunicator()
   {
      return simulatedSensorCommunicator;
   }

   @Override
   public String toString()
   {
      return "DRCNetworkModuleParameters [useController=" + useController + "\n useSensorModule=" + useSensorModule + "\n useSimulatedSensors="
            + useSimulatedSensors + "\n useUiModule=" + useUiModule + "\n useBehaviorModule=" + useBehaviorModule + "\n useBehaviorVisualizer="
            + useBehaviorVisualizer + "\n useHandModule=" + useHandModule + "\n usePerceptionModule=" + usePerceptionModule + "\n useRosModule="
            + useRosModule + "\n useDrillDetectionModule=" + useDrillDetectionModule + "\n simulatedSensorCommunicator=" + simulatedSensorCommunicator
            + "\n rosUri=" + rosUri + "]";
   }

   public void addRobotSpecificModuleCommunicatorPort(NetworkPorts networkPort, PacketDestination communicatorId)
   {
      extraIntraProcessCommunicatorPorts.put(networkPort, communicatorId);
   }

   public HashMap<NetworkPorts, PacketDestination> getRobotSpecificModuleCommunicatorPorts()
   {
      return extraIntraProcessCommunicatorPorts;
   }

   public boolean isFilterControllerInputMessages()
   {
      return filterControllerInputMessages;
   }

   public void setFilterControllerInputMessages(boolean filterControllerInputMessages)
   {
      this.filterControllerInputMessages = filterControllerInputMessages;
   }

   public boolean isEnableJoystickBasedStepping()
   {
      return enableJoystickBasedStepping;
   }

   public void setEnableJoystickBasedStepping(boolean enableJoystickBasedStepping)
   {
      this.enableJoystickBasedStepping = enableJoystickBasedStepping;
   }
}
