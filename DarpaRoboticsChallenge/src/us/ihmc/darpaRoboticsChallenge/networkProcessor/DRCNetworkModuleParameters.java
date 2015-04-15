package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.communication.net.LocalObjectCommunicator;

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
   private boolean useGFECommunicator;
   private boolean useMocapModule;
   private boolean useLocalControllerCommunicator;
   private boolean runAutomaticDiagnostic = false;

   private LocalObjectCommunicator simulatedSensorCommunicator;
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
   
   public void enableGFECommunicator(boolean useGFECommunicator)
   {
      this.useGFECommunicator = useGFECommunicator;
   }
   
   public boolean isGFECommunicatorEnabled()
   {
      return useGFECommunicator;
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
            + useBehaviorVisualizer + "\n useHandModule=" + useHandModule + "\n usePerceptionModule=" + usePerceptionModule + "\n useRosModule=" + useRosModule
            + "\n simulatedSensorCommunicator=" + simulatedSensorCommunicator + "\n rosUri=" + rosUri
            + "]";
   }
}
