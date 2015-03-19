package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;

public class DRCNetworkModuleParameters
{
   private boolean useController;
   private boolean useSensorModule;
   private boolean useSimulatedSensors;
   private boolean useUiModule;
   private boolean useBehaviorModule;
   private boolean useBehaviorVisualizer;
   private boolean useHandModule;
   private boolean usePerceptionModule;
   private boolean useRosModule;

   private boolean runAutomaticDiagnostic = false;

   private PacketCommunicator controllerCommunicator;
   private PacketCommunicator simulatedSensorCommunicator;
   private URI rosUri;

   public boolean useSensorModule()
   {
      return useSensorModule;
   }

   public boolean usePerceptionModule()
   {
      return usePerceptionModule;
   }

   public boolean useRosModule()
   {
      return useRosModule;
   }

   public boolean useBehaviorModule()
   {
      return useBehaviorModule;
   }

   public boolean useBehaviorVisualizer()
   {
      return useBehaviorVisualizer;
   }

   public boolean useHandModule()
   {
      return useHandModule;
   }

   public boolean useSimulatedSensors()
   {
      return useSimulatedSensors;
   }

   public boolean useController()
   {
      return useController;
   }

   public boolean useUiModule()
   {
      return useUiModule;
   }

   public PacketCommunicator getSimulatedSensorCommunicator()
   {
      return simulatedSensorCommunicator;
   }

   public PacketCommunicator getControllerCommunicator()
   {
      return controllerCommunicator;
   }

   public URI getRosUri()
   {
      return rosUri;
   }

   public void setUseSensorModule(boolean b)
   {
      useSensorModule = b;
      if (b)
         useController = true;
   }

   public void setUseUiModule(boolean b)
   {
      useUiModule = b;
   }

   public void setUseBehaviorModule(boolean b)
   {
      useBehaviorModule = b;
      if (b)
         useController = true;
   }

   public void setUseBehaviorVisualizer(boolean useBehaviorVisualizer)
   {
      this.useBehaviorVisualizer = useBehaviorVisualizer;
   }

   public void setUseHandModule(boolean b)
   {
      useHandModule = b;
   }

   public void setUsePerceptionModule(boolean b)
   {
      usePerceptionModule = b;
      if (b)
         useController = true;
   }

   public void setUseRosModule(boolean b)
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

   public void setRosUri(URI rosURI)
   {
      rosUri = rosURI;
   }

   public boolean isRunAutomaticDiagnostic()
   {
      return runAutomaticDiagnostic;
   }

   public void setRunAutomaticDiagnostic(boolean runAutomaticDiagnostic)
   {
      this.runAutomaticDiagnostic = runAutomaticDiagnostic;
      if (runAutomaticDiagnostic)
      {
         setUseBehaviorModule(true);
      }
   }

   public void setControllerCommunicator(PacketCommunicator controllerCommunicator)
   {
      this.controllerCommunicator = controllerCommunicator;
      useController = true;
   }

   public void setSimulatedSensorCommunicator(PacketCommunicator simulatedSensorCommunicator)
   {
      this.simulatedSensorCommunicator = simulatedSensorCommunicator;
      useSensorModule = true;
      useSimulatedSensors = true;
      useController = true;
   }

   @Override
   public String toString()
   {
      return "DRCNetworkModuleParameters [useController=" + useController + "\n useSensorModule=" + useSensorModule + "\n useSimulatedSensors="
            + useSimulatedSensors + "\n useUiModule=" + useUiModule + "\n useBehaviorModule=" + useBehaviorModule + "\n useBehaviorVisualizer="
            + useBehaviorVisualizer + "\n useHandModule=" + useHandModule + "\n usePerceptionModule=" + usePerceptionModule + "\n useRosModule=" + useRosModule
            + "\n controllerCommunicator=" + controllerCommunicator + "\n simulatedSensorCommunicator=" + simulatedSensorCommunicator + "\n rosUri=" + rosUri
            + "]";
   }
}
