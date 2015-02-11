package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.net.URI;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;

public class DRCNetworkModuleParameters
{
   private boolean useController;
   private boolean useSensorModule;      
   private boolean useSimulatedSensors;  
   private boolean useUiModule;          
   private boolean useBehaviorModule;    
   private boolean useHandModule;        
   private boolean usePerceptionModule;  
   private boolean useRosModule;     
   
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
   }

   public void setUseUiModule(boolean b)
   {
      useUiModule = b;
   }

   public void setUseBehaviorModule(boolean b)
   {
      useBehaviorModule = b;
   }

   public void setUseHandModule(boolean b)
   {
      useHandModule = b;
   }

   public void setUsePerceptionModule(boolean b)
   {
      usePerceptionModule = b;
   }

   public void setUseRosModule(boolean b)
   {
      useRosModule = b;
   }

   public void setRosUri(URI rosURI)
   {
      rosUri = rosURI;
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
   }
}
