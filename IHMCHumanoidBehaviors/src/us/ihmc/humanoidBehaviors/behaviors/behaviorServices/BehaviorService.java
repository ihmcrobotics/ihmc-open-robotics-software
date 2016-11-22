package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public abstract class BehaviorService
{
   private final CommunicationBridgeInterface communicationBridge;
   private final YoVariableRegistry registry;
   
   public BehaviorService(String name, CommunicationBridgeInterface communicationBridge)
   {
      this.communicationBridge = communicationBridge;
      registry = new YoVariableRegistry(name);
   }
   
   public abstract void initialize();
   
   public abstract void pause();
   
   public abstract void stop();
   
   public abstract void resume();
   
   protected CommunicationBridgeInterface getCommunicationBridge()
   {
      return communicationBridge;
   }
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
