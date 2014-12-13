package us.ihmc.humanoidBehaviors.communication;

import java.util.HashMap;

import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.communication.net.ObjectCommunicator;

public class BehaviorPacketPassThroughManager implements GlobalObjectConsumer
{

   private final ObjectCommunicator toCommunicator;
   private final HashMap<Class, Boolean> classPassthroughMap;
   private boolean passThroughActive;

   public BehaviorPacketPassThroughManager(ObjectCommunicator fromCommunicator, ObjectCommunicator toCommunicator, Class[] packetsToPass)
   {
      this.toCommunicator = toCommunicator;

      this.classPassthroughMap = new HashMap<Class, Boolean>();

      for (Class clz : packetsToPass)
      {
         classPassthroughMap.put(clz, true);
      }

      passThroughActive = true;
      fromCommunicator.attachGlobalListener(this);
   }
   
   public void setPassThrough(Class clzz, boolean activatePassthrough)
   {
      classPassthroughMap.put(clzz, activatePassthrough);
   }

   @Override
   public void consumeObject(Object object)
   {

   }

   @Override
   public void consumeObject(Object object, boolean consumeGlobal)
   {
      if (passThroughActive && classPassthroughMap.containsKey(object.getClass()) && classPassthroughMap.get(object.getClass()))
      {
         toCommunicator.consumeObject(object);
      }
   }

   public void setPassthroughActive(boolean passThroughActive)
   {
      this.passThroughActive = passThroughActive;
   }
}
