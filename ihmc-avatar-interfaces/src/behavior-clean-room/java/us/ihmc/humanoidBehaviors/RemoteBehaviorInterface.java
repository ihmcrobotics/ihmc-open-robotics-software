package us.ihmc.humanoidBehaviors;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.Messager;
import us.ihmc.messager.kryo.KryoMessager;

public class RemoteBehaviorInterface
{
   public static Messager createForUI(String backpackAddress)
   {
      KryoMessager moduleMessager = KryoMessager.createClient(BehaviorModule.getBehaviorAPI(),
                                                              backpackAddress,
                                                              NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(),
                                                              RemoteBehaviorInterface.class.getSimpleName(),
                                                              5);
      ExceptionTools.handle(() -> moduleMessager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return moduleMessager;
   }

   public static Messager createForTest(Messager messager)
   {
      return messager;
   }
}
