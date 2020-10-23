package us.ihmc.humanoidBehaviors;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.tools.BehaviorMessagerUpdateThread;
import us.ihmc.messager.Messager;
import us.ihmc.messager.kryo.KryoMessager;

public class RemoteBehaviorInterface
{
   public static Messager createForUI(BehaviorRegistry behaviorRegistry, String behaviorModuleAddress)
   {
      KryoMessager moduleMessager = KryoMessager.createClient(behaviorRegistry.getMessagerAPI(),
                                                              behaviorModuleAddress,
                                                              NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(),
                                                              new BehaviorMessagerUpdateThread(RemoteBehaviorInterface.class.getSimpleName(), 5));
      ThreadTools.startAThread(() -> ExceptionTools.handle(() -> moduleMessager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION),
                               "KryoMessagerAsyncConnectionThread");
      return moduleMessager;
   }
}
