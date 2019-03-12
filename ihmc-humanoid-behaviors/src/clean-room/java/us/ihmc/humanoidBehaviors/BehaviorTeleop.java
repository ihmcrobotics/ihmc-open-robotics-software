package us.ihmc.humanoidBehaviors;

import com.esotericsoftware.minlog.Log;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.StepInPlaceBehavior.API;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.kryo.KryoMessager;

public class BehaviorTeleop
{
   private final Messager moduleMessager;

   public static BehaviorTeleop createForUI(DRCRobotModel robotModel, String backpackAddress)
   {
      Log.TRACE();
      KryoMessager backpackMessager = KryoMessager
            .createClient(BehaviorBackpack.getBehaviorAPI(), backpackAddress, NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(), BehaviorTeleop.class.getSimpleName(),
                          5);
      ExceptionTools.handle(() -> backpackMessager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return new BehaviorTeleop(robotModel, backpackMessager);
   }

   public static BehaviorTeleop createForTest(DRCRobotModel robotModel, Messager messager)
   {
      return new BehaviorTeleop(robotModel, messager);
   }

   private BehaviorTeleop(DRCRobotModel robotModel, Messager moduleMessager)
   {
      this.moduleMessager = moduleMessager;

      LogTools.info("Waiting for teleop to connect");
      while (!moduleMessager.isMessagerOpen());
   }

   public void setStepping(boolean stepping)
   {
      moduleMessager.submitMessage(API.Stepping, stepping);
   }

   public void abort()
   {
      moduleMessager.submitMessage(API.Abort, true);
   }
}
