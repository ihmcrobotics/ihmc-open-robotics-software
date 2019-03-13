package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.StepInPlaceBehavior.API;
import us.ihmc.messager.Messager;
import us.ihmc.messager.kryo.KryoMessager;

public class BehaviorTeleop
{
   private final Messager moduleMessager;

   public static BehaviorTeleop createForUI(DRCRobotModel robotModel, String backpackAddress)
   {
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

   private BehaviorTeleop(DRCRobotModel robotModel, Messager backpackMessager)
   {
      this.moduleMessager = backpackMessager;
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
