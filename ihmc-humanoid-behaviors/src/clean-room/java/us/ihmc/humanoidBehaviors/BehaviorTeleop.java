package us.ihmc.humanoidBehaviors;

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
   private final Messager messager;

   public static BehaviorTeleop createForTeleop(DRCRobotModel robotModel, String backpackAddress)
   {
      KryoMessager messager = KryoMessager
            .createClient(BehaviorModule.getBehaviorAPI(), backpackAddress, NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(), BehaviorTeleop.class.getSimpleName(),
                          5);
      ExceptionTools.handle(() -> messager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return new BehaviorTeleop(robotModel, messager);
   }

   public static BehaviorTeleop createForTest(DRCRobotModel robotModel, Messager messager)
   {
      return new BehaviorTeleop(robotModel, messager);
   }

   private BehaviorTeleop(DRCRobotModel robotModel, Messager messager)
   {
      this.messager = messager;

      LogTools.info("Waiting for teleop to connect");
      while (!messager.isMessagerOpen());
   }

   public void setStepping(boolean stepping)
   {
      messager.submitMessage(API.Stepping, stepping);
   }

   public void abort()
   {
      messager.submitMessage(API.Abort, true);
   }
}
