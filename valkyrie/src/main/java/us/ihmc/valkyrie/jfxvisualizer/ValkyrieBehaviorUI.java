package us.ihmc.valkyrie.jfxvisualizer;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieBehaviorUI
{
   private static final boolean launchBehaviorModule = false;

   public ValkyrieBehaviorUI()
   {
      DRCRobotModel drcRobotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT);

      final Messager behaviorMessager = RemoteBehaviorInterface.createForUI("localhost");
      JavaFXApplicationCreator.attachStopListener(() -> ExceptionTools.handle(() -> behaviorMessager.closeMessager(),
                                                                              DefaultExceptionHandler.RUNTIME_EXCEPTION));

      new BehaviorUI(behaviorMessager, drcRobotModel, PubSubImplementation.FAST_RTPS);

      ExceptionTools.handle(() -> behaviorMessager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      if (launchBehaviorModule)
      {
         // launch behavior module
      }
   }

   public static void main(String[] args)
   {
      new ValkyrieBehaviorUI();
   }
}
