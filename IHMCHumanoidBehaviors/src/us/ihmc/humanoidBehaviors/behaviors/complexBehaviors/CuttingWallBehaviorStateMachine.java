package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.CuttingWallBehaviorStateMachine.CuttingWallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CuttingWallBehaviorStateMachine extends StateMachineBehavior<CuttingWallBehaviorState> implements CoactiveDataListenerInterface
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private CommunicationBridge communicationBridge;

   public enum CuttingWallBehaviorState
   {
      WAITING, PLANNING, CUTTING
   }

   public CuttingWallBehaviorStateMachine(CommunicationBridge communicationBridge, YoDouble yoTime, FullHumanoidRobotModel fullRobotModel,
                                          HumanoidReferenceFrames referenceFrames, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("cuttingWallBehaviorState", CuttingWallBehaviorState.class, yoTime, communicationBridge);

      this.communicationBridge = communicationBridge;
      communicationBridge.addListeners(this);

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      BehaviorAction<CuttingWallBehaviorState> waiting = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.WAITING,
                                                                                                      atlasPrimitiveActions.rightArmGoHomeBehavior,
                                                                                                      atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput WAITING");
            GoHomeMessage goHomeRightArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.RIGHT, 2);
            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(goHomeRightArmMessage);

            HandDesiredConfigurationMessage handMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(handMessage);
         }
      };

      BehaviorAction<CuttingWallBehaviorState> planning = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.PLANNING,
                                                                                                       atlasPrimitiveActions.rightArmGoHomeBehavior,
                                                                                                       atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput PLANNING");
            GoHomeMessage goHomeRightArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.RIGHT, 2);
            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(goHomeRightArmMessage);

            HandDesiredConfigurationMessage handMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(handMessage);
         }
      };

      BehaviorAction<CuttingWallBehaviorState> cutting = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.CUTTING,
                                                                                                      atlasPrimitiveActions.rightArmGoHomeBehavior,
                                                                                                      atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput CUTTING");
            GoHomeMessage goHomeRightArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.RIGHT, 2);
            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(goHomeRightArmMessage);

            HandDesiredConfigurationMessage handMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(handMessage);
         }
      };

      statemachine.addStateWithDoneTransition(waiting, CuttingWallBehaviorState.PLANNING);
      statemachine.addStateWithDoneTransition(planning, CuttingWallBehaviorState.CUTTING);

      statemachine.addState(cutting);
      statemachine.setStartState(CuttingWallBehaviorState.WAITING);
   }

   @Override
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data)
   {
      System.out.println("BEHAVIOR RECIEVED " + data.key + " " + data.value);
   }

   @Override
   public void onBehaviorExited()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("exit cuttingWallBehaviorState");
      sendPacket(p1);

   }
}
