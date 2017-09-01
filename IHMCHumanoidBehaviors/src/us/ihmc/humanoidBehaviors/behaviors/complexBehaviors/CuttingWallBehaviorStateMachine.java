package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.CuttingWallBehaviorStateMachine.CuttingWallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoDouble;

public class CuttingWallBehaviorStateMachine extends StateMachineBehavior<CuttingWallBehaviorState> implements CoactiveDataListenerInterface
{

   private HandTrajectoryBehavior leftHandTrajectoryBehavior;
   private HandTrajectoryBehavior rightHandTrajectoryBehavior;

   private WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;

   private CommunicationBridge communicationBridge;

   public enum CuttingWallBehaviorState
   {
      WAITING, PLANNING, CUTTING
   }

   public CuttingWallBehaviorStateMachine(CommunicationBridge communicationBridge, YoDouble yoTime, FullHumanoidRobotModel fullRobotModel,
                                          HumanoidReferenceFrames referenceFrames)
   {
      super("cuttingWallBehaviorState", CuttingWallBehaviorState.class, yoTime, communicationBridge);

      this.communicationBridge = communicationBridge;
      communicationBridge.addListeners(this);

      this.leftHandTrajectoryBehavior = new HandTrajectoryBehavior("left", communicationBridge, yoTime);
      this.rightHandTrajectoryBehavior = new HandTrajectoryBehavior("right", communicationBridge, yoTime);

      this.wholebodyTrajectoryBehavior = new WholeBodyTrajectoryBehavior(communicationBridge, yoTime);

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      BehaviorAction<CuttingWallBehaviorState> waiting = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.WAITING,
                                                                                                      leftHandTrajectoryBehavior, rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput WAITING");

            HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.5, 0.4, 1.0), new Quaternion(),
                                                                                        ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.5, -0.4, 1.0), new Quaternion(),
                                                                                         ReferenceFrame.getWorldFrame());

            leftHandTrajectoryBehavior.setInput(leftHandTrajectoryMessage);
            rightHandTrajectoryBehavior.setInput(rightHandTrajectoryMessage);
         }
      };

      BehaviorAction<CuttingWallBehaviorState> planning = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.PLANNING,
                                                                                                       leftHandTrajectoryBehavior, rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput PLANNING");

            HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.4, 1.0), new Quaternion(),
                                                                                        ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.6, -0.4, 1.0), new Quaternion(),
                                                                                         ReferenceFrame.getWorldFrame());

            leftHandTrajectoryBehavior.setInput(leftHandTrajectoryMessage);
            rightHandTrajectoryBehavior.setInput(rightHandTrajectoryMessage);
         }
      };

      BehaviorAction<CuttingWallBehaviorState> cutting = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.CUTTING,
                                                                                                      leftHandTrajectoryBehavior, rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput CUTTING");

            HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.5, 0.4, 1.0), new Quaternion(),
                                                                                        ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.5, -0.4, 1.0), new Quaternion(),
                                                                                         ReferenceFrame.getWorldFrame());

            leftHandTrajectoryBehavior.setInput(leftHandTrajectoryMessage);
            rightHandTrajectoryBehavior.setInput(rightHandTrajectoryMessage);
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
