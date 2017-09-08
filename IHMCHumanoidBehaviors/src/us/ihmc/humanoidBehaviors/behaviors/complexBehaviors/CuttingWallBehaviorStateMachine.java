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
   
   private YoDouble yoTime;

   public enum CuttingWallBehaviorState
   {
      PLANNING, WAITING_CONFIRM, MOTION
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

      this.yoTime = yoTime;
      
      setupStateMachine();
   }

   private void setupStateMachine()
   {
      BehaviorAction<CuttingWallBehaviorState> planning = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.PLANNING,
                                                                                                       leftHandTrajectoryBehavior, rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput PLANNING " + yoTime.getDoubleValue());

            HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.4, 1.0), new Quaternion(),
                                                                                        ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.6, -0.4, 1.0), new Quaternion(),
                                                                                         ReferenceFrame.getWorldFrame());

            leftHandTrajectoryBehavior.setInput(leftHandTrajectoryMessage);
            rightHandTrajectoryBehavior.setInput(rightHandTrajectoryMessage);

         }
      };

      BehaviorAction<CuttingWallBehaviorState> waiting = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.WAITING_CONFIRM,
                                                                                                      leftHandTrajectoryBehavior, rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput WAITING " + yoTime.getDoubleValue());

            HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.5, 0.4, 1.0), new Quaternion(),
                                                                                        ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.5, -0.4, 1.0), new Quaternion(),
                                                                                         ReferenceFrame.getWorldFrame());

            leftHandTrajectoryBehavior.setInput(leftHandTrajectoryMessage);
            rightHandTrajectoryBehavior.setInput(rightHandTrajectoryMessage);
         }
      };

      BehaviorAction<CuttingWallBehaviorState> motion = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.MOTION,
                                                                                                      leftHandTrajectoryBehavior, rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput MOTION " + yoTime.getDoubleValue());

            HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.4, 1.0), new Quaternion(),
                                                                                        ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.6, -0.4, 1.0), new Quaternion(),
                                                                                         ReferenceFrame.getWorldFrame());

            leftHandTrajectoryBehavior.setInput(leftHandTrajectoryMessage);
            rightHandTrajectoryBehavior.setInput(rightHandTrajectoryMessage);
         }
      };

      statemachine.addStateWithDoneTransition(planning, CuttingWallBehaviorState.WAITING_CONFIRM);
      statemachine.addStateWithDoneTransition(waiting, CuttingWallBehaviorState.MOTION);

      statemachine.addState(motion);
      statemachine.setStartState(CuttingWallBehaviorState.PLANNING);
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
