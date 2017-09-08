package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.CuttingWallBehaviorStateMachine.CuttingWallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PlanConstrainedWholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior.WalkToLocationStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.DrawingTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.variable.YoDouble;

public class CuttingWallBehaviorStateMachine extends StateMachineBehavior<CuttingWallBehaviorState> implements CoactiveDataListenerInterface
{
   private HandTrajectoryBehavior leftHandTrajectoryBehavior;
   private HandTrajectoryBehavior rightHandTrajectoryBehavior;

   private WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;

   private PlanConstrainedWholeBodyTrajectoryBehavior planConstrainedWholeBodyTrajectoryBehavior;

   private CommunicationBridge communicationBridge;

   private YoDouble yoTime;

   private FullHumanoidRobotModel fullRobotModel;

   public enum CuttingWallBehaviorState
   {
      PLANNING, WAITING_CONFIRM, MOTION, PLAN_FALIED, DONE
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

      this.fullRobotModel = fullRobotModel;

      this.planConstrainedWholeBodyTrajectoryBehavior = new PlanConstrainedWholeBodyTrajectoryBehavior("CuttingWallPlanning", communicationBridge,
                                                                                                       this.fullRobotModel, yoTime);

      this.yoTime = yoTime;

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      BehaviorAction<CuttingWallBehaviorState> planning = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.PLANNING,
                                                                                                       planConstrainedWholeBodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput PLANNING " + yoTime.getDoubleValue());

            ConstrainedEndEffectorTrajectory endeffectorTrajectory = new DrawingTrajectory(20.0);

            planConstrainedWholeBodyTrajectoryBehavior.setInputs(endeffectorTrajectory, fullRobotModel);
            PlanConstrainedWholeBodyTrajectoryBehavior.constrainedEndEffectorTrajectory = endeffectorTrajectory;
         }
      };

      //      BehaviorAction<CuttingWallBehaviorState> planning = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.PLANNING,
      //            rightHandTrajectoryBehavior)
      //      {
      //         @Override
      //         protected void setBehaviorInput()
      //         {
      //            PrintTools.info("setBehaviorInput PLANNING " + yoTime.getDoubleValue());
      //
      //            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.5, -0.4, 1.0), new Quaternion(),
      //                                                                                         ReferenceFrame.getWorldFrame());
      //
      //            rightHandTrajectoryBehavior.setInput(rightHandTrajectoryMessage);
      //         }
      //      };

      BehaviorAction<CuttingWallBehaviorState> waiting = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.WAITING_CONFIRM,
                                                                                                      new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput WAITING " + yoTime.getDoubleValue());

         }
      };

      BehaviorAction<CuttingWallBehaviorState> motion = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.MOTION,
                                                                                                     wholebodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput MOTION " + yoTime.getDoubleValue());

            WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

//            HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.4, 1.0), new Quaternion(),
//                                                                                        ReferenceFrame.getWorldFrame());
//            HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.7, -0.4, 1.0), new Quaternion(),
//                                                                                         ReferenceFrame.getWorldFrame());
//
//            wholebodyTrajectoryMessage.setHandTrajectoryMessage(rightHandTrajectoryMessage);
//            wholebodyTrajectoryMessage.setHandTrajectoryMessage(leftHandTrajectoryMessage);
            
            wholebodyTrajectoryMessage = planConstrainedWholeBodyTrajectoryBehavior.getWholebodyTrajectoryMessage();

            wholebodyTrajectoryBehavior.setInput(wholebodyTrajectoryMessage);
         }
      };

      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean condition = planConstrainedWholeBodyTrajectoryBehavior.isDone() && !planConstrainedWholeBodyTrajectoryBehavior.planSuccess(); 
            if(condition)
               PrintTools.info("planFailedCondition condition okay");;
            return condition;
         }
      };

      StateTransitionCondition planSuccededCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean condition = planConstrainedWholeBodyTrajectoryBehavior.isDone() && planConstrainedWholeBodyTrajectoryBehavior.planSuccess(); 
            if(condition)
               PrintTools.info("planSuccededCondition condition okay");;
            return condition;
         }
      };

      BehaviorAction<CuttingWallBehaviorState> planFailedState = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.PLAN_FALIED,
                                                                                                              new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput PLAN_FALIED " + yoTime.getDoubleValue());
            
            TextToSpeechPacket p1 = new TextToSpeechPacket("Plan Failed");
            sendPacket(p1);
         }
      };

      BehaviorAction<CuttingWallBehaviorState> doneState = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.DONE,
                                                                                                        new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput DONE " + yoTime.getDoubleValue());
            TextToSpeechPacket p1 = new TextToSpeechPacket("Finished Walking");
            sendPacket(p1);
         }
      };

      statemachine.addState(planning);
      planning.addStateTransition(CuttingWallBehaviorState.WAITING_CONFIRM, planSuccededCondition);
      planning.addStateTransition(CuttingWallBehaviorState.PLAN_FALIED, planFailedCondition);

      statemachine.addStateWithDoneTransition(waiting, CuttingWallBehaviorState.MOTION);
      statemachine.addStateWithDoneTransition(planFailedState, CuttingWallBehaviorState.DONE);
      statemachine.addStateWithDoneTransition(motion, CuttingWallBehaviorState.DONE);

      statemachine.addState(doneState);
      statemachine.setStartState(CuttingWallBehaviorState.PLANNING);
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
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
