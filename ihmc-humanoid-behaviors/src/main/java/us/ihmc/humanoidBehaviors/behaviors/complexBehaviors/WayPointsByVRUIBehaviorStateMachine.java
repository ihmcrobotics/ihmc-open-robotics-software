package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.SetBooleanParameterPacket;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WayPointsByVRUIBehaviorStateMachine.WayPointsByVRUIBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PlanConstrainedWholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WayPointsPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.WayPointsTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.variable.YoDouble;

public class WayPointsByVRUIBehaviorStateMachine extends StateMachineBehavior<WayPointsByVRUIBehaviorState> implements CoactiveDataListenerInterface
{
   private WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;

   private PlanConstrainedWholeBodyTrajectoryBehavior planConstrainedWholeBodyTrajectoryBehavior;

   private CommunicationBridge communicationBridge;

   private YoDouble yoTime;

   private FullHumanoidRobotModel fullRobotModel;

   private FramePose centerFramePose;

   private final ConcurrentListeningQueue<WayPointsPacket> wayPointsPacketQueue = new ConcurrentListeningQueue<WayPointsPacket>(5);

   private final ConcurrentListeningQueue<SetBooleanParameterPacket> confirmQueue = new ConcurrentListeningQueue<SetBooleanParameterPacket>(5);

   private final HumanoidReferenceFrames referenceFrames;

   public enum WayPointsByVRUIBehaviorState
   {
      WAITING_INPUT, PLANNING, WAITING_CONFIRM, MOTION, PLAN_FALIED, DONE
   }

   public WayPointsByVRUIBehaviorStateMachine(FullHumanoidRobotModelFactory robotModelFactory, CommunicationBridge communicationBridge, YoDouble yoTime,
                                              FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super("WayPointsByVRUIBehaviorStateMachine", WayPointsByVRUIBehaviorState.class, yoTime, communicationBridge);

      this.communicationBridge = communicationBridge;
      this.referenceFrames = referenceFrames;
      communicationBridge.addListeners(this);

      this.wholebodyTrajectoryBehavior = new WholeBodyTrajectoryBehavior(communicationBridge, yoTime);

      this.fullRobotModel = fullRobotModel;

      this.planConstrainedWholeBodyTrajectoryBehavior = new PlanConstrainedWholeBodyTrajectoryBehavior("CuttingWallPlanning", robotModelFactory,
                                                                                                       communicationBridge, this.fullRobotModel, yoTime);

      this.yoTime = yoTime;

      attachNetworkListeningQueue(wayPointsPacketQueue, WayPointsPacket.class);

      attachNetworkListeningQueue(confirmQueue, SetBooleanParameterPacket.class);

      setupStateMachine();
   }

   public PlanConstrainedWholeBodyTrajectoryBehavior getPlanConstrainedWholeBodyTrajectoryBehavior()
   {
      return planConstrainedWholeBodyTrajectoryBehavior;
   }

   private void setupStateMachine()
   {
      BehaviorAction<WayPointsByVRUIBehaviorState> waiting_input = new BehaviorAction<WayPointsByVRUIBehaviorState>(WayPointsByVRUIBehaviorState.WAITING_INPUT,
                                                                                                                    new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput WAITING_INPUT " + yoTime.getDoubleValue());
         }

         @Override
         public boolean isDone()
         {
            return wayPointsPacketQueue.isNewPacketAvailable();
         }
      };

      BehaviorAction<WayPointsByVRUIBehaviorState> planning = new BehaviorAction<WayPointsByVRUIBehaviorState>(WayPointsByVRUIBehaviorState.PLANNING,
                                                                                                               planConstrainedWholeBodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput PLANNING " + yoTime.getDoubleValue());

            // TODO
            WayPointsPacket latestPacket = wayPointsPacketQueue.getLatestPacket();

            PrintTools.info("numberOfWayPoints " + latestPacket.numberOfWayPoints);

            // TODO
            ConstrainedEndEffectorTrajectory endeffectorTrajectory = new WayPointsTrajectory(latestPacket);
            
            planConstrainedWholeBodyTrajectoryBehavior.setInputs(endeffectorTrajectory, fullRobotModel);

            planConstrainedWholeBodyTrajectoryBehavior.setNumberOfEndEffectorWayPoints(30);
            planConstrainedWholeBodyTrajectoryBehavior.setNumberOfExpanding(1000);
            planConstrainedWholeBodyTrajectoryBehavior.setNumberOfFindInitialGuess(80);

            PlanConstrainedWholeBodyTrajectoryBehavior.constrainedEndEffectorTrajectory = endeffectorTrajectory;
         }
      };

      BehaviorAction<WayPointsByVRUIBehaviorState> waiting = new BehaviorAction<WayPointsByVRUIBehaviorState>(WayPointsByVRUIBehaviorState.WAITING_CONFIRM,
                                                                                                              new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput WAITING_CONFIRM " + yoTime.getDoubleValue());
         }

         @Override
         public boolean isDone()
         {
            if (confirmQueue.isNewPacketAvailable())
            {
               boolean parameterValue = confirmQueue.getLatestPacket().getParameterValue();
               System.out.println("user confirmed " + parameterValue);
               return parameterValue;
            }
            else
               return false;
         }
      };

      BehaviorAction<WayPointsByVRUIBehaviorState> motion = new BehaviorAction<WayPointsByVRUIBehaviorState>(WayPointsByVRUIBehaviorState.MOTION,
                                                                                                             wholebodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput MOTION " + yoTime.getDoubleValue());

            WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

            wholebodyTrajectoryMessage = planConstrainedWholeBodyTrajectoryBehavior.getWholebodyTrajectoryMessage();

            wholebodyTrajectoryBehavior.setInput(wholebodyTrajectoryMessage);
            
            Point3D tempPoint = new Point3D();
            wholebodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT).getTrajectoryPoint(0).getPosition(tempPoint);
            PrintTools.info(""+tempPoint);
            
            tempPoint = new Point3D();
            wholebodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT).getTrajectoryPoint(10).getPosition(tempPoint);
            PrintTools.info(""+tempPoint);
         }
      };

      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean condition = planConstrainedWholeBodyTrajectoryBehavior.isDone() && !planConstrainedWholeBodyTrajectoryBehavior.planSuccess();
            if (condition)
               PrintTools.info("planFailedCondition condition okay");
            ;
            return condition;
         }
      };

      StateTransitionCondition planSuccededCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean condition = planConstrainedWholeBodyTrajectoryBehavior.isDone() && planConstrainedWholeBodyTrajectoryBehavior.planSuccess();
            if (condition)
               PrintTools.info("planSuccededCondition condition okay");
            ;
            // pause();
            return condition;
         }
      };

      BehaviorAction<WayPointsByVRUIBehaviorState> planFailedState = new BehaviorAction<WayPointsByVRUIBehaviorState>(WayPointsByVRUIBehaviorState.PLAN_FALIED,
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

      BehaviorAction<WayPointsByVRUIBehaviorState> doneState = new BehaviorAction<WayPointsByVRUIBehaviorState>(WayPointsByVRUIBehaviorState.DONE,
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
      planning.addStateTransition(WayPointsByVRUIBehaviorState.WAITING_CONFIRM, planSuccededCondition);
      planning.addStateTransition(WayPointsByVRUIBehaviorState.PLAN_FALIED, planFailedCondition);

      statemachine.addStateWithDoneTransition(waiting_input, WayPointsByVRUIBehaviorState.PLANNING);

      statemachine.addStateWithDoneTransition(waiting, WayPointsByVRUIBehaviorState.MOTION);
      statemachine.addStateWithDoneTransition(planFailedState, WayPointsByVRUIBehaviorState.DONE);
      statemachine.addStateWithDoneTransition(motion, WayPointsByVRUIBehaviorState.DONE);

      statemachine.addState(doneState);

      statemachine.setStartState(WayPointsByVRUIBehaviorState.WAITING_INPUT);
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
