package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.SetBooleanParameterPacket;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.CuttingWallBehaviorStateMachine.CuttingWallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PlanWholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WallPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.FunctionTrajectoryTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.variable.YoDouble;

public class CuttingWallBehaviorStateMachine extends StateMachineBehavior<CuttingWallBehaviorState> implements CoactiveDataListenerInterface
{
   private WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;

   private PlanWholeBodyTrajectoryBehavior planWholeBodyTrajectoryBehavior;

   private CommunicationBridge communicationBridge;

   private YoDouble yoTime;

   private FullHumanoidRobotModel fullRobotModel;

   private FramePose centerFramePose;

   private final ConcurrentListeningQueue<WallPosePacket> wallPacketQueue = new ConcurrentListeningQueue<WallPosePacket>(5);

   private final ConcurrentListeningQueue<SetBooleanParameterPacket> confirmQueue = new ConcurrentListeningQueue<SetBooleanParameterPacket>(5);

   private final HumanoidReferenceFrames referenceFrames;

   public enum CuttingWallBehaviorState
   {
      WAITING_INPUT, PLANNING, WAITING_CONFIRM, MOTION, PLAN_FALIED, DONE
   }

   public CuttingWallBehaviorStateMachine(FullHumanoidRobotModelFactory robotModelFactory, CommunicationBridge communicationBridge, YoDouble yoTime,
                                          FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super("cuttingWallBehaviorState", CuttingWallBehaviorState.class, yoTime, communicationBridge);

      this.communicationBridge = communicationBridge;
      this.referenceFrames = referenceFrames;
      communicationBridge.addListeners(this);

      this.wholebodyTrajectoryBehavior = new WholeBodyTrajectoryBehavior(communicationBridge, yoTime);

      this.fullRobotModel = fullRobotModel;

      this.planWholeBodyTrajectoryBehavior = new PlanWholeBodyTrajectoryBehavior("CuttingWallPlanning", robotModelFactory,
                                                                                                       communicationBridge, this.fullRobotModel, yoTime);

      this.yoTime = yoTime;

      attachNetworkListeningQueue(wallPacketQueue, WallPosePacket.class);

      attachNetworkListeningQueue(confirmQueue, SetBooleanParameterPacket.class);

      setupStateMachine();
   }

   public PlanWholeBodyTrajectoryBehavior getPlanConstrainedWholeBodyTrajectoryBehavior()
   {
      return planWholeBodyTrajectoryBehavior;
   }

   @Override
   public void doControl()
   {
      if (isPaused())
         return;

      super.doControl();
   }

   public void setCenterFramePose(FramePose centerFramePose)
   {
      PrintTools.info("" + centerFramePose);
      this.centerFramePose = centerFramePose;
   }

   private Quaternion computeWallOrientation(QuaternionReadOnly quaternionReadOnly)
   {
      RotationMatrix wallRotationMatrix = new RotationMatrix(quaternionReadOnly);

      System.out.println(wallRotationMatrix);

      Vector3D cuttingWallVz = new Vector3D(wallRotationMatrix.getM02(), wallRotationMatrix.getM12(), wallRotationMatrix.getM22());

      double xz = 0;
      double xy = 0;
      if (cuttingWallVz.getZ() == 0.0)
      {
         xz = 1.0;
         xy = 0.0;
      }
      else if (cuttingWallVz.getY() == 0.0)
      {
         xz = 0.0;
         xy = 1.0;
      }
      else
      {
         xz = Math.sqrt(1 / (1 + (cuttingWallVz.getZ() / cuttingWallVz.getY()) * (cuttingWallVz.getZ() / cuttingWallVz.getY())));
         xy = -(cuttingWallVz.getZ()) / (cuttingWallVz.getY()) * xz;
      }

      Vector3D cuttingWallVx = new Vector3D(0.0, xy, xz);

      Vector3D cuttingWallVy = new Vector3D();
      cuttingWallVy.cross(cuttingWallVz, cuttingWallVx);

      RotationMatrix cuttingWallRotationMatrix = new RotationMatrix();
      cuttingWallRotationMatrix.setColumns(cuttingWallVx, cuttingWallVy, cuttingWallVz);
      cuttingWallRotationMatrix.normalize();

      System.out.println(cuttingWallRotationMatrix);

      Quaternion cuttingWallQuaternion = new Quaternion(cuttingWallRotationMatrix);

      System.out.println(cuttingWallQuaternion);

      return cuttingWallQuaternion;
   }

   private void setupStateMachine()
   {
      BehaviorAction<CuttingWallBehaviorState> waiting_input = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.WAITING_INPUT,
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
            return wallPacketQueue.isNewPacketAvailable();
         }
      };

      BehaviorAction<CuttingWallBehaviorState> planning = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.PLANNING,
                                                                                                       planWholeBodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("setBehaviorInput PLANNING " + yoTime.getDoubleValue());

            FramePose centerFramePose = new FramePose();
            WallPosePacket latestPacket = wallPacketQueue.getLatestPacket();
            centerFramePose.setPosition(latestPacket.getCenterPosition());
            centerFramePose.setOrientation(latestPacket.getCenterOrientation());

            centerFramePose.changeFrame(referenceFrames.getMidFootZUpGroundFrame());

            PrintTools.info("received WallPosePacket " + centerFramePose);

            RigidBodyTransform transform = new RigidBodyTransform(centerFramePose.getOrientation(), centerFramePose.getPosition());
            PrintTools.info("transform ");
            System.out.println(transform);

            RigidBodyTransform transform1 = new RigidBodyTransform(new Quaternion(computeWallOrientation(centerFramePose.getOrientation())),
                                                                   new Point3D(centerFramePose.getPosition()));
            PrintTools.info("transform ");
            System.out.println(transform1);

            Point3DReadOnly circleCenter = new Point3D(0.56, 0.0, 1.1);
            Quaternion circleOrientation = new Quaternion();
            circleOrientation.appendPitchRotation(-0.48 * Math.PI);
            Quaternion outputOrientation = new Quaternion();
            outputOrientation.setYawPitchRoll(0.0, -0.4 * Math.PI, 0.0);
            double radius = 0.35;
            double angleStart = 0.0;
            boolean clockwise = true;
            double t0 = 0.0;
            double tf = 20.0;
            FunctionTrajectory circleTrajectory = FunctionTrajectoryTools.circleTrajectory(circleCenter, circleOrientation, outputOrientation, radius, angleStart, clockwise, t0, tf);
            RigidBody leftHand = fullRobotModel.getHand(RobotSide.LEFT);
            WaypointBasedTrajectoryMessage circleTrajectoryMessage = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(leftHand, t0, tf, 0.05, circleTrajectory, ConfigurationSpaceName.YAW);
            RigidBody rightHand = fullRobotModel.getHand(RobotSide.LEFT);
            Pose3D rightHandPose = new Pose3D(-0.2, -0.5, 0.6, -0.4 * Math.PI, 0.0, 0.5 * Math.PI);
            Pose3D[] waypoints = {rightHandPose, rightHandPose};
            double[] waypointTimes = {t0, tf};
            WaypointBasedTrajectoryMessage rightHandTrajectoryMessage = new WaypointBasedTrajectoryMessage(rightHand, waypointTimes, waypoints);

            WholeBodyTrajectoryToolboxConfigurationMessage configurationMessage = new WholeBodyTrajectoryToolboxConfigurationMessage(320, 1000);
            configurationMessage.setInitialConfigration(fullRobotModel);

            WholeBodyTrajectoryToolboxMessage wholeBodyTrajectoryToolboxMessage = new WholeBodyTrajectoryToolboxMessage();
            wholeBodyTrajectoryToolboxMessage.addEndEffectorTrajectories(circleTrajectoryMessage, rightHandTrajectoryMessage);
            wholeBodyTrajectoryToolboxMessage.setConfiguration(configurationMessage);

            planWholeBodyTrajectoryBehavior.setInput(wholeBodyTrajectoryToolboxMessage);
         }
      };

      BehaviorAction<CuttingWallBehaviorState> waiting = new BehaviorAction<CuttingWallBehaviorState>(CuttingWallBehaviorState.WAITING_CONFIRM,
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
               System.out.println("user confirmed "+parameterValue);
               return parameterValue;
            }
            else
               return false;
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

            // TODO
            wholebodyTrajectoryMessage = planWholeBodyTrajectoryBehavior.getWholebodyTrajectoryMessage();

            wholebodyTrajectoryBehavior.setInput(wholebodyTrajectoryMessage);
         }
      };

      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean condition = planWholeBodyTrajectoryBehavior.isDone() && !planWholeBodyTrajectoryBehavior.planSuccess();
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
            boolean condition = planWholeBodyTrajectoryBehavior.isDone() && planWholeBodyTrajectoryBehavior.planSuccess();
            if (condition)
               PrintTools.info("planSuccededCondition condition okay");
            ;
            // pause();
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

      statemachine.addStateWithDoneTransition(waiting_input, CuttingWallBehaviorState.PLANNING);

      statemachine.addStateWithDoneTransition(waiting, CuttingWallBehaviorState.MOTION);
      statemachine.addStateWithDoneTransition(planFailedState, CuttingWallBehaviorState.DONE);
      statemachine.addStateWithDoneTransition(motion, CuttingWallBehaviorState.DONE);

      statemachine.addState(doneState);

      statemachine.setStartState(CuttingWallBehaviorState.WAITING_INPUT);
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