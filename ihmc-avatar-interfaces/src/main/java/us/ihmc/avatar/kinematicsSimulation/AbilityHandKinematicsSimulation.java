package us.ihmc.avatar.kinematicsSimulation;

import static us.ihmc.handsros2.abilityHand.AbilityHand.*;

import java.util.function.Function;

import ihmc_hands_ros2.AbilityHandCommand;
import ihmc_hands_ros2.AbilityHandState;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.handsros2.HandModel;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.abilityHand.AbilityHandModel;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2API;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;

public class AbilityHandKinematicsSimulation
{
   private static final double DEFAULT_GOAL_VELOCITY_DEG_PER_SEC = 180.0;

   private final OneDoFJointBasics[] joints = new OneDoFJointBasics[AbilityHandJointName.values.length];
   private final AbilityHandState stateMessage = new AbilityHandState();
   private final TypedNotification<AbilityHandCommand> commandNotification = new TypedNotification<>();
   private final ROS2Publisher<AbilityHandState> statePublisher;
   private final Throttler stateThrottler = new Throttler().setFrequency(30.0);
   private final double[] goalPositions = new double[ACTUATOR_COUNT];
   private final double[] goalVelocities = new double[ACTUATOR_COUNT];
   private final double[] actuatorPositions = new double[ACTUATOR_COUNT];
   private final boolean enabled;

   private AbilityHandControlMode controlMode = AbilityHandControlMode.POSITION;
   private int gripStage = 0;
   private double lastSimTime = Double.NaN;
   private long lastWallTimeNanos = -1;

   public AbilityHandKinematicsSimulation(RobotSide side, ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel, HandModel handModel)
   {
      this(side, ros2Node, name -> fullRobotModel.getOneDoFJointByName(name), handModel);
   }

   public AbilityHandKinematicsSimulation(RobotSide side, ROS2Node ros2Node, Function<String, OneDoFJointBasics> jointLookup, HandModel handModel)
   {
      // Resolve joint names from the robot hand model so this works when the URDF does not use default Ability Hand names.
      AbilityHandModel abilityHandModel = handModel instanceof AbilityHandModel model ? model : new AbilityHandModel();
      boolean allJointsFound = true;

      for (AbilityHandJointName jointName : AbilityHandJointName.values)
      {
         String jointNameString = abilityHandModel.getAbilityHandJointName(side, jointName);
         OneDoFJointBasics joint = jointLookup.apply(jointNameString);
         joints[jointName.ordinal()] = joint;
         allJointsFound &= joint != null;
      }

      enabled = allJointsFound;

      ros2Node.createSubscriptionSampler(AbilityHandROS2API.COMMAND_TOPICS.get(side), sample -> {
         AbilityHandCommand message = new AbilityHandCommand();
         message.set(sample);
         commandNotification.set(message);
      });
      statePublisher = ros2Node.createPublisher(AbilityHandROS2API.STATE_TOPICS.get(side));

      for (int i = 0; i < ACTUATOR_COUNT; i++)
         goalVelocities[i] = DEFAULT_GOAL_VELOCITY_DEG_PER_SEC;

      setGripGoals(AbilityHandGrip.KEY_CLOSE);
      snapToGoals();
      applyToJoints();
      publishState(true);
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   public void update()
   {
      updateWithDt(wallClockDt());
   }

   /**
    * Advance using simulation time so finger motion tracks sim DT, not wall-clock.
    */
   public void update(double time)
   {
      double dt = 0.0;
      if (!Double.isNaN(lastSimTime))
         dt = time - lastSimTime;
      lastSimTime = time;
      updateWithDt(dt);
   }

   public void writeJointOutput(ControllerOutput output)
   {
      if (!enabled)
         return;

      for (AbilityHandJointName jointName : AbilityHandJointName.values)
      {
         OneDoFJointBasics joint = joints[jointName.ordinal()];
         if (joint == null)
            continue;
         OneDoFJointStateBasics jointOutput = output.getOneDoFJointOutput(joint);
         if (jointOutput == null)
            continue;
         jointOutput.setConfiguration(joint.getQ());
         jointOutput.setVelocity(0.0);
      }
   }

   private void updateWithDt(double dt)
   {
      if (!enabled)
         return;
      if (commandNotification.poll())
      {
         AbilityHandCommand commandMessage = commandNotification.read();
         controlMode = AbilityHandControlMode.fromByte(commandMessage.getControlMode());
         gripStage = 0;
         for (int i1 = 0; i1 < ACTUATOR_COUNT; i1++)
            goalVelocities[i1] = commandMessage.getGoalVelocities()[i1];
         if (controlMode == AbilityHandControlMode.GRIP)
            setGripGoals(AbilityHandGrip.fromByte(commandMessage.getGrip()));
         else
            for (int i1 = 0; i1 < ACTUATOR_COUNT; i1++)
               goalPositions[i1] = commandMessage.getGoalPositions()[i1];
      }

      if (dt > 0.0)
      {
         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            double error = goalPositions[i] - actuatorPositions[i];
            if (Math.abs(error) > 5.0)
            {
               double direction = Math.signum(error);
               actuatorPositions[i] += direction * goalVelocities[i] * dt;
            }
         }
      }

      applyToJoints();
      publishState(false);
   }

   private double wallClockDt()
   {
      long now = System.nanoTime();
      double dt = 0.0;
      if (lastWallTimeNanos > 0)
         dt = (now - lastWallTimeNanos) * 1.0e-9;
      lastWallTimeNanos = now;
      return dt;
   }

   private void applyToJoints()
   {
      if (!enabled)
         return;

      joints[AbilityHandJointName.INDEX_Q1.ordinal()].setQ(Math.toRadians(actuatorPositions[0]));
      joints[AbilityHandJointName.INDEX_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(Math.toRadians(actuatorPositions[0])));
      joints[AbilityHandJointName.MIDDLE_Q1.ordinal()].setQ(Math.toRadians(actuatorPositions[1]));
      joints[AbilityHandJointName.MIDDLE_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(Math.toRadians(actuatorPositions[1])));
      joints[AbilityHandJointName.RING_Q1.ordinal()].setQ(Math.toRadians(actuatorPositions[2]));
      joints[AbilityHandJointName.RING_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(Math.toRadians(actuatorPositions[2])));
      joints[AbilityHandJointName.PINKY_Q1.ordinal()].setQ(Math.toRadians(actuatorPositions[3]));
      joints[AbilityHandJointName.PINKY_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(Math.toRadians(actuatorPositions[3])));
      joints[AbilityHandJointName.THUMB_Q1.ordinal()].setQ(Math.toRadians(actuatorPositions[5]));
      joints[AbilityHandJointName.THUMB_Q2.ordinal()].setQ(Math.toRadians(actuatorPositions[4]));
   }

   private void publishState(boolean force)
   {
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         stateMessage.getActuatorPositions()[i] = (float) actuatorPositions[i];
         stateMessage.getGoalPositions()[i] = (float) goalPositions[i];
         stateMessage.getGoalVelocities()[i] = (float) goalVelocities[i];
      }

      stateMessage.setGripStage(gripStage);

      if (force || stateThrottler.run())
         statePublisher.publish(stateMessage);
   }

   private void snapToGoals()
   {
      for (int i = 0; i < ACTUATOR_COUNT; i++)
         actuatorPositions[i] = goalPositions[i];
   }

   private void setGripGoals(AbilityHandGrip grip)
   {
      gripStage = Math.max(0, grip.getNumberOfStages() - 1);
      for (int i = 0; i < ACTUATOR_COUNT; i++)
         goalPositions[i] = 0.0;
      for (int stage = 0; stage < grip.getNumberOfStages(); stage++)
         for (int i = 0; i < grip.getFingersInStage(stage); i++)
         {
            int finger = grip.getStageFingerIndex(stage, i);
            goalPositions[finger] = grip.getStageFingerPosition(stage, i);
         }
   }
}
