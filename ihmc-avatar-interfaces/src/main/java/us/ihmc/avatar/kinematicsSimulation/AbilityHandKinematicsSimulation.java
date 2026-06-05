package us.ihmc.avatar.kinematicsSimulation;

import ihmc_hands_ros2.AbilityHandCommand;
import ihmc_hands_ros2.AbilityHandState;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.HandModel;
import us.ihmc.handsros2.abilityHand.AbilityHandModel;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2API;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;

import static us.ihmc.handsros2.abilityHand.AbilityHand.*;

public class AbilityHandKinematicsSimulation
{
   private final RevoluteJoint[] joints = new RevoluteJoint[AbilityHandJointName.values.length];
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
   private long lastTime = -1;

   public AbilityHandKinematicsSimulation(RobotSide side, ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel, HandModel handModel)
   {
      AbilityHandModel abilityHandModel = handModel instanceof AbilityHandModel model ? model : new AbilityHandModel();
      boolean allJointsFound = true;

      for (AbilityHandJointName jointName : AbilityHandJointName.values)
      {
         String jointNameString = abilityHandModel.getAbilityHandJointName(side, jointName);
         RevoluteJoint joint = (RevoluteJoint) fullRobotModel.getOneDoFJointByName(jointNameString);
         joints[jointName.ordinal()] = joint;
         allJointsFound &= joint != null;
      }

      enabled = allJointsFound;

      ros2Node.createSubscription(AbilityHandROS2API.COMMAND_TOPICS.get(side), reader -> ROS2Tools.readIfPresent(reader, commandNotification::set));
      statePublisher = ros2Node.createPublisher(AbilityHandROS2API.STATE_TOPICS.get(side));

      setGripGoals(AbilityHandGrip.RELAX);
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   public void update()
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

      long now = System.nanoTime();
      if (lastTime > 0)
      {
         double dt = (now - lastTime) * 1.0e-9;
         for (int i = 0; i < ACTUATOR_COUNT; i++)
            if (Math.abs(goalPositions[i] - actuatorPositions[i]) > 5.0)
            {
               double direction = Math.signum(goalPositions[i] - actuatorPositions[i]);
               double step = direction * goalVelocities[i] * dt;
               actuatorPositions[i] += step;
            }
      }
      lastTime = now;

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

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         stateMessage.getActuatorPositions()[i] = (float) actuatorPositions[i];
         stateMessage.getGoalPositions()[i] = (float) goalPositions[i];
         stateMessage.getGoalVelocities()[i] = (float) goalVelocities[i];
      }

      stateMessage.setGripStage(gripStage);

      if (stateThrottler.run())
         statePublisher.publish(stateMessage);
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
