package us.ihmc.avatar.kinematicsSimulation;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2API;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import static us.ihmc.handsros2.abilityHand.AbilityHand.*;

public class AbilityHandKinematicsSimulation
{
   private final RevoluteJoint[] joints = new RevoluteJoint[AbilityHandJointName.values.length];
   private final AbilityHandState stateMessage = new AbilityHandState();
   private final TypedNotification<AbilityHandCommand> commandNotification = new TypedNotification<>();
   private final ROS2Publisher<AbilityHandState> statePublisher;
   private final Throttler stateThrottler = new Throttler().setFrequency(30.0);
   private final double[] desiredActuatorPositionsDeg = new double[ACTUATOR_COUNT];
   private final double[] desiredActuatorVelocitiesDeg = new double[ACTUATOR_COUNT];
   private final double[] actuatorPositionsDeg = new double[ACTUATOR_COUNT];

   private AbilityHandControlMode controlMode = AbilityHandControlMode.POSITION;
   private int gripStage = 0;

   public AbilityHandKinematicsSimulation(RobotSide side, ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      for (AbilityHandJointName jointName : AbilityHandJointName.values)
         joints[jointName.ordinal()] = (RevoluteJoint) fullRobotModel.getOneDoFJointByName(jointName.getJointName(side));

      ros2Node.createSubscription2(AbilityHandROS2API.COMMAND_TOPICS.get(side), commandNotification::set);
      statePublisher = ros2Node.createPublisher(AbilityHandROS2API.STATE_TOPICS.get(side));

      applyGripTargets(AbilityHandGrip.RELAX);
      applyDesiredPositionsToJoints();
   }

   public void update()
   {
      if (commandNotification.poll())
      {
         AbilityHandCommand commandMessage = commandNotification.read();
         controlMode = AbilityHandControlMode.fromByte(commandMessage.getControlMode());
         gripStage = 0;
         for (int i1 = 0; i1 < ACTUATOR_COUNT; i1++)
            desiredActuatorVelocitiesDeg[i1] = commandMessage.getGoalVelocities()[i1];
         if (controlMode == AbilityHandControlMode.GRIP)
         {
            applyGripTargets(AbilityHandGrip.fromByte(commandMessage.getGrip()));
         }
         else
            for (int i1 = 0; i1 < ACTUATOR_COUNT; i1++)
               desiredActuatorPositionsDeg[i1] = commandMessage.getGoalPositions()[i1];

         applyDesiredPositionsToJoints();
      }

      actuatorPositionsDeg[0] = Math.toDegrees(joints[AbilityHandJointName.INDEX_Q1.ordinal()].getQ());
      actuatorPositionsDeg[1] = Math.toDegrees(joints[AbilityHandJointName.MIDDLE_Q1.ordinal()].getQ());
      actuatorPositionsDeg[2] = Math.toDegrees(joints[AbilityHandJointName.RING_Q1.ordinal()].getQ());
      actuatorPositionsDeg[3] = Math.toDegrees(joints[AbilityHandJointName.PINKY_Q1.ordinal()].getQ());
      actuatorPositionsDeg[5] = Math.toDegrees(joints[AbilityHandJointName.THUMB_Q1.ordinal()].getQ());
      actuatorPositionsDeg[4] = Math.toDegrees(joints[AbilityHandJointName.THUMB_Q2.ordinal()].getQ());

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         stateMessage.getActuatorPositions()[i] = (float) actuatorPositionsDeg[i];
         stateMessage.getGoalPositions()[i] = (float) desiredActuatorPositionsDeg[i];
         stateMessage.getGoalVelocities()[i] = (float) desiredActuatorVelocitiesDeg[i];
      }

      stateMessage.setGripStage(gripStage);

      if (stateThrottler.run())
         statePublisher.publish(stateMessage);
   }

   private void applyGripTargets(AbilityHandGrip grip)
   {
      gripStage = Math.max(0, grip.getNumberOfStages() - 1);
      for (int i = 0; i < ACTUATOR_COUNT; i++)
         desiredActuatorPositionsDeg[i] = 0.0;
      for (int stage = 0; stage < grip.getNumberOfStages(); stage++)
         for (int i = 0; i < grip.getFingersInStage(stage); i++)
         {
            int finger = grip.getStageFingerIndex(stage, i);
            desiredActuatorPositionsDeg[finger] = grip.getStageFingerPosition(stage, i);
         }
   }

   private void applyDesiredPositionsToJoints()
   {
      double indexQ1 = Math.toRadians(desiredActuatorPositionsDeg[0]);
      double middleQ1 = Math.toRadians(desiredActuatorPositionsDeg[1]);
      double ringQ1 = Math.toRadians(desiredActuatorPositionsDeg[2]);
      double pinkyQ1 = Math.toRadians(desiredActuatorPositionsDeg[3]);
      double thumbQ1 = Math.toRadians(desiredActuatorPositionsDeg[5]);
      double thumbQ2 = Math.toRadians(desiredActuatorPositionsDeg[4]);

      joints[AbilityHandJointName.INDEX_Q1.ordinal()].setQ(indexQ1);
      joints[AbilityHandJointName.INDEX_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(indexQ1));
      joints[AbilityHandJointName.MIDDLE_Q1.ordinal()].setQ(middleQ1);
      joints[AbilityHandJointName.MIDDLE_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(middleQ1));
      joints[AbilityHandJointName.RING_Q1.ordinal()].setQ(ringQ1);
      joints[AbilityHandJointName.RING_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(ringQ1));
      joints[AbilityHandJointName.PINKY_Q1.ordinal()].setQ(pinkyQ1);
      joints[AbilityHandJointName.PINKY_Q2.ordinal()].setQ(AbilityHandJointName.getQ2Position(pinkyQ1));
      joints[AbilityHandJointName.THUMB_Q1.ordinal()].setQ(thumbQ1);
      joints[AbilityHandJointName.THUMB_Q2.ordinal()].setQ(thumbQ2);
   }
}
