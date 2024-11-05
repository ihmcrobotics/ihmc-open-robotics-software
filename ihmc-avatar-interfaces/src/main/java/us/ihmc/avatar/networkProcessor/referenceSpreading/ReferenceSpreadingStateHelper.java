package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.referenceSpreading.ReferenceSpreadingToolboxController.HandTrajectoryMessagePublisher;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Optional;

public class ReferenceSpreadingStateHelper
{
   private final Double BLEND_INTERVAL = 0.1;
   private final Double PREPARE_DURATION = 1.5;

   public enum States
   {
      BEFORE,
      AFTER,
      WAITING,
      PREPARE,
      NORMAL
   }

   private HandTrajectoryMessagePublisher trajectoryMessagePublisher;
   private final YoRegistry registry;
   private final String demoDirectory;

   ReferenceSpreader referenceSpreader;
   private ReferenceSpreadingTrajectory preImpactReference;
   private ReferenceSpreadingTrajectory blendImpactReference;
   private HashMap<RobotSide, SpatialVectorMessage> handWrenches = new HashMap<>(RobotSide.values().length);
   private us.ihmc.idl.IDLSequence.Float jointVelocities = new us.ihmc.idl.IDLSequence.Float(50, "type_5");
   private us.ihmc.idl.IDLSequence.Float jointTorques = new us.ihmc.idl.IDLSequence.Float(50, "type_5");
   private Double timeInPreTrajectory = 0.0;

   private final CollisionDetection collisionDetection;

   public ReferenceSpreadingStateHelper(String demoDirectory, DRCRobotModel robotModel,  FullHumanoidRobotModel fullRobotModel, HandTrajectoryMessagePublisher trajectoryMessagePublisher, YoRegistry registry)
   {
      this.trajectoryMessagePublisher = trajectoryMessagePublisher;
      this.registry = registry;
      this.demoDirectory = demoDirectory;

      collisionDetection = new CollisionDetection(10, 10, fullRobotModel, registry);
      referenceSpreader = new ReferenceSpreader(demoDirectory, 0.01, BLEND_INTERVAL, robotModel, fullRobotModel, collisionDetection, registry);

      preImpactReference = referenceSpreader.getPreImpactReferenceTrajectory();
      blendImpactReference = null;

      for (RobotSide robotSide : RobotSide.values()) {
         handWrenches.put(robotSide, new SpatialVectorMessage());
      }
   }

   public StateMachine<States, State> setUpStateMachines(DoubleProvider time)
   {
      StateMachineFactory<States, State> factory = new StateMachineFactory<>(States.class);
      factory.setNamePrefix("stateMachine").setRegistry(registry).buildYoClock(time);

      factory.addState(States.BEFORE, new BeforeState());
      factory.addState(States.AFTER, new AfterState());
      factory.addState(States.NORMAL, new NormalState());
      factory.addState(States.WAITING, new WaitingState());
      factory.addState(States.PREPARE, new PrepareState());

      StateTransitionCondition beforeToAfterTransitionCondition = t -> collisionDetection.detectCollision(handWrenches, jointVelocities, t);
      StateTransitionCondition prepareToBeforeTransitionCondition = t -> t > PREPARE_DURATION;

      factory.addTransition(States.PREPARE, States.BEFORE, prepareToBeforeTransitionCondition);
      factory.addTransition(States.BEFORE, States.AFTER, beforeToAfterTransitionCondition);
      factory.addDoneTransition(States.AFTER, States.WAITING);

      return factory.build(States.WAITING);
   }

   public void setTrajectoryMessagePublisher(HandTrajectoryMessagePublisher trajectoryMessagePublisher)
   {
      this.trajectoryMessagePublisher = trajectoryMessagePublisher;
   }

   public void updateHandWrenches(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      handWrenches.get(RobotSide.LEFT).set(capturabilityBasedStatus.getLeftHandWrench());
      handWrenches.get(RobotSide.RIGHT).set(capturabilityBasedStatus.getRightHandWrench());
   }

   public void updateJointVelocities(us.ihmc.idl.IDLSequence.Float jointVelocities)
   {
      this.jointVelocities = jointVelocities;
   }

   public void updateJointTorques(us.ihmc.idl.IDLSequence.Float jointTorques)
   {
      this.jointTorques = jointTorques;
   }

   private class PrepareState implements State
   {

      public PrepareState()
      {
      }

      public void doAction(double timeInState)
      {
      }

      public void onEntry()
      {
         try {
            Optional<Path> mostRecentFile = Files.list(Paths.get(demoDirectory))
                                                 .filter(Files::isRegularFile)
                                                 .max(Comparator.comparingLong(p -> p.toFile().lastModified()));

            mostRecentFile.ifPresent(file -> {
               LogTools.info("Preparing signal: " + file);
               referenceSpreader.setTrajectoryFilePath(file.toString(), true);
               referenceSpreader.spreadTrajectories();
            });

            for (RobotSide robotSide : RobotSide.values())
            {
               HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = referenceSpreader.getOriginalReferenceTrajectory().getFirstHandHybridTrajectoryMessage(robotSide, PREPARE_DURATION);
               trajectoryMessagePublisher.publish(handHybridTrajectoryMessage);
            }

            LogTools.info("Published Message to go back to the initial state!");
         } catch (IOException e) {
            e.printStackTrace();
         }
      }

      public void onExit(double timeInState)
      {
      }
   }

   private class BeforeState implements State
   {

      public BeforeState()
      {
      }

      public void doAction(double timeInState)
      {
      }

      public void onEntry()
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = preImpactReference.getHandHybridTrajectoryMessage(robotSide);
            LogTools.info("Message: " + handHybridTrajectoryMessage);
            trajectoryMessagePublisher.publish(handHybridTrajectoryMessage);
         }

         LogTools.info("Published all messages");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting BeforeState");
         timeInPreTrajectory = timeInState;
      }
   }

   private class AfterState implements State
   {
      public AfterState()
      {
      }

      public void doAction(double timeInState)
      {
//         LogTools.info("AfterState: " + timeInState);
      }

      public void onEntry()
      {
         LogTools.info("Entering AfterState");
         referenceSpreader.blendImpactTrajectory(timeInPreTrajectory + preImpactReference.getStartTimeCSV());
         blendImpactReference = referenceSpreader.getBlendedReferenceTrajectory();
         for (RobotSide robotSide : RobotSide.values())
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = blendImpactReference.getHandHybridTrajectoryMessage(robotSide);
            LogTools.info("Message: " + handHybridTrajectoryMessage);
            trajectoryMessagePublisher.publish(handHybridTrajectoryMessage);
         }

         LogTools.info("Published all messages");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting AfterState");
      }
   }

   private class NormalState implements State
   {
      public NormalState()
      {
      }

      public void doAction(double timeInState)
      {
         //         LogTools.info("AfterState: " + timeInState);
      }

      public void onEntry()
      {

         try {
            Optional<Path> mostRecentFile = Files.list(Paths.get(demoDirectory))
                                                 .filter(Files::isRegularFile)
                                                 .max(Comparator.comparingLong(p -> p.toFile().lastModified()));

            mostRecentFile.ifPresent(file -> {
               LogTools.info("Preparing signal: " + file);
               referenceSpreader.setTrajectoryFilePath(file.toString(), true);
            });
         } catch (IOException e) {
            e.printStackTrace();
         }

         ReferenceSpreadingTrajectory originalTrajectory = referenceSpreader.getOriginalReferenceTrajectory();
         originalTrajectory.setInitialTimeDuration(PREPARE_DURATION);
         for (RobotSide robotSide : RobotSide.values())
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = originalTrajectory.getHandHybridTrajectoryMessage(robotSide);
            LogTools.info("Message: " + handHybridTrajectoryMessage);
            trajectoryMessagePublisher.publish(handHybridTrajectoryMessage);
         }

         LogTools.info("Published all messages");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting AfterState");
      }
   }

   private class WaitingState implements State
   {
      public WaitingState()
      {
      }

      public void doAction(double timeInState)
      {
      }

      public void onEntry()
      {
         LogTools.info("Entering WaitingState");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting WaitingState: " + timeInState);
      }
   }

   public interface RSTimeProvider
   {
      void initialize();

      void update(long currentTime);

      double getTime();

      static RSTimeProvider createTimeProvider()
      {
         return new RSTimeProvider()
         {
            private double time = 0.0;

            @Override
            public void initialize()
            {
               time = 0.0;
            }

            @Override
            public void update(long currentTime)
            {
               time = Conversions.nanosecondsToSeconds(currentTime);

            }

            @Override
            public double getTime()
            {
               return time;
            }
         };
      }
   }
}
