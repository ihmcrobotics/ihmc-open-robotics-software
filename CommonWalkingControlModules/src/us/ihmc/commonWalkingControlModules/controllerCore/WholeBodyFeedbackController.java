package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.OrientationFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.PointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.time.ExecutionTimer;

public class WholeBodyFeedbackController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final InverseDynamicsCommandList inverseDynamicsOutput = new InverseDynamicsCommandList();
   private final InverseKinematicsCommandList inverseKinematicsOutput = new InverseKinematicsCommandList();
   private final InverseDynamicsCommandList virtualModelControlOutput = new InverseDynamicsCommandList();

   private final List<FeedbackControllerInterface> allControllers = new ArrayList<>();

   private final Map<RigidBody, SpatialFeedbackController> spatialFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBody, PointFeedbackController> pointFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBody, OrientationFeedbackController> orientationFeedbackControllerMap = new HashMap<>();
   private final Map<OneDoFJoint, OneDoFJointFeedbackController> oneDoFJointFeedbackControllerMap = new HashMap<>();

   private final WholeBodyControlCoreToolbox coreToolbox;
   private final FeedbackControllerToolbox feedbackControllerToolbox;

   private final ExecutionTimer feedbackControllerTimer = new ExecutionTimer("wholeBodyFeedbackControllerTimer", 1.0, registry);
   private final ExecutionTimer achievedComputationTimer = new ExecutionTimer("achievedComputationTimer", 1.0, registry);

   public WholeBodyFeedbackController(WholeBodyControlCoreToolbox coreToolbox, FeedbackControlCommandList allPossibleCommands,
                                      YoVariableRegistry parentRegistry)
   {
      this.coreToolbox = coreToolbox;
      this.feedbackControllerToolbox = new FeedbackControllerToolbox(registry);

      if (allPossibleCommands == null)
         return;

      registerControllers(allPossibleCommands);

      parentRegistry.addChild(registry);
   }

   private void registerControllers(FeedbackControlCommandList allPossibleCommands)
   {
      for (int i = 0; i < allPossibleCommands.getNumberOfCommands(); i++)
      {
         FeedbackControlCommand<?> commandExample = allPossibleCommands.getCommand(i);
         ControllerCoreCommandType commandType = commandExample.getCommandType();
         switch (commandType)
         {
         case TASKSPACE:
            registerSpatialControllers((SpatialFeedbackControlCommand) commandExample);
            break;
         case POINT:
            registerPointControllers((PointFeedbackControlCommand) commandExample);
            break;
         case ORIENTATION:
            registerOrientationControllers((OrientationFeedbackControlCommand) commandExample);
            break;
         case JOINTSPACE:
            registerJointspaceControllers((JointspaceFeedbackControlCommand) commandExample);
            break;
         case COMMAND_LIST:
            registerControllers((FeedbackControlCommandList) commandExample);
            break;
         default:
            throw new RuntimeException("The command type: " + commandExample.getCommandType() + " is not handled.");
         }
      }
   }

   private void registerSpatialControllers(SpatialFeedbackControlCommand commandExample)
   {
      RigidBody endEffector = commandExample.getEndEffector();

      if (spatialFeedbackControllerMap.containsKey(endEffector))
         return;

      SpatialFeedbackController controller = new SpatialFeedbackController(endEffector, coreToolbox, feedbackControllerToolbox, registry);
      spatialFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerPointControllers(PointFeedbackControlCommand commandExample)
   {
      RigidBody endEffector = commandExample.getEndEffector();

      if (pointFeedbackControllerMap.containsKey(endEffector))
         return;

      PointFeedbackController controller = new PointFeedbackController(endEffector, coreToolbox, feedbackControllerToolbox, registry);
      pointFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerOrientationControllers(OrientationFeedbackControlCommand commandExample)
   {
      RigidBody endEffector = commandExample.getEndEffector();

      if (orientationFeedbackControllerMap.containsKey(endEffector))
         return;

      OrientationFeedbackController controller = new OrientationFeedbackController(endEffector, coreToolbox, feedbackControllerToolbox, registry);
      orientationFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerJointspaceControllers(JointspaceFeedbackControlCommand commandExample)
   {
      for (int i = 0; i < commandExample.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = commandExample.getJoint(i);
         if (oneDoFJointFeedbackControllerMap.containsKey(joint))
            continue;

         double controlDT = coreToolbox.getControlDT();
         OneDoFJointFeedbackController controller = new OneDoFJointFeedbackController(joint, controlDT, registry);
         oneDoFJointFeedbackControllerMap.put(joint, controller);
         allControllers.add(controller);
      }
   }

   public void initialize()
   {
      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         controller.initialize();
      }
   }

   public void reset()
   {
      // FIXME This results into discontinuities in the remote visualizer, need to change it such as the YoVariables change only once per tick.
//      feedbackControllerToolbox.clearData();

      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         controller.setEnabled(false);
      }
   }

   public void computeInverseDynamics()
   {
      feedbackControllerTimer.startMeasurement();
      inverseDynamicsOutput.clear();

      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         if (controller.isEnabled())
         {
            controller.computeInverseDynamics();
            inverseDynamicsOutput.addCommand(controller.getInverseDynamicsOutput());
         }
      }
      feedbackControllerTimer.stopMeasurement();
   }

   public void computeInverseKinematics()
   {
      feedbackControllerTimer.startMeasurement();
      inverseKinematicsOutput.clear();

      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         if (controller.isEnabled())
         {
            controller.computeInverseKinematics();
            inverseKinematicsOutput.addCommand(controller.getInverseKinematicsOutput());
         }
      }
      feedbackControllerTimer.stopMeasurement();
   }

   public void computeVirtualModelControl()
   {
      feedbackControllerTimer.startMeasurement();
      virtualModelControlOutput.clear();

      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         if (controller.isEnabled())
         {
            controller.computeVirtualModelControl();
            virtualModelControlOutput.addCommand(controller.getVirtualModelControlOutput());
         }
      }
      feedbackControllerTimer.stopMeasurement();
   }

   public void computeAchievedAccelerations()
   {
      achievedComputationTimer.startMeasurement();
      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         if (controller.isEnabled())
         {
            controller.computeAchievedAcceleration();
         }
      }
      achievedComputationTimer.stopMeasurement();
   }

   public void submitFeedbackControlCommandList(FeedbackControlCommandList feedbackControlCommandList)
   {
      while (feedbackControlCommandList.getNumberOfCommands() > 0)
      {
         FeedbackControlCommand<?> feedbackControlCommand = feedbackControlCommandList.pollCommand();
         ControllerCoreCommandType commandType = feedbackControlCommand.getCommandType();
         switch (commandType)
         {
         case TASKSPACE:
            submitSpatialFeedbackControlCommand((SpatialFeedbackControlCommand) feedbackControlCommand);
            break;
         case POINT:
            submitPointFeedbackControlCommand((PointFeedbackControlCommand) feedbackControlCommand);
            break;
         case ORIENTATION:
            submitOrientationFeedbackControlCommand((OrientationFeedbackControlCommand) feedbackControlCommand);
            break;
         case JOINTSPACE:
            submitJointspaceFeedbackControlCommand((JointspaceFeedbackControlCommand) feedbackControlCommand);
            break;
         case COMMAND_LIST:
            submitFeedbackControlCommandList((FeedbackControlCommandList) feedbackControlCommand);
            break;
         default:
            throw new RuntimeException("The command type: " + commandType + " is not handled.");
         }
      }
   }

   private void submitSpatialFeedbackControlCommand(SpatialFeedbackControlCommand feedbackControlCommand)
   {
      RigidBody endEffector = feedbackControlCommand.getEndEffector();
      SpatialFeedbackController controller = spatialFeedbackControllerMap.get(endEffector);
      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitPointFeedbackControlCommand(PointFeedbackControlCommand feedbackControlCommand)
   {
      RigidBody endEffector = feedbackControlCommand.getEndEffector();
      PointFeedbackController controller = pointFeedbackControllerMap.get(endEffector);
      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitOrientationFeedbackControlCommand(OrientationFeedbackControlCommand feedbackControlCommand)
   {
      RigidBody endEffector = feedbackControlCommand.getEndEffector();
      OrientationFeedbackController controller = orientationFeedbackControllerMap.get(endEffector);
      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitJointspaceFeedbackControlCommand(JointspaceFeedbackControlCommand feedbackControlCommand)
   {
      for (int i = 0; i < feedbackControlCommand.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = feedbackControlCommand.getJoint(i);
         double desiredPosition = feedbackControlCommand.getDesiredPosition(i);
         double desiredVelocity = feedbackControlCommand.getDesiredVelocity(i);
         double feedForwardAcceleration = feedbackControlCommand.getFeedForwardAcceleration(i);

         OneDoFJointFeedbackController controller = oneDoFJointFeedbackControllerMap.get(joint);
         controller.setGains(feedbackControlCommand.getGains(i));
         controller.setDesireds(desiredPosition, desiredVelocity, feedForwardAcceleration);
         controller.setWeightForSolver(feedbackControlCommand.getWeightForSolver(i));
         controller.setEnabled(true);
      }
   }

   public InverseDynamicsCommandList getInverseDynamicsOutput()
   {
      return inverseDynamicsOutput;
   }

   public InverseKinematicsCommandList getInverseKinematicsOutput()
   {
      return inverseKinematicsOutput;
   }

   public InverseDynamicsCommandList getVirtualModelControlOutput()
   {
      return virtualModelControlOutput;
   }
}
