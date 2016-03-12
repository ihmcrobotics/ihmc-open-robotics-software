package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand.FeedbackControlCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.OrientationFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.PointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.robotics.controllers.PDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class WholeBodyFeedbackController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final InverseDynamicsCommandList output = new InverseDynamicsCommandList();

   private final List<FeedbackControllerInterface> allControllers = new ArrayList<>();

   private final Map<RigidBody, SpatialFeedbackController> spatialFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBody, PointFeedbackController> pointFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBody, OrientationFeedbackController> orientationFeedbackControllerMap = new HashMap<>();
   private final Map<OneDoFJoint, OneDoFJointFeedbackController> oneDoFJointFeedbackControllerMap = new HashMap<>();

   private final WholeBodyControlCoreToolbox toolbox;

   public WholeBodyFeedbackController(WholeBodyControlCoreToolbox toolbox, FeedbackControlCommandList allPossibleCommands, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;

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
         FeedbackControlCommandType commandType = commandExample.getCommandType();
         switch (commandType)
         {
         case SPATIAL_CONTROL:
            registerSpatialControllers((SpatialFeedbackControlCommand) commandExample);
            break;
         case POINT_CONTROL:
            registerPointControllers((PointFeedbackControlCommand) commandExample);
            break;
         case ORIENTATION_CONTROL:
            registerOrientationControllers((OrientationFeedbackControlCommand) commandExample);
            break;
         case JOINTSPACE_CONTROL:
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

      SpatialFeedbackController controller = new SpatialFeedbackController(endEffector, toolbox, registry);
      spatialFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerPointControllers(PointFeedbackControlCommand commandExample)
   {
      RigidBody endEffector = commandExample.getEndEffector();

      if (pointFeedbackControllerMap.containsKey(endEffector))
         return;

      PointFeedbackController controller = new PointFeedbackController(endEffector, toolbox, registry);
      pointFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerOrientationControllers(OrientationFeedbackControlCommand commandExample)
   {
      RigidBody endEffector = commandExample.getEndEffector();

      if (orientationFeedbackControllerMap.containsKey(endEffector))
         return;

      OrientationFeedbackController controller = new OrientationFeedbackController(endEffector, toolbox, registry);
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

         OneDoFJointFeedbackController controller = new OneDoFJointFeedbackController(joint, toolbox.getControlDT(), registry);
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
      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         controller.setEnabled(false);
      }
   }

   public void compute()
   {
      output.clear();

      for (int i = 0; i < allControllers.size(); i++)
      {
         FeedbackControllerInterface controller = allControllers.get(i);
         if (controller.isEnabled())
         {
            controller.compute();
            output.addCommand(controller.getOutput());
         }
      }
   }

   public void submitFeedbackControlCommandList(FeedbackControlCommandList feedbackControlCommandList)
   {
      for (int i = 0; i < feedbackControlCommandList.getNumberOfCommands(); i++)
      {
         FeedbackControlCommand<?> feedbackControlCommand = feedbackControlCommandList.getCommand(i);
         FeedbackControlCommandType commandType = feedbackControlCommand.getCommandType();
         switch (commandType)
         {
         case SPATIAL_CONTROL:
            submitSpatialFeedbackControlCommand((SpatialFeedbackControlCommand) feedbackControlCommand);
            break;
         case POINT_CONTROL:
            submitPointFeedbackControlCommand((PointFeedbackControlCommand) feedbackControlCommand);
            break;
         case ORIENTATION_CONTROL:
            submitOrientationFeedbackControlCommand((OrientationFeedbackControlCommand) feedbackControlCommand);
            break;
         case JOINTSPACE_CONTROL:
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
      PDGainsInterface gains = feedbackControlCommand.getGains();

      for (int i = 0; i < feedbackControlCommand.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = feedbackControlCommand.getJoint(i);
         double desiredPosition = feedbackControlCommand.getDesiredPosition(i);
         double desiredVelocity = feedbackControlCommand.getDesiredVelocity(i);
         double feedForwardAcceleration = feedbackControlCommand.getFeedForwardAcceleration(i);

         OneDoFJointFeedbackController controller = oneDoFJointFeedbackControllerMap.get(joint);
         controller.setGains(gains);
         controller.setDesireds(desiredPosition, desiredVelocity, feedForwardAcceleration);
         controller.setWeightForSolver(feedbackControlCommand.getWeightForSolver());
         controller.setEnabled(true);
      }
   }

   public InverseDynamicsCommandList getOutput()
   {
      return output;
   }
}
