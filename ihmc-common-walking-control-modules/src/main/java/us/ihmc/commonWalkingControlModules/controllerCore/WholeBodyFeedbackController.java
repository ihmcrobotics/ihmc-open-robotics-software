package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.CenterOfMassFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.OrientationFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.PointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WholeBodyFeedbackController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final InverseDynamicsCommandList inverseDynamicsOutput = new InverseDynamicsCommandList();
   private final InverseKinematicsCommandList inverseKinematicsOutput = new InverseKinematicsCommandList();
   private final VirtualModelControlCommandList virtualModelControlOutput = new VirtualModelControlCommandList();

   private final List<FeedbackControllerInterface> allControllers = new ArrayList<>();

   private CenterOfMassFeedbackController centerOfMassFeedbackController;
   private final Map<RigidBodyBasics, SpatialFeedbackController> spatialFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBodyBasics, PointFeedbackController> pointFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBodyBasics, OrientationFeedbackController> orientationFeedbackControllerMap = new HashMap<>();
   private final Map<OneDoFJointBasics, OneDoFJointFeedbackController> oneDoFJointFeedbackControllerMap = new HashMap<>();

   private final WholeBodyControlCoreToolbox coreToolbox;
   private final FeedbackControllerToolbox feedbackControllerToolbox;

   private final ExecutionTimer feedbackControllerTimer = new ExecutionTimer("wholeBodyFeedbackControllerTimer", 1.0, registry);
   private final ExecutionTimer achievedComputationTimer = new ExecutionTimer("achievedComputationTimer", 1.0, registry);

   public WholeBodyFeedbackController(WholeBodyControlCoreToolbox coreToolbox, FeedbackControlCommandList allPossibleCommands,
                                      YoVariableRegistry parentRegistry)
   {
      this.coreToolbox = coreToolbox;
      this.feedbackControllerToolbox = new FeedbackControllerToolbox(coreToolbox.getFeedbackControllerSettings(), registry);

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
            registerOneDoFJointControllers((OneDoFJointFeedbackControlCommand) commandExample);
            break;
         case MOMENTUM:
            registerCenterOfMassController((CenterOfMassFeedbackControlCommand) commandExample);
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
      RigidBodyBasics endEffector = commandExample.getEndEffector();

      if (spatialFeedbackControllerMap.containsKey(endEffector))
         return;

      SpatialFeedbackController controller = new SpatialFeedbackController(endEffector, coreToolbox, feedbackControllerToolbox, registry);
      spatialFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerPointControllers(PointFeedbackControlCommand commandExample)
   {
      RigidBodyBasics endEffector = commandExample.getEndEffector();

      if (pointFeedbackControllerMap.containsKey(endEffector))
         return;

      PointFeedbackController controller = new PointFeedbackController(endEffector, coreToolbox, feedbackControllerToolbox, registry);
      pointFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerOrientationControllers(OrientationFeedbackControlCommand commandExample)
   {
      RigidBodyBasics endEffector = commandExample.getEndEffector();

      if (orientationFeedbackControllerMap.containsKey(endEffector))
         return;

      OrientationFeedbackController controller = new OrientationFeedbackController(endEffector, coreToolbox, feedbackControllerToolbox, registry);
      orientationFeedbackControllerMap.put(endEffector, controller);
      allControllers.add(controller);
   }

   private void registerOneDoFJointControllers(OneDoFJointFeedbackControlCommand commandExample)
   {
      OneDoFJointBasics joint = commandExample.getJoint();
      if (oneDoFJointFeedbackControllerMap.containsKey(joint))
         return;

      OneDoFJointFeedbackController controller = new OneDoFJointFeedbackController(joint, coreToolbox, feedbackControllerToolbox, registry);
      oneDoFJointFeedbackControllerMap.put(joint, controller);
      allControllers.add(controller);
   }

   private void registerCenterOfMassController(CenterOfMassFeedbackControlCommand commandExample)
   {
      if (centerOfMassFeedbackController != null)
         return;
      centerOfMassFeedbackController = new CenterOfMassFeedbackController(coreToolbox, feedbackControllerToolbox, registry);
      allControllers.add(centerOfMassFeedbackController);
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
         else
         {
            controller.initialize();
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
         else
         {
            controller.initialize();
         }
      }
      feedbackControllerToolbox.clearUnusedData();
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
         else
         {
            controller.initialize();
         }
      }
      feedbackControllerToolbox.clearUnusedData();
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

   public void submitFeedbackControlCommandList(WholeBodyControllerCoreMode activeControlMode, FeedbackControlCommandList feedbackControlCommandList)
   {
      for (int commandIndex = 0; commandIndex < feedbackControlCommandList.getNumberOfCommands(); commandIndex++)
      {
         FeedbackControlCommand<?> feedbackControlCommand = feedbackControlCommandList.getCommand(commandIndex);
         ControllerCoreCommandType commandType = feedbackControlCommand.getCommandType();
         switch (commandType)
         {
         case TASKSPACE:
            submitSpatialFeedbackControlCommand(activeControlMode, (SpatialFeedbackControlCommand) feedbackControlCommand);
            break;
         case POINT:
            submitPointFeedbackControlCommand(activeControlMode, (PointFeedbackControlCommand) feedbackControlCommand);
            break;
         case ORIENTATION:
            submitOrientationFeedbackControlCommand(activeControlMode, (OrientationFeedbackControlCommand) feedbackControlCommand);
            break;
         case JOINTSPACE:
            submitOneDoFJointFeedbackControlCommand(activeControlMode, (OneDoFJointFeedbackControlCommand) feedbackControlCommand);
            break;
         case MOMENTUM:
            submitCenterOfMassFeedbackControlCommand(activeControlMode, (CenterOfMassFeedbackControlCommand) feedbackControlCommand);
            break;
         case COMMAND_LIST:
            submitFeedbackControlCommandList(activeControlMode, (FeedbackControlCommandList) feedbackControlCommand);
            break;
         default:
            throw new RuntimeException("The command type: " + commandType + " is not handled.");
         }
      }
   }

   private void submitSpatialFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, SpatialFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      RigidBodyBasics endEffector = feedbackControlCommand.getEndEffector();
      SpatialFeedbackController controller = spatialFeedbackControllerMap.get(endEffector);
      if (controller.isEnabled())
         throw new RuntimeException("Cannot submit more than one feedback control command to the same controller. Controller end-effector: " + endEffector);
      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitPointFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, PointFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      RigidBodyBasics endEffector = feedbackControlCommand.getEndEffector();
      PointFeedbackController controller = pointFeedbackControllerMap.get(endEffector);
      if (controller.isEnabled())
         throw new RuntimeException("Cannot submit more than one feedback control command to the same controller. Controller end-effector: " + endEffector);
      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitOrientationFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, OrientationFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      RigidBodyBasics endEffector = feedbackControlCommand.getEndEffector();
      OrientationFeedbackController controller = orientationFeedbackControllerMap.get(endEffector);
      if (controller.isEnabled())
         throw new RuntimeException("Cannot submit more than one feedback control command to the same controller. Controller end-effector: " + endEffector);
      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitOneDoFJointFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, OneDoFJointFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      OneDoFJointBasics joint = feedbackControlCommand.getJoint();
      OneDoFJointFeedbackController controller = oneDoFJointFeedbackControllerMap.get(joint);
      if (controller.isEnabled())
         throw new RuntimeException("Cannot submit more than one feedback control command to the same controller. Controller joint: " + joint.getName());
      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitCenterOfMassFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode,
                                                         CenterOfMassFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      centerOfMassFeedbackController.submitFeedbackControlCommand(feedbackControlCommand);
      centerOfMassFeedbackController.setEnabled(true);
   }

   private static void checkRequestedControlMode(WholeBodyControllerCoreMode activeControlMode, WholeBodyControllerCoreMode requestedControlMode)
   {
      if (activeControlMode != requestedControlMode)
         throw new IllegalArgumentException("Incompatible feedback control command: command requires: " + requestedControlMode + ", current mode: "
               + activeControlMode);
   }

   public InverseDynamicsCommandList getInverseDynamicsOutput()
   {
      return inverseDynamicsOutput;
   }

   public InverseKinematicsCommandList getInverseKinematicsOutput()
   {
      return inverseKinematicsOutput;
   }

   public VirtualModelControlCommandList getVirtualModelControlOutput()
   {
      return virtualModelControlOutput;
   }

   public FeedbackControllerDataReadOnly getWholeBodyFeedbackControllerDataHolder()
   {
      return feedbackControllerToolbox;
   }
}
