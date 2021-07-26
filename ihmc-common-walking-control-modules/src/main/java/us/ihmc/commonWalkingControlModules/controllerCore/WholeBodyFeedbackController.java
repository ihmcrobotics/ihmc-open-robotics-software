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
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.CenterOfMassFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.OrientationFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.PointFeedbackController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class WholeBodyFeedbackController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final InverseDynamicsCommandList inverseDynamicsOutput = new InverseDynamicsCommandList();
   private final InverseKinematicsCommandList inverseKinematicsOutput = new InverseKinematicsCommandList();
   private final VirtualModelControlCommandList virtualModelControlOutput = new VirtualModelControlCommandList();

   private final List<FeedbackControllerInterface> allControllers = new ArrayList<>();

   private final YoBoolean dynamicControllerConstructionEnabled = new YoBoolean("dynamicControllerConstructionEnabled", registry);

   private CenterOfMassFeedbackController centerOfMassFeedbackController;
   private final Map<RigidBodyBasics, List<SpatialFeedbackController>> spatialFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBodyBasics, List<PointFeedbackController>> pointFeedbackControllerMap = new HashMap<>();
   private final Map<RigidBodyBasics, List<OrientationFeedbackController>> orientationFeedbackControllerMap = new HashMap<>();
   private final Map<OneDoFJointBasics, OneDoFJointFeedbackController> oneDoFJointFeedbackControllerMap = new HashMap<>();

   private final WholeBodyControlCoreToolbox coreToolbox;
   private final FeedbackControllerToolbox feedbackControllerToolbox;
   private final FeedbackControllerFactory feedbackControllerFactory;

   private final ExecutionTimer feedbackControllerTimer = new ExecutionTimer("wholeBodyFeedbackControllerTimer", 1.0, registry);
   private final ExecutionTimer achievedComputationTimer = new ExecutionTimer("achievedComputationTimer", 1.0, registry);

   public WholeBodyFeedbackController(WholeBodyControlCoreToolbox coreToolbox, FeedbackControllerTemplate feedbackControllerTemplate, YoRegistry parentRegistry)
   {
      this.coreToolbox = coreToolbox;
      feedbackControllerToolbox = new FeedbackControllerToolbox(coreToolbox.getFeedbackControllerSettings(), registry);

      if (feedbackControllerTemplate != null)
      {
         feedbackControllerFactory = feedbackControllerTemplate.getFeedbackControllerFactory();
         feedbackControllerFactory.configure(coreToolbox, feedbackControllerToolbox, registry);
         registerControllers(feedbackControllerTemplate);
         parentRegistry.addChild(registry);
      }
      else
      {
         feedbackControllerFactory = null;
      }
   }

   private void registerControllers(FeedbackControllerTemplate template)
   {
      dynamicControllerConstructionEnabled.set(template.isDynamicControllerConstructionAllowed());
      template.getSpatialFeedbackControllerTemplate().forEach(this::registerSpatialControllers);
      template.getPointFeedbackControllerTemplate().forEach(this::registerPointControllers);
      template.getOrientationFeedbackControllerTemplate().forEach(this::registerOrientationControllers);
      template.getOneDoFJointFeedbackControllerTemplate().forEach(this::registerOneDoFJointControllers);
      if (template.isCenterOfMassFeedbackControllerEnabled())
         registerCenterOfMassController();
   }

   private void registerSpatialControllers(RigidBodyBasics endEffector, int numberOfControllers)
   {
      List<SpatialFeedbackController> endEffectorControllers = new ArrayList<>();
      spatialFeedbackControllerMap.put(endEffector, endEffectorControllers);

      for (int controllerIndex = 0; controllerIndex < numberOfControllers; controllerIndex++)
      {
         SpatialFeedbackController controller = feedbackControllerFactory.buildSpatialFeedbackController(endEffector, controllerIndex);
         endEffectorControllers.add(controller);
         allControllers.add(controller);
      }
   }

   private void registerPointControllers(RigidBodyBasics endEffector, int numberOfControllers)
   {
      List<PointFeedbackController> endEffectorControllers = new ArrayList<>();
      pointFeedbackControllerMap.put(endEffector, endEffectorControllers);

      for (int controllerIndex = 0; controllerIndex < numberOfControllers; controllerIndex++)
      {
         PointFeedbackController controller = feedbackControllerFactory.buildPointFeedbackController(endEffector, controllerIndex);
         endEffectorControllers.add(controller);
         allControllers.add(controller);
      }
   }

   private void registerOrientationControllers(RigidBodyBasics endEffector, int numberOfControllers)
   {
      List<OrientationFeedbackController> endEffectorControllers = new ArrayList<>();
      orientationFeedbackControllerMap.put(endEffector, endEffectorControllers);

      for (int controllerIndex = 0; controllerIndex < numberOfControllers; controllerIndex++)
      {
         OrientationFeedbackController controller = feedbackControllerFactory.buildOrientationFeedbackController(endEffector, controllerIndex);
         endEffectorControllers.add(controller);
         allControllers.add(controller);
      }
   }

   private void registerOneDoFJointControllers(OneDoFJointBasics joint)
   {
      OneDoFJointFeedbackController controller = feedbackControllerFactory.buildOneDoFJointFeedbackController(joint);
      oneDoFJointFeedbackControllerMap.put(joint, controller);
      allControllers.add(controller);
   }

   private void registerCenterOfMassController()
   {
      centerOfMassFeedbackController = feedbackControllerFactory.buildCenterOfMassFeedbackController();
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
      feedbackControllerToolbox.registerFeedbackControllerOutput(inverseDynamicsOutput);
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
      feedbackControllerToolbox.registerFeedbackControllerOutput(inverseKinematicsOutput);
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
      feedbackControllerToolbox.registerFeedbackControllerOutput(virtualModelControlOutput);
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
               throw new FeedbackControllerException("The command type: " + commandType + " is not handled.");
         }
      }
   }

   private void submitSpatialFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, SpatialFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      RigidBodyBasics endEffector = feedbackControlCommand.getEndEffector();
      List<SpatialFeedbackController> endEffectorControllers = spatialFeedbackControllerMap.get(endEffector);

      if (endEffectorControllers == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
         {
            throw new FeedbackControllerException("No feedback controller was created for the end-effector " + endEffector.getName() + ".");
         }
         else
         {
            registerSpatialControllers(endEffector, 1);
            endEffectorControllers = spatialFeedbackControllerMap.get(endEffector);
         }
      }

      SpatialFeedbackController nextControllerAvailable = null;

      for (int controllerIndex = 0; controllerIndex < endEffectorControllers.size(); controllerIndex++)
      {
         SpatialFeedbackController controller = endEffectorControllers.get(controllerIndex);
         if (controller.isEnabled())
            continue;

         if (isOrientationControllerCreatedAndEnabled(endEffector, controllerIndex))
            continue; // The orientation controller and the spatial controller share data => cannot be used at the same time.
         if (isPointControllerCreatedAndEnabled(endEffector, controllerIndex))
            continue; // The point controller and the spatial controller share data => cannot be used at the same time.

         nextControllerAvailable = controller;
      }

      if (nextControllerAvailable == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
         {
            throw new FeedbackControllerException("Could not find a controller available for the end-effector: " + endEffector.getName()
                  + ", number of controllers: " + endEffectorControllers.size());
         }
         else
         {
            while (nextControllerAvailable == null)
            {
               int controllerIndex = endEffectorControllers.size();
               SpatialFeedbackController controller = new SpatialFeedbackController(endEffector,
                                                                                    controllerIndex,
                                                                                    coreToolbox,
                                                                                    feedbackControllerToolbox,
                                                                                    registry);
               endEffectorControllers.add(controller);
               allControllers.add(controller);

               if (isOrientationControllerCreatedAndEnabled(endEffector, controllerIndex))
                  continue; // The orientation controller and the spatial controller share data => cannot be used at the same time.
               if (isPointControllerCreatedAndEnabled(endEffector, controllerIndex))
                  continue; // The point controller and the spatial controller share data => cannot be used at the same time.

               nextControllerAvailable = controller;
            }
         }
      }

      nextControllerAvailable.submitFeedbackControlCommand(feedbackControlCommand);
      nextControllerAvailable.setEnabled(true);
   }

   private void submitPointFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, PointFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      RigidBodyBasics endEffector = feedbackControlCommand.getEndEffector();
      List<PointFeedbackController> endEffectorControllers = pointFeedbackControllerMap.get(endEffector);

      if (endEffectorControllers == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
         {
            throw new FeedbackControllerException("No feedback controller was created for the end-effector " + endEffector.getName() + ".");
         }
         else
         {
            registerPointControllers(endEffector, 1);
            endEffectorControllers = pointFeedbackControllerMap.get(endEffector);
         }
      }

      PointFeedbackController nextControllerAvailable = null;

      for (int controllerIndex = 0; controllerIndex < endEffectorControllers.size(); controllerIndex++)
      {
         PointFeedbackController controller = endEffectorControllers.get(controllerIndex);
         if (controller.isEnabled())
            continue;

         if (isSpatialControllerCreatedAndEnabled(endEffector, controllerIndex))
            continue; // The point controller and the spatial controller share data => cannot be used at the same time.

         nextControllerAvailable = controller;
      }

      if (nextControllerAvailable == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
         {
            throw new FeedbackControllerException("Could not find a controller available for the end-effector: " + endEffector.getName()
                  + ", number of controllers: " + endEffectorControllers.size());
         }
         else
         {
            while (nextControllerAvailable == null)
            {
               int controllerIndex = endEffectorControllers.size();
               PointFeedbackController controller = new PointFeedbackController(endEffector, controllerIndex, coreToolbox, feedbackControllerToolbox, registry);
               endEffectorControllers.add(controller);
               allControllers.add(controller);

               if (isSpatialControllerCreatedAndEnabled(endEffector, controllerIndex))
                  continue; // The point controller and the spatial controller share data => cannot be used at the same time.

               nextControllerAvailable = controller;
            }
         }
      }

      nextControllerAvailable.submitFeedbackControlCommand(feedbackControlCommand);
      nextControllerAvailable.setEnabled(true);
   }

   private void submitOrientationFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, OrientationFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      RigidBodyBasics endEffector = feedbackControlCommand.getEndEffector();
      List<OrientationFeedbackController> endEffectorControllers = orientationFeedbackControllerMap.get(endEffector);

      if (endEffectorControllers == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
         {
            throw new FeedbackControllerException("No feedback controller was created for the end-effector " + endEffector.getName() + ".");
         }
         else
         {
            registerOrientationControllers(endEffector, 1);
            endEffectorControllers = orientationFeedbackControllerMap.get(endEffector);
         }
      }

      OrientationFeedbackController nextControllerAvailable = null;

      for (int controllerIndex = 0; controllerIndex < endEffectorControllers.size(); controllerIndex++)
      {
         OrientationFeedbackController controller = endEffectorControllers.get(controllerIndex);
         if (controller.isEnabled())
            continue;

         if (isSpatialControllerCreatedAndEnabled(endEffector, controllerIndex))
            continue; // The orientation controller and the spatial controller share data => cannot be used at the same time.

         nextControllerAvailable = controller;
      }

      if (nextControllerAvailable == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
         {
            throw new FeedbackControllerException("Could not find a controller available for the end-effector: " + endEffector.getName()
                  + ", number of controllers: " + endEffectorControllers.size());
         }
         else
         {
            while (nextControllerAvailable == null)
            {
               int controllerIndex = endEffectorControllers.size();
               OrientationFeedbackController controller = new OrientationFeedbackController(endEffector,
                                                                                            controllerIndex,
                                                                                            coreToolbox,
                                                                                            feedbackControllerToolbox,
                                                                                            registry);
               endEffectorControllers.add(controller);
               allControllers.add(controller);

               if (isSpatialControllerCreatedAndEnabled(endEffector, controllerIndex))
                  continue; // The orientation controller and the spatial controller share data => cannot be used at the same time.

               nextControllerAvailable = controller;
            }
         }
      }

      nextControllerAvailable.submitFeedbackControlCommand(feedbackControlCommand);
      nextControllerAvailable.setEnabled(true);
   }

   private boolean isSpatialControllerCreatedAndEnabled(RigidBodyBasics endEffector, int controllerIndex)
   {
      List<SpatialFeedbackController> endEffectorControllers = spatialFeedbackControllerMap.get(endEffector);
      if (endEffectorControllers == null)
         return false;
      if (endEffectorControllers.size() <= controllerIndex)
         return false;
      return endEffectorControllers.get(controllerIndex).isEnabled();
   }

   private boolean isPointControllerCreatedAndEnabled(RigidBodyBasics endEffector, int controllerIndex)
   {
      List<PointFeedbackController> endEffectorControllers = pointFeedbackControllerMap.get(endEffector);
      if (endEffectorControllers == null)
         return false;
      if (endEffectorControllers.size() <= controllerIndex)
         return false;
      return endEffectorControllers.get(controllerIndex).isEnabled();
   }

   private boolean isOrientationControllerCreatedAndEnabled(RigidBodyBasics endEffector, int controllerIndex)
   {
      List<OrientationFeedbackController> endEffectorControllers = orientationFeedbackControllerMap.get(endEffector);
      if (endEffectorControllers == null)
         return false;
      if (endEffectorControllers.size() <= controllerIndex)
         return false;
      return endEffectorControllers.get(controllerIndex).isEnabled();
   }

   private void submitOneDoFJointFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode, OneDoFJointFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());
      OneDoFJointBasics joint = feedbackControlCommand.getJoint();
      OneDoFJointFeedbackController controller = oneDoFJointFeedbackControllerMap.get(joint);

      if (controller == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
         {
            throw new FeedbackControllerException("No feedback controller was created for the joint " + joint.getName() + ".");
         }
         else
         {
            registerOneDoFJointControllers(joint);
            controller = oneDoFJointFeedbackControllerMap.get(joint);
         }
      }

      if (controller.isEnabled())
      {
         throw new FeedbackControllerException("Cannot submit more than one feedback control command to the same controller. Controller joint: "
               + joint.getName());
      }

      controller.submitFeedbackControlCommand(feedbackControlCommand);
      controller.setEnabled(true);
   }

   private void submitCenterOfMassFeedbackControlCommand(WholeBodyControllerCoreMode activeControlMode,
                                                         CenterOfMassFeedbackControlCommand feedbackControlCommand)
   {
      checkRequestedControlMode(activeControlMode, feedbackControlCommand.getControlMode());

      if (centerOfMassFeedbackController == null)
      {
         if (!dynamicControllerConstructionEnabled.getValue())
            throw new FeedbackControllerException("No feedback controller was created for the center of mass.");
         else
            registerCenterOfMassController();
      }

      centerOfMassFeedbackController.submitFeedbackControlCommand(feedbackControlCommand);
      centerOfMassFeedbackController.setEnabled(true);
   }

   private static void checkRequestedControlMode(WholeBodyControllerCoreMode activeControlMode, WholeBodyControllerCoreMode requestedControlMode)
   {
      if (activeControlMode != requestedControlMode)
         throw new FeedbackControllerException("Incompatible feedback control command: command requires: " + requestedControlMode + ", current mode: "
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

   public FeedbackControllerDataHolderReadOnly getWholeBodyFeedbackControllerDataHolder()
   {
      return feedbackControllerToolbox;
   }
}
