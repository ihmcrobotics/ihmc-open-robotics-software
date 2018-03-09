package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelController;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public class WholeBodyVirtualModelControlSolver
{
   private static final boolean USE_LIMITED_JOINT_TORQUES = true;
   private static final boolean USE_CONTACT_FORCE_QP = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final VirtualModelControlOptimizationControlModule optimizationControlModule;
   private final VirtualModelController virtualModelController;

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final VirtualWrenchCommandList virtualWrenchCommandList = new VirtualWrenchCommandList();
   private final Wrench tmpWrench = new Wrench();

   private final OneDoFJoint[] controlledOneDoFJoints;
   private final RigidBody controlRootBody;

   private final List<RigidBody> controlledBodies;

   private final YoFrameVector yoDesiredMomentumRateLinear;
   private final YoFrameVector yoAchievedMomentumRateLinear;
   private final YoFrameVector yoDesiredMomentumRateAngular;
   private final YoFrameVector yoAchievedMomentumRateAngular;
   private final FrameVector3D achievedMomentumRateLinear = new FrameVector3D();
   private final Map<OneDoFJoint, RateLimitedYoVariable> jointTorqueSolutions = new HashMap<>();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector3D residualRootJointForce = new FrameVector3D();
   private final FrameVector3D residualRootJointTorque = new FrameVector3D();

   private final YoFrameVector yoResidualRootJointForce;
   private final YoFrameVector yoResidualRootJointTorque;

   private boolean firstTick = true;

   public WholeBodyVirtualModelControlSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      rootJoint = toolbox.getRootJoint();
      optimizationControlModule = new VirtualModelControlOptimizationControlModule(toolbox, USE_CONTACT_FORCE_QP, registry);

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, JointDesiredControlMode.EFFORT);

      controlRootBody = toolbox.getVirtualModelControlMainBody();
      virtualModelController = new VirtualModelController(controlRootBody, controlledOneDoFJoints, registry, toolbox.getYoGraphicsListRegistry());

      yoDesiredMomentumRateAngular = toolbox.getYoDesiredMomentumRateAngular();
      yoAchievedMomentumRateAngular = toolbox.getYoAchievedMomentumRateAngular();

      if (toolbox.getControlledBodies() != null)
      {
         controlledBodies = Arrays.asList(toolbox.getControlledBodies());
         for (RigidBody controlledBody : controlledBodies)
         {
            virtualModelController.createYoVariable(controlledBody);
         }

         wrenchVisualizer = new WrenchVisualizer("VMCDesiredExternalWrench", controlledBodies, 1.0, toolbox.getYoGraphicsListRegistry(), registry,
                                                 YoAppearance.Red(), YoAppearance.Blue());
      }
      else
      {
         controlledBodies = null;
         wrenchVisualizer = null;
      }

      if (USE_LIMITED_JOINT_TORQUES)
      {
         for (OneDoFJoint joint : controlledOneDoFJoints)
         {
            RateLimitedYoVariable jointTorqueSolution = new RateLimitedYoVariable("limited_tau_vmc_" + joint.getName(), registry, 10.0, toolbox.getControlDT());
            jointTorqueSolutions.put(joint, jointTorqueSolution);
         }
      }

      planeContactWrenchProcessor = toolbox.getPlaneContactWrenchProcessor();

      yoDesiredMomentumRateLinear = toolbox.getYoDesiredMomentumRateLinear();
      yoAchievedMomentumRateLinear = toolbox.getYoAchievedMomentumRateLinear();

      yoResidualRootJointForce = toolbox.getYoResidualRootJointForce();
      yoResidualRootJointTorque = toolbox.getYoResidualRootJointTorque();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      optimizationControlModule.initialize();
      virtualModelController.reset();
      virtualWrenchCommandList.clear();
      firstTick = true;
   }

   public void clear()
   {
      optimizationControlModule.initialize();
      virtualModelController.clear();
      virtualWrenchCommandList.clear();
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
      optimizationControlModule.initialize();
      virtualModelController.reset();
      planeContactWrenchProcessor.initialize();
      firstTick = true;
   }

   public void compute()
   {
      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
      try
      {
         optimizationControlModule.compute(virtualModelControlSolution);
      }
      catch (VirtualModelControlModuleException virtualModelControlModuleException)
      {
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
         virtualModelControlSolution = virtualModelControlModuleException.getVirtualModelControlSolution();
      }

      // get output for contact forces
      Map<RigidBody, Wrench> externalWrenchSolution = virtualModelControlSolution.getExternalWrenchSolution();
      List<RigidBody> rigidBodiesWithExternalWrench = virtualModelControlSolution.getRigidBodiesWithExternalWrench();
      List<RigidBody> bodiesInContact = virtualModelControlSolution.getBodiesInContact();
      SpatialForceVector centroidalMomentumRateSolution = virtualModelControlSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.set(centroidalMomentumRateSolution.getLinearPart());
      yoAchievedMomentumRateAngular.set(centroidalMomentumRateSolution.getAngularPart());
      achievedMomentumRateLinear.setIncludingFrame(yoAchievedMomentumRateLinear);

      // submit forces for contact forces
      for (RigidBody rigidBody : bodiesInContact)
      {
         externalWrenchSolution.get(rigidBody).negate();
         virtualModelController.submitControlledBodyVirtualWrench(rigidBody, externalWrenchSolution.get(rigidBody),
                                                                  virtualModelControlSolution.getCentroidalMomentumSelectionMatrix());
      }
      planeContactWrenchProcessor.compute(externalWrenchSolution);

      // submit virtual wrenches for bodies not in contact
      for (int i = 0; i < virtualWrenchCommandList.getNumberOfCommands(); i++)
      {
         VirtualWrenchCommand virtualWrenchCommand = virtualWrenchCommandList.getCommand(i);
         if (!bodiesInContact.contains(virtualWrenchCommand.getControlledBody()))
         {
            if (controlledBodies.contains(virtualWrenchCommand.getControlledBody()))
            {
               virtualModelController.submitControlledBodyVirtualWrench(virtualWrenchCommand);
            }
            else
               PrintTools.warn(this, "Received a command for " + virtualWrenchCommand.getControlledBody().getName()
                     + ", which is not registered. Skipping this body.");
         }
      }

      virtualModelController.compute(virtualModelControlSolution);
      Map<InverseDynamicsJoint, Double> jointTorquesSolution = virtualModelControlSolution.getJointTorques();

      for (RigidBody rigidBody : rigidBodiesWithExternalWrench)
      {
         externalWrenchSolution.get(rigidBody).changeBodyFrameAttachedToSameBody(rigidBody.getBodyFixedFrame());
         externalWrenchSolution.get(rigidBody).negate();
      }

      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(externalWrenchSolution);

      updateLowLevelData(jointTorquesSolution);

      rootJoint.getWrench(residualRootJointWrench);
      residualRootJointWrench.getAngularPartIncludingFrame(residualRootJointTorque);
      residualRootJointWrench.getLinearPartIncludingFrame(residualRootJointForce);
      yoResidualRootJointForce.setAndMatchFrame(residualRootJointForce);
      yoResidualRootJointTorque.setAndMatchFrame(residualRootJointTorque);
   }

   private void updateLowLevelData(Map<InverseDynamicsJoint, Double> jointTorquesSolution)
   {
      rootJointDesiredConfiguration.setDesiredAccelerationFromJoint(rootJoint);

      for (OneDoFJoint joint : controlledOneDoFJoints)
      {
         if (jointTorquesSolution.containsKey(joint))
         {
            if (USE_LIMITED_JOINT_TORQUES)
            {
               if (firstTick)
                  jointTorqueSolutions.get(joint).set(jointTorquesSolution.get(joint));
               else
                  jointTorqueSolutions.get(joint).update(jointTorquesSolution.get(joint));
               lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, jointTorqueSolutions.get(joint).getDoubleValue());
            }
            else
            {
               lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, jointTorquesSolution.get(joint));
            }
         }
      }
   }

   public void submitVirtualModelControlCommandList(VirtualModelControlCommandList virtualModelControlCommandList)
   {
      while (virtualModelControlCommandList.getNumberOfCommands() > 0)
      {
         VirtualModelControlCommand<?> command = virtualModelControlCommandList.pollCommand();
         switch (command.getCommandType())
         {
         case MOMENTUM:
            optimizationControlModule.submitMomentumRateCommand((MomentumRateCommand) command);
            recordMomentumRate((MomentumRateCommand) command);
            break;
         case EXTERNAL_WRENCH:
            handleExternalWrenchCommand((ExternalWrenchCommand) command);
            break;
         case PLANE_CONTACT_STATE:
            optimizationControlModule.submitPlaneContactStateCommand((PlaneContactStateCommand) command);
            break;
         case VIRTUAL_WRENCH:
            handleVirtualWrenchCommand((VirtualWrenchCommand) command);
            break;
            /* TODO
         case VIRTUAL_FORCE:
            break;
         case VIRTUAL_TORQUE:
            break;
         case JOINT_TORQUE:
            break;
            */
         case CONTROLLED_BODIES:
            registerAllControlledBodies((ControlledBodiesCommand) command);
            break;
         case COMMAND_LIST:
            submitVirtualModelControlCommandList((VirtualModelControlCommandList) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled by the Jacobian Transpose solver mode.");
         }
      }
   }

   private void recordMomentumRate(MomentumRateCommand command)
   {
      DenseMatrix64F momentumRate = command.getMomentumRate();
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateLinear, momentumRate, 3);
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateAngular, momentumRate, 0);
   }

   private final Wrench tmpExternalWrench = new Wrench();

   private void handleVirtualWrenchCommand(VirtualWrenchCommand command)
   {
      virtualWrenchCommandList.addCommand(command);

      if (command.getControlledBody() == controlRootBody)
      {
         tmpExternalWrench.set(command.getVirtualWrench());
         tmpExternalWrench.negate();
         optimizationControlModule.submitExternalWrench(command.getControlledBody(), tmpExternalWrench);
      }

      optimizationControlModule.addSelection(command.getSelectionMatrix());
   }

   private void handleExternalWrenchCommand(ExternalWrenchCommand command)
   {
      optimizationControlModule.submitExternalWrench(command.getRigidBody(), tmpExternalWrench);

      tmpWrench.set(command.getExternalWrench());
      tmpWrench.negate();

      VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
      virtualWrenchCommand.set(command.getRigidBody(), tmpWrench);
      virtualWrenchCommandList.addCommand(virtualWrenchCommand);
   }

   private void registerAllControlledBodies(ControlledBodiesCommand command)
   {
      // clear list of all bodies currently being used
      virtualModelController.reset();

      // add bodies desired to be used
      for (int i = 0; i < command.getNumberOfControlledBodies(); i++)
      {
         if (command.hasJointsToUse(i))
            virtualModelController.registerControlledBody(command.getControlledBody(i), command.getJointsToUse(i));
         else if (command.hasBaseForControl(i))
            virtualModelController.registerControlledBody(command.getControlledBody(i), command.getBaseForControl(i));
         else
            virtualModelController.registerControlledBody(command.getControlledBody(i));
      }
   }

   public LowLevelOneDoFJointDesiredDataHolder getOutput()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return rootJointDesiredConfiguration;
   }
}
