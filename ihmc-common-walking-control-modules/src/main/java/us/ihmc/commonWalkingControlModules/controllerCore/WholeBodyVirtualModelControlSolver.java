package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelMomentumController;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.VectorMean;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.List;
import java.util.Map;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class WholeBodyVirtualModelControlSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final VirtualModelControlOptimizationControlModule optimizationControlModule;
   private final VirtualModelMomentumController virtualModelController;

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final ReferenceFrame centerOfMassFrame;

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

   private final RigidBody controlRootBody;

   private final Wrench tempWrench = new Wrench();
   private final FrameVector3D tempForce = new FrameVector3D();
   private final FrameVector3D tempTorque = new FrameVector3D();
   private final SelectionMatrix6D tempSelectionMatrix = new SelectionMatrix6D();

   private final YoFrameVector3D yoDesiredMomentumRateLinear;
   private final YoFrameVector3D yoAchievedMomentumRateLinear;
   private final YoFrameVector3D yoDesiredMomentumRateAngular;
   private final YoFrameVector3D yoAchievedMomentumRateAngular;
   private final FrameVector3D achievedMomentumRateLinear = new FrameVector3D();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector3D residualRootJointForce = new FrameVector3D();
   private final FrameVector3D residualRootJointTorque = new FrameVector3D();

   private final YoFrameVector3D yoResidualRootJointForce;
   private final YoFrameVector3D yoResidualRootJointTorque;

   private final JointIndexHandler jointIndexHandler;
   private final WholeBodyControllerBoundCalculator boundCalculator;

   private final Vector3DReadOnly defaultLinearMomentumWeight = new ParameterVector3D("DefaultVMCRootLinearMomentumRateWeight", new Vector3D(5.0, 5.0, 2.5), registry);
   private final Vector3DReadOnly defaultAngularMomentumWeight = new ParameterVector3D("DefaultVMCRootAngularMomentumRateWeight", new Vector3D(2.5, 2.5, 1.0), registry);

   private final MomentumRateCommand rootBodyDefaultMomentumCommand = new MomentumRateCommand();

   public WholeBodyVirtualModelControlSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      rootJoint = toolbox.getRootJoint();
      optimizationControlModule = new VirtualModelControlOptimizationControlModule(toolbox, registry);
      centerOfMassFrame = toolbox.getCenterOfMassFrame();

      jointIndexHandler = toolbox.getJointIndexHandler();
      OneDoFJoint[] controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, JointDesiredControlMode.EFFORT);

      boundCalculator = toolbox.getQPBoundCalculator();

      controlRootBody = toolbox.getVirtualModelControlMainBody();
      virtualModelController = new VirtualModelMomentumController(toolbox.getJointIndexHandler());

      yoDesiredMomentumRateLinear = toolbox.getYoDesiredMomentumRateLinear();
      yoAchievedMomentumRateLinear = toolbox.getYoAchievedMomentumRateLinear();
      yoDesiredMomentumRateAngular = toolbox.getYoDesiredMomentumRateAngular();
      yoAchievedMomentumRateAngular = toolbox.getYoAchievedMomentumRateAngular();

      planeContactWrenchProcessor = toolbox.getPlaneContactWrenchProcessor();
      wrenchVisualizer = toolbox.getWrenchVisualizer();

      yoResidualRootJointForce = toolbox.getYoResidualRootJointForce();
      yoResidualRootJointTorque = toolbox.getYoResidualRootJointTorque();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      optimizationControlModule.initialize();
      virtualModelController.reset();

      yoDesiredMomentumRateLinear.setToZero();
      yoDesiredMomentumRateAngular.setToZero();
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
      optimizationControlModule.initialize();
      virtualModelController.reset();
      planeContactWrenchProcessor.initialize();
   }

   public void compute()
   {
      VirtualModelControlSolution virtualModelControlSolution;

      try
      {
         virtualModelControlSolution = optimizationControlModule.compute();
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
      SpatialForceVector centroidalMomentumRateSolution = virtualModelControlSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.set(centroidalMomentumRateSolution.getLinearPart());
      yoAchievedMomentumRateAngular.set(centroidalMomentumRateSolution.getAngularPart());
      achievedMomentumRateLinear.setIncludingFrame(yoAchievedMomentumRateLinear);

      // submit forces for contact forces
      for (int bodyIndex = 0; bodyIndex < rigidBodiesWithExternalWrench.size(); bodyIndex++)
      {
         RigidBody rigidBody = rigidBodiesWithExternalWrench.get(bodyIndex);
         externalWrenchSolution.get(rigidBody).negate();
         virtualModelController.addExternalWrench(controlRootBody, rigidBody, externalWrenchSolution.get(rigidBody));
      }

      virtualModelController.populateTorqueSolution(virtualModelControlSolution);
      DenseMatrix64F jointTorquesSolution = virtualModelControlSolution.getJointTorques();

      for (int i = 0; i < rigidBodiesWithExternalWrench.size(); i++)
      {
         RigidBody rigidBody = rigidBodiesWithExternalWrench.get(i);
         externalWrenchSolution.get(rigidBody).changeBodyFrameAttachedToSameBody(rigidBody.getBodyFixedFrame());
         externalWrenchSolution.get(rigidBody).negate();
      }

      updateLowLevelData(jointTorquesSolution);

      boundCalculator.enforceJointTorqueLimits(lowLevelOneDoFJointDesiredDataHolder);

      if (rootJoint != null)
      {
         rootJoint.getWrench(residualRootJointWrench);
         residualRootJointWrench.getAngularPartIncludingFrame(residualRootJointTorque);
         residualRootJointWrench.getLinearPartIncludingFrame(residualRootJointForce);
         yoResidualRootJointForce.setMatchingFrame(residualRootJointForce);
         yoResidualRootJointTorque.setMatchingFrame(residualRootJointTorque);
      }

      planeContactWrenchProcessor.compute(externalWrenchSolution);
      wrenchVisualizer.visualize(externalWrenchSolution);
   }

   private void updateLowLevelData(DenseMatrix64F jointTorquesSolution)
   {
      if (rootJoint != null)
         rootJointDesiredConfiguration.setDesiredAccelerationFromJoint(rootJoint);

      for (OneDoFJoint joint : jointIndexHandler.getIndexedOneDoFJoints())
      {
         int[] jointIndices = jointIndexHandler.getJointIndices(joint);

         for (int jointIndex : jointIndices)
         {

            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, jointTorquesSolution.get(jointIndex));
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
         case VIRTUAL_FORCE:
            handleVirtualForceCommand((VirtualForceCommand) command);
            break;
         case VIRTUAL_TORQUE:
            handleVirtualTorqueCommand((VirtualTorqueCommand) command);
            break;
         case JOINTSPACE:
            virtualModelController.addJointTorqueCommand((JointTorqueCommand) command);
            break;
         case JOINT_LIMIT_ENFORCEMENT:
            boundCalculator.submitJointLimitEnforcementCommand((JointLimitEnforcementCommand) command);
            break;
         case COMMAND_LIST:
            submitVirtualModelControlCommandList((VirtualModelControlCommandList) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled by the Virtual Model Control solver mode.");
         }
      }
   }

   private void recordMomentumRate(MomentumRateCommand command)
   {
      DenseMatrix64F momentumRate = command.getMomentumRate();
      MatrixTools.extractAddFixedFrameTupleFromEJMLVector(yoDesiredMomentumRateLinear, momentumRate, 3);
      MatrixTools.extractAddFixedFrameTupleFromEJMLVector(yoDesiredMomentumRateAngular, momentumRate, 0);
   }

   private void handleVirtualWrenchCommand(VirtualWrenchCommand commandToSubmit)
   {
      if (commandToSubmit.getEndEffector() == controlRootBody)
      {
         commandToSubmit.getSelectionMatrix(tempSelectionMatrix);
         commandToSubmit.getDesiredWrench(controlFrame, tempWrench);
         tempWrench.getAngularPart(tempTorque);
         tempWrench.getLinearPart(tempForce);
         tempTorque.changeFrame(worldFrame);
         tempForce.changeFrame(worldFrame);

         rootBodyDefaultMomentumCommand.setSelectionMatrix(tempSelectionMatrix);
         rootBodyDefaultMomentumCommand.setWeights(defaultAngularMomentumWeight, defaultLinearMomentumWeight);
         rootBodyDefaultMomentumCommand.setMomentumRate(tempTorque, tempForce);

         optimizationControlModule.submitMomentumRateCommand(rootBodyDefaultMomentumCommand);
         recordMomentumRate(rootBodyDefaultMomentumCommand);

         return;
      }

      virtualModelController.addVirtualEffortCommand(commandToSubmit);
   }

   private void handleVirtualForceCommand(VirtualForceCommand commandToSubmit)
   {
      if (commandToSubmit.getEndEffector() == controlRootBody)
      {
         commandToSubmit.getSelectionMatrix(tempSelectionMatrix);
         commandToSubmit.getDesiredLinearForce(controlFrame, tempForce);
         tempForce.changeFrame(worldFrame);

         rootBodyDefaultMomentumCommand.setSelectionMatrix(tempSelectionMatrix);
         rootBodyDefaultMomentumCommand.setLinearWeights(defaultLinearMomentumWeight);
         rootBodyDefaultMomentumCommand.setLinearMomentumRate(tempForce);

         optimizationControlModule.submitMomentumRateCommand(rootBodyDefaultMomentumCommand);
         recordMomentumRate(rootBodyDefaultMomentumCommand);

         return;
      }

      virtualModelController.addVirtualEffortCommand(commandToSubmit);
   }

   private void handleVirtualTorqueCommand(VirtualTorqueCommand commandToSubmit)
   {
      if (commandToSubmit.getEndEffector() == controlRootBody)
      {
         commandToSubmit.getSelectionMatrix(tempSelectionMatrix);
         commandToSubmit.getDesiredAngularTorque(controlFrame, tempTorque);
         tempTorque.changeFrame(worldFrame);

         rootBodyDefaultMomentumCommand.setSelectionMatrix(tempSelectionMatrix);
         rootBodyDefaultMomentumCommand.setAngularWeights(defaultLinearMomentumWeight);
         rootBodyDefaultMomentumCommand.setAngularMomentumRate(tempTorque);

         optimizationControlModule.submitMomentumRateCommand(rootBodyDefaultMomentumCommand);
         recordMomentumRate(rootBodyDefaultMomentumCommand);

         return;
      }

      virtualModelController.addVirtualEffortCommand(commandToSubmit);
   }

   private void handleExternalWrenchCommand(ExternalWrenchCommand command)
   {
      optimizationControlModule.submitExternalWrench(command.getRigidBody(), command.getExternalWrench());
   }

   public LowLevelOneDoFJointDesiredDataHolder getOutput()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return rootJointDesiredConfiguration;
   }

   public FrameVector3DReadOnly getAchievedMomentumRateLinear()
   {
      return achievedMomentumRateLinear;
   }
}
