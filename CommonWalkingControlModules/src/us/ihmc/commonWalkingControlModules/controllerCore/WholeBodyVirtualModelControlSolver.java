package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.ControlledBodiesCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelController;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.*;

public class WholeBodyVirtualModelControlSolver
{
   private static final boolean USE_MOMENTUM_QP = false;
   private static final boolean USE_LIMITED_JOINT_TORQUES = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final VirtualModelControlOptimizationControlModule optimizationControlModule;
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final VirtualModelController virtualModelController;

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;

   private final SixDoFJoint rootJoint;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final VirtualWrenchCommandList virtualWrenchCommandList = new VirtualWrenchCommandList();
   private final VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
   private final TwistCalculator twistCalculator;
   private final Wrench tmpWrench = new Wrench();
   private final Twist tmpTwist = new Twist();

   private final FullRobotModel controllerModel;
   private final ReferenceFrame centerOfMassFrame;
   private final OneDoFJoint[] controlledOneDoFJoints;
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>();

   private final List<RigidBody> controlledBodies;
   private final Map<RigidBody, RigidBodyInertia> conversionInertias = new HashMap<>();

   private final YoFrameVector yoDesiredMomentumRateLinear;
   private final YoFrameVector yoAchievedMomentumRateLinear;
   private final YoFrameVector yoDesiredMomentumRateAngular;
   private final YoFrameVector yoAchievedMomentumRateAngular;
   private final FrameVector achievedMomentumRateLinear = new FrameVector();
   private final Map<OneDoFJoint, RateLimitedYoVariable> jointTorqueSolutions = new HashMap<>();
   private final List<RigidBody> bodiesInContact = new ArrayList<>();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector residualRootJointForce = new FrameVector();
   private final FrameVector residualRootJointTorque = new FrameVector();

   private final YoFrameVector yoResidualRootJointForce;
   private final YoFrameVector yoResidualRootJointTorque;

   private boolean firstTick = true;

   public WholeBodyVirtualModelControlSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      twistCalculator = toolbox.getTwistCalculator();

      centerOfMassFrame = toolbox.getCenterOfMassFrame();
      controllerModel = toolbox.getFullRobotModel();
      rootJoint = toolbox.getRobotRootJoint();
      optimizationControlModule = new VirtualModelControlOptimizationControlModule(toolbox, rootJoint, registry);

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, LowLevelJointControlMode.FORCE_CONTROL);

      virtualModelController = new VirtualModelController(toolbox.getGeometricJacobianHolder(), controllerModel.getPelvis(), controlledOneDoFJoints, registry,
            toolbox.getYoGraphicsListRegistry());

      yoDesiredMomentumRateAngular = new YoFrameVector("desiredMomentumRateAngular", toolbox.getCenterOfMassFrame(), registry);
      yoAchievedMomentumRateAngular = new YoFrameVector("achievedMomentumRateAngular", toolbox.getCenterOfMassFrame(), registry);

      controlledBodies = Arrays.asList(toolbox.getControlledBodies());
      for (RigidBody controlledBody : controlledBodies)
      {
         virtualModelController.createYoVariable(controlledBody);
         RigidBodyInertia conversionInertia = new RigidBodyInertia(controlledBody.getBodyFixedFrame(), 1.0, 1.0, 1.0, 1.0);
         conversionInertias.put(controlledBody, conversionInertia);
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
      wrenchVisualizer = new WrenchVisualizer("VMCDesiredExternalWrench", controlledBodies, 1.0, toolbox.getYoGraphicsListRegistry(),
         registry, YoAppearance.Red(), YoAppearance.Blue());

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
      bodiesInContact.clear();
      firstTick = true;
   }

   public void clear()
   {
      optimizationControlModule.initialize();
      virtualModelController.clear();
      virtualWrenchCommandList.clear();
      bodiesInContact.clear();
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
      if (USE_MOMENTUM_QP)
      {
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
      }
      else
      {
         if (CommonOps.elementSum(momentumRateCommand.getSelectionMatrix()) != 0.0)
            submitMomentumRateAsVirtualWrench(virtualModelControlSolution, momentumRateCommand);
      }

      Map<RigidBody, Wrench> externalWrenchSolution = virtualModelControlSolution.getExternalWrenchSolution();
      List<RigidBody> rigidBodiesWithExternalWrench = virtualModelControlSolution.getRigidBodiesWithExternalWrench();
      SpatialForceVector centroidalMomentumRateSolution = virtualModelControlSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.set(centroidalMomentumRateSolution.getLinearPart());
      yoAchievedMomentumRateAngular.set(centroidalMomentumRateSolution.getAngularPart());
      yoAchievedMomentumRateLinear.getFrameTupleIncludingFrame(achievedMomentumRateLinear);

      // submit forces for stability
      for (RigidBody rigidBody : rigidBodiesWithExternalWrench)
      {
         externalWrenchSolution.get(rigidBody).negate();
         virtualModelController.submitControlledBodyVirtualWrench(rigidBody, externalWrenchSolution.get(rigidBody), virtualModelControlSolution.getCentroidalMomentumSelectionMatrix());
      }
      planeContactWrenchProcessor.compute(externalWrenchSolution);

      // submit virtual wrenches for tracking
      for (int i = 0; i < virtualWrenchCommandList.getNumberOfCommands(); i++)
      {
         virtualWrenchCommand.set(virtualWrenchCommandList.getCommand(i));
         if (!rigidBodiesWithExternalWrench.contains(virtualWrenchCommand.getControlledBody()))
         {
            if (controlledBodies.contains(virtualWrenchCommand.getControlledBody()))
               virtualModelController.submitControlledBodyVirtualWrench(virtualWrenchCommand);
            else
               PrintTools.warn(this, "Received a command for " + virtualWrenchCommand.getControlledBody().getName() + ", which is not registered. Skipping this body.");
         }
      }

      virtualModelController.compute(virtualModelControlSolution);
      Map<InverseDynamicsJoint, Double> jointTorquesSolution = virtualModelControlSolution.getJointTorques();


      for (RigidBody rigidBody : rigidBodiesWithExternalWrench)
      {
         externalWrenchSolution.get(rigidBody).changeBodyFrameAttachedToSameBody(rigidBody.getBodyFixedFrame());
         externalWrenchSolution.get(rigidBody).negate();
      }
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

   public void submitVirtualModelControlCommandList(InverseDynamicsCommandList virtualModelControlCommandList)
   {
      while (virtualModelControlCommandList.getNumberOfCommands() > 0)
      {
         InverseDynamicsCommand<?> command = virtualModelControlCommandList.pollCommand();
         switch (command.getCommandType())
         {
         case TASKSPACE:
            convertAndAddSpatialAccelerationCommand((SpatialAccelerationCommand) command);
            break;
         case MOMENTUM:
            if (USE_MOMENTUM_QP)
               optimizationControlModule.submitMomentumRateCommand((MomentumRateCommand) command);
            recordMomentumRate((MomentumRateCommand) command);
            break;
         case EXTERNAL_WRENCH:
            optimizationControlModule.submitExternalWrenchCommand((ExternalWrenchCommand) command);
            break;
         case PLANE_CONTACT_STATE:
            optimizationControlModule.submitPlaneContactStateCommand((PlaneContactStateCommand) command);
            if (((PlaneContactStateCommand) command).getNumberOfContactPoints() > 0)
               bodiesInContact.add(((PlaneContactStateCommand) command).getContactingRigidBody());
            break;
         case JOINT_ACCELERATION_INTEGRATION:
            submitJointAccelerationIntegrationCommand((JointAccelerationIntegrationCommand) command);
            break;
         case VIRTUAL_WRENCH:
            virtualWrenchCommandList.addCommand((VirtualWrenchCommand) command);
            break;
         case CONTROLLED_BODIES:
            registerAllControlledBodies((ControlledBodiesCommand) command);
            break;
         case COMMAND_LIST:
            submitVirtualModelControlCommandList((InverseDynamicsCommandList) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled by the Jacobian Transpose solver mode.");
         }
      }
   }

   private void recordMomentumRate(MomentumRateCommand command)
   {
      momentumRateCommand.set(command);
      DenseMatrix64F momentumRate = command.getMomentumRate();
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateLinear, momentumRate, 3);
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateAngular, momentumRate, 0);
   }

   private void convertAndAddSpatialAccelerationCommand(SpatialAccelerationCommand command)
   {
      RigidBody controlledBody = command.getEndEffector();
      SpatialAccelerationVector accelerationVector = command.getSpatialAcceleration();

      accelerationVector.changeBodyFrameNoRelativeAcceleration(controlledBody.getBodyFixedFrame());
      accelerationVector.changeBaseFrameNoRelativeAcceleration(ReferenceFrame.getWorldFrame());
      tmpWrench.setToZero(accelerationVector.getBodyFrame(), accelerationVector.getExpressedInFrame());

      twistCalculator.getTwistOfBody(tmpTwist, controlledBody);
      conversionInertias.get(controlledBody).computeDynamicWrenchInBodyCoordinates(tmpWrench, accelerationVector, tmpTwist);

      VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
      virtualWrenchCommand.set(controlledBody, tmpWrench, command.getSelectionMatrix());
      virtualWrenchCommandList.addCommand(virtualWrenchCommand);
   }

   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final Map<RigidBody, Wrench> wrenchSolution = new HashMap<>();
   private final SpatialForceVector centroidalMomentumRateSolution = new SpatialForceVector();
   private void submitMomentumRateAsVirtualWrench(VirtualModelControlSolution virtualModelControlSolutionToPack, MomentumRateCommand command)
   {
      wrenchSolution.clear();

      DenseMatrix64F additionalExternalWrench = optimizationControlModule.getExternalWrenchHandler().getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = optimizationControlModule.getExternalWrenchHandler().getGravitationalWrench();
      DenseMatrix64F momentumRate = command.getMomentumRate();

      CommonOps.subtract(momentumRate, additionalExternalWrench, tempTaskObjective);
      CommonOps.subtract(tempTaskObjective, gravityWrench, tempTaskObjective);

      centroidalMomentumRateSolution.set(null, momentumRate);

      double loadFraction = 1.0 / (double) bodiesInContact.size();
      CommonOps.scale(loadFraction, tempTaskObjective);

      for (RigidBody controlledBody : bodiesInContact)
      {
         wrenchSolution.put(controlledBody, new Wrench(centerOfMassFrame, centerOfMassFrame, tempTaskObjective));
      }
      virtualModelControlSolutionToPack.setExternalWrenchSolution(bodiesInContact, wrenchSolution);
      virtualModelControlSolutionToPack.setCentroidalMomentumRateSolution(centroidalMomentumRateSolution);
      virtualModelControlSolutionToPack.setCentroidalMomentumSelectionMatrix(momentumRateCommand.getSelectionMatrix());
   }

   private void submitJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand command)
   {
      for (int i = 0; i < command.getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         OneDoFJoint jointToComputeDesiedPositionFor = command.getJointToComputeDesiredPositionFor(i);
         if (!jointsToComputeDesiredPositionFor.contains(jointToComputeDesiedPositionFor))
            jointsToComputeDesiredPositionFor.add(jointToComputeDesiedPositionFor);
      }
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

   public CenterOfPressureDataHolder getDesiredCenterOfPressureDataHolder()
   {
      return planeContactWrenchProcessor.getDesiredCenterOfPressureDataHolder();
   }

   public FrameVector getAchievedMomentumRateLinear()
   {
      return achievedMomentumRateLinear;
   }

   public InverseDynamicsJoint[] getJointsToOptimizeFors()
   {
      return jointsToOptimizeFor;
   }
}
