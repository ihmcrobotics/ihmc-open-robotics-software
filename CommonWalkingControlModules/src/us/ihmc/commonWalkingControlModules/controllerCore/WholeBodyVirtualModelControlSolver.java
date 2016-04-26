package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class WholeBodyVirtualModelControlSolver
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullRobotModel controllerModel;

   private final VirtualModelControlOptimizationControlModule optimizationControlModule;
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

   private final OneDoFJoint[] controlledOneDoFJoints;
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>();

   private final YoFrameVector yoDesiredMomentumRateLinear;
   private final YoFrameVector yoAchievedMomentumRateLinear;
   private final FrameVector achievedMomentumRateLinear = new FrameVector();
   private final Map<OneDoFJoint, DoubleYoVariable> jointTorqueSolution = new HashMap<>();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector residualRootJointForce = new FrameVector();
   private final FrameVector residualRootJointTorque = new FrameVector();

   private final YoFrameVector yoResidualRootJointForce;
   private final YoFrameVector yoResidualRootJointTorque;

   private final double controlDT;

   public WholeBodyVirtualModelControlSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      controlDT = toolbox.getControlDT();
      twistCalculator = toolbox.getTwistCalculator();
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();
      YoGraphicsListRegistry yoGraphicsListRegistry = toolbox.getYoGraphicsListRegistry();

      controllerModel = toolbox.getFullRobotModel();
      rootJoint = toolbox.getRobotRootJoint();
      virtualModelController = new VirtualModelController(toolbox.getGeometricJacobianHolder(), rootJoint.getSuccessor());
      optimizationControlModule = new VirtualModelControlOptimizationControlModule(toolbox, rootJoint, registry);

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, LowLevelJointControlMode.FORCE_CONTROL);

      for (OneDoFJoint joint : controlledOneDoFJoints)
      {
         DoubleYoVariable jointAccelerationSolution = new DoubleYoVariable("tau_vmc_" + joint.getName(), registry);
         jointTorqueSolution.put(joint, jointAccelerationSolution);
      }

      RigidBody[] endEffectors = toolbox.getEndEffectors();
      for (int i = 0; i < endEffectors.length; i++)
         virtualModelController.registerEndEffector(endEffectors[i]);

      planeContactWrenchProcessor = toolbox.getPlaneContactWrenchProcessor();
      wrenchVisualizer = toolbox.getWrenchVisualizer();

      yoDesiredMomentumRateLinear = toolbox.getYoDesiredMomentumRateLinear();
      yoAchievedMomentumRateLinear = toolbox.getYoAchievedMomentumRateLinear();

      yoResidualRootJointForce  = toolbox.getYoResidualRootJointForce();
      yoResidualRootJointTorque = toolbox.getYoResidualRootJointTorque();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      optimizationControlModule.initialize();
      virtualModelController.reset();
      virtualWrenchCommandList.clear();
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

      Map<RigidBody, Wrench> externalWrenchSolution = virtualModelControlSolution.getExternalWrenchSolution();
      List<RigidBody> rigidBodiesWithExternalWrench = virtualModelControlSolution.getRigidBodiesWithExternalWrench();
      SpatialForceVector centroidalMomentumRateSolution = virtualModelControlSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.set(centroidalMomentumRateSolution.getLinearPart());
      yoAchievedMomentumRateLinear.getFrameTupleIncludingFrame(achievedMomentumRateLinear);

      // submit forces for stability
      for (RigidBody rigidBody : rigidBodiesWithExternalWrench)
      {
         virtualModelController.submitEndEffectorVirtualWrench(rigidBody, externalWrenchSolution.get(rigidBody));
      }
      // submit virtual wrenches for tracking
      for (int i = 0; i < virtualWrenchCommandList.getNumberOfCommands(); i++)
      {
         virtualWrenchCommand.set(virtualWrenchCommandList.getCommand(i));
         if (!rigidBodiesWithExternalWrench.contains(virtualWrenchCommand.getRigidBody()))
            virtualModelController.submitEndEffectorVirtualWrench(virtualWrenchCommand);
      }

      virtualModelController.compute(virtualModelControlSolution);
      Map<InverseDynamicsJoint, Double> jointTorquesSolution = virtualModelControlSolution.getJointTorques();

      for (OneDoFJoint joint : controlledOneDoFJoints)
      {
         if (jointTorquesSolution.containsKey(joint))
            joint.setTau(jointTorquesSolution.get(joint));
      }

      updateLowLevelData();

      rootJoint.getWrench(residualRootJointWrench);
      residualRootJointWrench.getAngularPartIncludingFrame(residualRootJointTorque);
      residualRootJointWrench.getLinearPartIncludingFrame(residualRootJointForce);
      yoResidualRootJointForce.setAndMatchFrame(residualRootJointForce);
      yoResidualRootJointTorque.setAndMatchFrame(residualRootJointTorque);

      for (OneDoFJoint joint : controlledOneDoFJoints)
         jointTorqueSolution.get(joint).set(joint.getTau());

      planeContactWrenchProcessor.compute(externalWrenchSolution);
      wrenchVisualizer.visualize(externalWrenchSolution);
   }

   private void updateLowLevelData()
   {
      rootJointDesiredConfiguration.setDesiredAccelerationFromJoint(rootJoint);
      lowLevelOneDoFJointDesiredDataHolder.setDesiredTorqueFromJoints(controlledOneDoFJoints);
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
            optimizationControlModule.submitMomentumRateCommand((MomentumRateCommand) command);
            recordMomentumRate((MomentumRateCommand) command);
            break;
         case EXTERNAL_WRENCH:
            optimizationControlModule.submitExternalWrenchCommand((ExternalWrenchCommand) command);
            break;
         case PLANE_CONTACT_STATE:
            optimizationControlModule.submitPlaneContactStateCommand((PlaneContactStateCommand) command);
            break;
         case JOINT_ACCELERATION_INTEGRATION:
            submitJointAccelerationIntegrationCommand((JointAccelerationIntegrationCommand) command);
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
      DenseMatrix64F momentumRate = command.getMomentumRate();
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateLinear, momentumRate, 3);
   }

   private void convertAndAddSpatialAccelerationCommand(SpatialAccelerationCommand command)
   {
      RigidBody endEffector = command.getEndEffector();
      RigidBodyInertia inertia = endEffector.getInertia();

      SpatialAccelerationVector accelerationVector = command.getSpatialAcceleration();
      accelerationVector.changeBodyFrameNoRelativeAcceleration(inertia.getBodyFrame());
      accelerationVector.changeBaseFrameNoRelativeAcceleration(ReferenceFrame.getWorldFrame());

      tmpWrench.setToZero(accelerationVector.getBodyFrame(), accelerationVector.getExpressedInFrame());

      twistCalculator.getTwistOfBody(tmpTwist, endEffector);
      inertia.computeDynamicWrenchInBodyCoordinates(tmpWrench, accelerationVector, tmpTwist);

      virtualWrenchCommand.set(endEffector, tmpWrench, command.getSelectionMatrix());
      virtualWrenchCommandList.addCommand(virtualWrenchCommand);
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
