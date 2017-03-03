package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.*;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.*;

public class WholeBodyInverseDynamicsSolver
{
   private static final boolean USE_DYNAMIC_MATRIX_CALCULATOR = false;
   private static final boolean MINIMIZE_JOINT_TORQUES = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final InverseDynamicsOptimizationControlModule optimizationControlModule;
   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final Map<OneDoFJoint, DoubleYoVariable> jointAccelerationsSolution = new HashMap<>();

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;
   private final JointAccelerationIntegrationCalculator jointAccelerationIntegrationCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final OneDoFJoint[] controlledOneDoFJoints;
   private final InverseDynamicsJoint[] jointsToOptimizeFor;

   private final YoFrameVector yoDesiredMomentumRateLinear;
   private final YoFrameVector yoDesiredMomentumRateAngular;
   // TODO It seems that the achieved CMP (computed from this guy) can be off sometimes.
   // Need to review the computation of the achieved linear momentum rate or of the achieved CMP. (Sylvain)
   private final YoFrameVector yoAchievedMomentumRateLinear;
   private final YoFrameVector yoAchievedMomentumRateAngular;
   private final FrameVector achievedMomentumRateLinear = new FrameVector();
   private final FrameVector achievedMomentumRateAngular = new FrameVector();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector residualRootJointForce = new FrameVector();
   private final FrameVector residualRootJointTorque = new FrameVector();

   private final YoFrameVector yoResidualRootJointForce;
   private final YoFrameVector yoResidualRootJointTorque;

   private final double controlDT;

   public WholeBodyInverseDynamicsSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      controlDT = toolbox.getControlDT();
      rootJoint = toolbox.getRobotRootJoint();
      inverseDynamicsCalculator = toolbox.getInverseDynamicsCalculator();
      WrenchMatrixCalculator wrenchMatrixCalculator = new WrenchMatrixCalculator(toolbox, registry);
      dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox, wrenchMatrixCalculator);
      optimizationControlModule = new InverseDynamicsOptimizationControlModule(toolbox, wrenchMatrixCalculator, dynamicsMatrixCalculator, registry);
      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, LowLevelJointControlMode.FORCE_CONTROL);

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJoint joint = controlledOneDoFJoints[i];
         DoubleYoVariable jointAccelerationSolution = new DoubleYoVariable("qdd_qp_" + joint.getName(), registry);
         jointAccelerationsSolution.put(joint, jointAccelerationSolution);
      }

      planeContactWrenchProcessor = toolbox.getPlaneContactWrenchProcessor();
      wrenchVisualizer = toolbox.getWrenchVisualizer();

      jointAccelerationIntegrationCalculator = new JointAccelerationIntegrationCalculator(controlDT, registry);

      yoDesiredMomentumRateLinear = toolbox.getYoDesiredMomentumRateLinear();
      yoAchievedMomentumRateLinear = toolbox.getYoAchievedMomentumRateLinear();
      yoDesiredMomentumRateAngular = toolbox.getYoDesiredMomentumRateAngular();
      yoAchievedMomentumRateAngular = toolbox.getYoAchievedMomentumRateAngular();

      yoResidualRootJointForce = toolbox.getYoResidualRootJointForce();
      yoResidualRootJointTorque = toolbox.getYoResidualRootJointTorque();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      optimizationControlModule.initialize();

      if (USE_DYNAMIC_MATRIX_CALCULATOR)
         dynamicsMatrixCalculator.reset();
      else
         inverseDynamicsCalculator.reset();
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
      optimizationControlModule.initialize();
      planeContactWrenchProcessor.initialize();

      if (USE_DYNAMIC_MATRIX_CALCULATOR)
         dynamicsMatrixCalculator.reset();
      else
         inverseDynamicsCalculator.compute();
   }

   public void compute()
   {
      if (USE_DYNAMIC_MATRIX_CALCULATOR)
      {
         dynamicsMatrixCalculator.compute();
         if (MINIMIZE_JOINT_TORQUES)
            optimizationControlModule.setupTorqueMinimizationCommand();
      }

      MomentumModuleSolution momentumModuleSolution;

      try
      {
         momentumModuleSolution = optimizationControlModule.compute();
      }
      catch (MomentumControlModuleException momentumControlModuleException)
      {
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();
      }

      DenseMatrix64F jointAccelerations = momentumModuleSolution.getJointAccelerations();
      DenseMatrix64F rhoSolution = momentumModuleSolution.getRhoSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
      List<RigidBody> rigidBodiesWithExternalWrench = momentumModuleSolution.getRigidBodiesWithExternalWrench();
      SpatialForceVector centroidalMomentumRateSolution = momentumModuleSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.set(centroidalMomentumRateSolution.getLinearPart());
      yoAchievedMomentumRateLinear.getFrameTupleIncludingFrame(achievedMomentumRateLinear);

      yoAchievedMomentumRateAngular.set(centroidalMomentumRateSolution.getAngularPart());
      yoAchievedMomentumRateAngular.getFrameTupleIncludingFrame(achievedMomentumRateAngular);


      if (USE_DYNAMIC_MATRIX_CALCULATOR)
      {
         spatialAccelerationCalculator.compute();

         dynamicsMatrixCalculator.compute();
         DenseMatrix64F tauSolution = dynamicsMatrixCalculator.computeJointTorques(jointAccelerations, rhoSolution);

         ScrewTools.setDesiredAccelerations(jointsToOptimizeFor, jointAccelerations);
         ScrewTools.setJointTorques(controlledOneDoFJoints, tauSolution);
      }
      else
      {
         for (int i = 0; i < rigidBodiesWithExternalWrench.size(); i++)
         {
            RigidBody rigidBody = rigidBodiesWithExternalWrench.get(i);
            inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenchSolution.get(rigidBody));
         }

         ScrewTools.setDesiredAccelerations(jointsToOptimizeFor, jointAccelerations);
         inverseDynamicsCalculator.compute();
      }

      updateLowLevelData();

      rootJoint.getWrench(residualRootJointWrench);
      residualRootJointWrench.getAngularPartIncludingFrame(residualRootJointTorque);
      residualRootJointWrench.getLinearPartIncludingFrame(residualRootJointForce);
      yoResidualRootJointForce.setAndMatchFrame(residualRootJointForce);
      yoResidualRootJointTorque.setAndMatchFrame(residualRootJointTorque);

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJoint joint = controlledOneDoFJoints[i];
         jointAccelerationsSolution.get(joint).set(joint.getQddDesired());
      }

      planeContactWrenchProcessor.compute(externalWrenchSolution);
      wrenchVisualizer.visualize(externalWrenchSolution);
   }

   private void updateLowLevelData()
   {
      rootJointDesiredConfiguration.setDesiredAccelerationFromJoint(rootJoint);
      lowLevelOneDoFJointDesiredDataHolder.setDesiredTorqueFromJoints(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setDesiredAccelerationFromJoints(controlledOneDoFJoints);

      jointAccelerationIntegrationCalculator.computeAndUpdateDataHolder(lowLevelOneDoFJointDesiredDataHolder);
   }

   public void submitInverseDynamicsCommandList(InverseDynamicsCommandList inverseDynamicsCommandList)
   {
      while (inverseDynamicsCommandList.getNumberOfCommands() > 0)
      {
         InverseDynamicsCommand<?> command = inverseDynamicsCommandList.pollCommand();
         switch (command.getCommandType())
         {
         case TASKSPACE:
            optimizationControlModule.submitSpatialAccelerationCommand((SpatialAccelerationCommand) command);
            break;
         case POINT:
            optimizationControlModule.submitPointAccelerationCommand((PointAccelerationCommand) command);
            break;
         case JOINTSPACE:
            optimizationControlModule.submitJointspaceAccelerationCommand((JointspaceAccelerationCommand) command);
            break;
         case MOMENTUM:
            optimizationControlModule.submitMomentumRateCommand((MomentumRateCommand) command);
            recordMomentumRate((MomentumRateCommand) command);
            break;
         case PRIVILEGED_CONFIGURATION:
            optimizationControlModule.submitPrivilegedConfigurationCommand((PrivilegedConfigurationCommand) command);
            break;
         case LIMIT_REDUCTION:
            optimizationControlModule.submitJointLimitReductionCommand((JointLimitReductionCommand) command);
            break;
         case JOINT_LIMIT_ENFORCEMENT:
            optimizationControlModule.submitJointLimitEnforcementMethodCommand((JointLimitEnforcementMethodCommand) command);
            break;
         case EXTERNAL_WRENCH:
            optimizationControlModule.submitExternalWrenchCommand((ExternalWrenchCommand) command);
            if (USE_DYNAMIC_MATRIX_CALCULATOR)
               dynamicsMatrixCalculator.setExternalWrench(((ExternalWrenchCommand) command).getRigidBody(), ((ExternalWrenchCommand) command).getExternalWrench());
            break;
         case PLANE_CONTACT_STATE:
            optimizationControlModule.submitPlaneContactStateCommand((PlaneContactStateCommand) command);
            break;
         case CENTER_OF_PRESSURE:
            optimizationControlModule.submitCenterOfPressureCommand((CenterOfPressureCommand) command);
            break;
         case JOINT_ACCELERATION_INTEGRATION:
            jointAccelerationIntegrationCalculator.submitJointAccelerationIntegrationCommand((JointAccelerationIntegrationCommand) command);
            break;
         case COMMAND_LIST:
            submitInverseDynamicsCommandList((InverseDynamicsCommandList) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }
   }

   private void recordMomentumRate(MomentumRateCommand command)
   {
      DenseMatrix64F momentumRate = command.getMomentumRate();
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateAngular, momentumRate, 0);
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateLinear, momentumRate, 3);
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

   public FrameVector getAchievedMomentumRateAngular()
   {
      return achievedMomentumRateAngular;
   }

   public InverseDynamicsJoint[] getJointsToOptimizeFors()
   {
      return jointsToOptimizeFor;
   }
}
