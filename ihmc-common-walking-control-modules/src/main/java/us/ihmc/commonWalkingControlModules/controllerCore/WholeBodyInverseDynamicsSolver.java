package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class WholeBodyInverseDynamicsSolver
{
   private static final boolean USE_DYNAMIC_MATRIX_CALCULATOR = false;
   private static final boolean MINIMIZE_JOINT_TORQUES = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final InverseDynamicsOptimizationControlModule optimizationControlModule;
   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final FloatingJointBasics rootJoint;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final Map<OneDoFJointBasics, YoDouble> jointAccelerationsSolution = new HashMap<>();

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;
   private final JointAccelerationIntegrationCalculator jointAccelerationIntegrationCalculator;

   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final JointBasics[] jointsToOptimizeFor;

   private final YoFrameVector3D yoDesiredMomentumRateLinear;
   private final YoFrameVector3D yoDesiredMomentumRateAngular;
   // TODO It seems that the achieved CMP (computed from this guy) can be off sometimes.
   // Need to review the computation of the achieved linear momentum rate or of the achieved CMP. (Sylvain)
   private final YoFrameVector3D yoAchievedMomentumRateLinear;
   private final YoFrameVector3D yoAchievedMomentumRateAngular;
   private final FrameVector3D achievedMomentumRateLinear = new FrameVector3D();
   private final FrameVector3D achievedMomentumRateAngular = new FrameVector3D();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector3D residualRootJointForce = new FrameVector3D();
   private final FrameVector3D residualRootJointTorque = new FrameVector3D();

   private final YoFrameVector3D yoResidualRootJointForce;
   private final YoFrameVector3D yoResidualRootJointTorque;

   private final double controlDT;

   public WholeBodyInverseDynamicsSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      controlDT = toolbox.getControlDT();
      rootJoint = toolbox.getRootJoint();
      inverseDynamicsCalculator = toolbox.getInverseDynamicsCalculator();
      WrenchMatrixCalculator wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
      dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox, wrenchMatrixCalculator);
      optimizationControlModule = new InverseDynamicsOptimizationControlModule(toolbox, dynamicsMatrixCalculator, registry);

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, JointDesiredControlMode.EFFORT);

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[i];
         YoDouble jointAccelerationSolution = new YoDouble("qdd_qp_" + joint.getName(), registry);
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
         inverseDynamicsCalculator.setExternalWrenchesToZero();
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

      optimizationControlModule.resetRateRegularization();
      for (int i = 0; i < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
         lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(i).clear();
   }

   public void compute()
   {
      if (USE_DYNAMIC_MATRIX_CALCULATOR)
      {
         dynamicsMatrixCalculator.compute();
         if (MINIMIZE_JOINT_TORQUES)
            optimizationControlModule.setupTorqueMinimizationCommand();
      }

      if (!optimizationControlModule.compute())
      {
         // TODO:
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
      }
      MomentumModuleSolution momentumModuleSolution = optimizationControlModule.getMomentumModuleSolution();

      DenseMatrix64F jointAccelerations = momentumModuleSolution.getJointAccelerations();
      DenseMatrix64F rhoSolution = momentumModuleSolution.getRhoSolution();
      Map<RigidBodyBasics, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
      List<RigidBodyBasics> rigidBodiesWithExternalWrench = momentumModuleSolution.getRigidBodiesWithExternalWrench();
      SpatialForceReadOnly centroidalMomentumRateSolution = momentumModuleSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.setMatchingFrame(centroidalMomentumRateSolution.getLinearPart());
      achievedMomentumRateLinear.setIncludingFrame(yoAchievedMomentumRateLinear);

      yoAchievedMomentumRateAngular.setMatchingFrame(centroidalMomentumRateSolution.getAngularPart());
      achievedMomentumRateAngular.setIncludingFrame(yoAchievedMomentumRateAngular);

      if (USE_DYNAMIC_MATRIX_CALCULATOR)
      {
         // TODO Since switch to Mecano: need to update the inverse dynamics to update the rigid-body accelerations, kinda dumb.
         //         rigidBodyAccelerationProvider.compute();
         inverseDynamicsCalculator.compute(jointAccelerations);

         dynamicsMatrixCalculator.compute();
         DenseMatrix64F tauSolution = dynamicsMatrixCalculator.computeJointTorques(jointAccelerations, rhoSolution);

         for (int jointIndex = 0; jointIndex < controlledOneDoFJoints.length; jointIndex++)
         {
            OneDoFJointBasics joint = controlledOneDoFJoints[jointIndex];
            int jointAccelerationIndex = inverseDynamicsCalculator.getInput().getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
            JointDesiredOutputBasics jointDesiredOutput = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
            jointDesiredOutput.setDesiredAcceleration(jointAccelerations.get(jointAccelerationIndex, 0));
            jointDesiredOutput.setDesiredTorque(tauSolution.get(jointIndex, 0));
         }
      }
      else
      {
         for (int i = 0; i < rigidBodiesWithExternalWrench.size(); i++)
         {
            RigidBodyBasics rigidBody = rigidBodiesWithExternalWrench.get(i);
            inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenchSolution.get(rigidBody));
         }

         inverseDynamicsCalculator.compute(jointAccelerations);

         for (OneDoFJointBasics joint : controlledOneDoFJoints)
         {
            int jointIndex = inverseDynamicsCalculator.getInput().getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
            JointDesiredOutputBasics jointDesiredOutput = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
            jointDesiredOutput.setDesiredAcceleration(jointAccelerations.get(jointIndex, 0));
            jointDesiredOutput.setDesiredTorque(inverseDynamicsCalculator.getComputedJointTau(joint).get(0));
         }
      }

      if (rootJoint != null)
         rootJointDesiredConfiguration.setDesiredAcceleration(jointAccelerations, 0);

      jointAccelerationIntegrationCalculator.computeAndUpdateDataHolder(lowLevelOneDoFJointDesiredDataHolder);

      if (rootJoint != null)
      {
         residualRootJointWrench.setIncludingFrame(rootJoint.getJointWrench());
         residualRootJointTorque.setIncludingFrame(residualRootJointWrench.getAngularPart());
         residualRootJointForce.setIncludingFrame(residualRootJointWrench.getLinearPart());
         yoResidualRootJointForce.setMatchingFrame(residualRootJointForce);
         yoResidualRootJointTorque.setMatchingFrame(residualRootJointTorque);
      }

      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); jointIndex++)
      {
         OneDoFJointBasics joint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         jointAccelerationsSolution.get(joint).set(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(jointIndex));
      }

      planeContactWrenchProcessor.compute(externalWrenchSolution);
      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(externalWrenchSolution);
   }

   public void submitResetIntegratorRequests(JointDesiredOutputListReadOnly jointDesiredOutputList)
   {
      for (int i = 0; i < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointBasics joint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(i);
         if (jointDesiredOutputList.hasDataForJoint(joint))
         {
            JointDesiredOutputReadOnly jointDesiredOutputOther = jointDesiredOutputList.getJointDesiredOutput(joint);
            lowLevelOneDoFJointDesiredDataHolder.setResetJointIntegrators(joint, jointDesiredOutputOther.peekResetIntegratorsRequest());
         }
      }
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
         case PRIVILEGED_JOINTSPACE_COMMAND:
            optimizationControlModule.submitPrivilegedAccelerationCommand((PrivilegedJointSpaceCommand) command);
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
               dynamicsMatrixCalculator.setExternalWrench(((ExternalWrenchCommand) command).getRigidBody(),
                                                          ((ExternalWrenchCommand) command).getExternalWrench());
            break;
         case CONTACT_WRENCH:
            optimizationControlModule.submitContactWrenchCommand((ContactWrenchCommand) command);
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
         case OPTIMIZATION_SETTINGS:
            optimizationControlModule.submitOptimizationSettingsCommand((InverseDynamicsOptimizationSettingsCommand) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }
   }

   // FIXME this assumes there is only one momentum rate command
   private void recordMomentumRate(MomentumRateCommand command)
   {
      DenseMatrix64F momentumRate = command.getMomentumRate();
      MatrixTools.extractFixedFrameTupleFromEJMLVector(yoDesiredMomentumRateAngular, momentumRate, 0);
      MatrixTools.extractFixedFrameTupleFromEJMLVector(yoDesiredMomentumRateLinear, momentumRate, 3);
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

   public FrameVector3DReadOnly getAchievedMomentumRateLinear()
   {
      return achievedMomentumRateLinear;
   }

   public FrameVector3DReadOnly getAchievedMomentumRateAngular()
   {
      return achievedMomentumRateAngular;
   }

   public JointBasics[] getJointsToOptimizeFors()
   {
      return jointsToOptimizeFor;
   }
}
