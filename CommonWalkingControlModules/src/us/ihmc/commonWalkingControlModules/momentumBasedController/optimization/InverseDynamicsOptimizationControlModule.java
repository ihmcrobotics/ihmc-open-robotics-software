package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.ConvectiveTermCalculator;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.PointJacobianConvectiveTermCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseDynamicsOptimizationControlModule
{
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CentroidalMomentumHandler centroidalMomentumHandler;
   private final PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator;
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final InverseDynamicsQPSolver qpSolver;
   private final InverseDynamicsMotionQPInput motionQPInput;
   private final ExternalWrenchHandler externalWrenchHandler;
   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final int numberOfDoFs;

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();
   private final PointJacobianConvectiveTermCalculator pointJacobianConvectiveTermCalculator;
   private final PointJacobian pointJacobian = new PointJacobian();
   private final TIntArrayList indicesIntoCompactBlock = new TIntArrayList();
   private final LinkedHashMap<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();

   private final FrameVector pPointVelocity = new FrameVector();
   private final DenseMatrix64F tempPPointMatrixVelocity = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   private final double controlDT;
   private final OneDoFJoint[] oneDoFJoints;
   private final DenseMatrix64F qDDotMinMatrix, qDDotMaxMatrix;

   private final Map<OneDoFJoint, DoubleYoVariable> jointMaximumAccelerations = new HashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointMinimumAccelerations = new HashMap<>();

   public InverseDynamicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         YoVariableRegistry parentRegistry)
   {
      controlDT = toolbox.getControlDT();
      jointsToOptimizeFor = momentumOptimizationSettings.getJointsToOptimizeFor();
      oneDoFJoints = ScrewTools.filterJoints(jointsToOptimizeFor, OneDoFJoint.class);

      InverseDynamicsJoint rootJoint = toolbox.getRobotRootJoint();
      ReferenceFrame centerOfMassFrame = toolbox.getCenterOfMassFrame();

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      int rhoSize = WholeBodyControlCoreToolbox.rhoSize;
      int maxNPointsPerPlane = WholeBodyControlCoreToolbox.nContactPointsPerContactableBody;
      int maxNSupportVectors = WholeBodyControlCoreToolbox.nBasisVectorsPerContactPoint;

      double gravityZ = toolbox.getGravityZ();
      double wRho = momentumOptimizationSettings.getRhoPlaneContactRegularization();
      double wRhoSmoother = momentumOptimizationSettings.getRateOfChangeOfRhoPlaneContactRegularization();
      double wRhoPenalizer = momentumOptimizationSettings.getPenalizerOfRhoPlaneContactRegularization();

      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      geometricJacobianHolder = toolbox.getGeometricJacobianHolder();
      centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, registry);
      wrenchMatrixCalculator = new PlaneContactWrenchMatrixCalculator(centerOfMassFrame, rhoSize, maxNPointsPerPlane, maxNSupportVectors, wRho, wRhoSmoother, 
            wRhoPenalizer, contactablePlaneBodies, registry);
      motionQPInput = new InverseDynamicsMotionQPInput(numberOfDoFs);
      externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint, contactablePlaneBodies);
      privilegedConfigurationHandler = new JointPrivilegedConfigurationHandler(jointsToOptimizeFor, registry);

      pointJacobianConvectiveTermCalculator = new PointJacobianConvectiveTermCalculator(twistCalculator);

      for (InverseDynamicsJoint joint : jointsToOptimizeFor)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsToOptimizeFor, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         columnsForJoints.put(joint, indices);
      }

      qDDotMinMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      qDDotMaxMatrix = new DenseMatrix64F(numberOfDoFs, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         jointMaximumAccelerations.put(joint, new DoubleYoVariable("qdd_max_qp_" + joint.getName(), registry));
         jointMinimumAccelerations.put(joint, new DoubleYoVariable("qdd_min_qp_" + joint.getName(), registry));
      }

      qpSolver = new InverseDynamicsQPSolver(numberOfDoFs, rhoSize, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      centroidalMomentumHandler.compute();
      externalWrenchHandler.reset();
   }

   public MomentumModuleSolution compute() throws MomentumControlModuleException
   {
      computePrivilegedJointAccelerations();
      wrenchMatrixCalculator.computeMatrices();
      if (SETUP_RHO_TASKS)
         setupRhoTasks();
      setupWrenchesEquilibriumConstraint();
      computeJointAccelerationLimits();
      qpSolver.setMinJointAccelerations(qDDotMinMatrix);
      qpSolver.setMaxJointAccelerations(qDDotMaxMatrix);
      qpSolver.setActiveRhos(wrenchMatrixCalculator.getActiveRhos());

      NoConvergenceException noConvergenceException = null;

      try
      {
         qpSolver.solve();
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
      }

      DenseMatrix64F qDDotSolution = qpSolver.getJointAccelerations();
      DenseMatrix64F rhoSolution = qpSolver.getRhos();

      Map<RigidBody, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenches(rhoSolution);
      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, qDDotSolution);

      SpatialForceVector centroidalMomentumRateSolution = centroidalMomentumHandler.getCentroidalMomentumRate();
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBody> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();
      MomentumModuleSolution momentumModuleSolution = new MomentumModuleSolution(jointsToOptimizeFor, qDDotSolution, centroidalMomentumRateSolution,
            externalWrenchSolution, rigidBodiesWithExternalWrench);

      if (noConvergenceException != null)
      {
         throw new MomentumControlModuleException(noConvergenceException, momentumModuleSolution);
      }

      return momentumModuleSolution;
   }

   private void computeJointAccelerationLimits()
   {
      CommonOps.fill(qDDotMinMatrix, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDDotMaxMatrix, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         int index = columnsForJoints.get(joint)[0];
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();
         
         double qDDotMin = Double.NEGATIVE_INFINITY;
         double qDDotMax = Double.POSITIVE_INFINITY;

         if (Double.isFinite(jointLimitLower))
         {
            double qDotMin = (jointLimitLower - joint.getQ()) / controlDT;
            qDDotMin = (qDotMin - joint.getQd()) / controlDT;
            qDDotMinMatrix.set(index, 0, qDDotMin);
         }
         if (Double.isFinite(jointLimitUpper))
         {
            double qDotMax = (jointLimitUpper - joint.getQ()) / controlDT;
            qDDotMax = (qDotMax - joint.getQd()) / controlDT;
            qDDotMaxMatrix.set(index, 0, qDDotMax);
         }

         jointMinimumAccelerations.get(joint).set(qDDotMin);
         jointMaximumAccelerations.get(joint).set(qDDotMax);
      }
   }

   private void computePrivilegedJointAccelerations()
   {
      if (privilegedConfigurationHandler.isEnabled())
      {
         privilegedConfigurationHandler.computePrivilegedJointAccelerations();
         OneDoFJoint[] joints = privilegedConfigurationHandler.getJoints();
         DenseMatrix64F privilegedJointAccelerations = privilegedConfigurationHandler.getPrivilegedJointAccelerations();
         DenseMatrix64F weight = privilegedConfigurationHandler.getWeight();
         DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

         int taskSize = privilegedJointAccelerations.getNumRows();

         tempSelectionMatrix.reshape(taskSize, numberOfDoFs);
         compactBlockToFullBlock(joints, selectionMatrix, tempSelectionMatrix);

         qpSolver.projectPrivilegedJointAccelerationsInNullspaceOfPreviousTasks(tempSelectionMatrix, privilegedJointAccelerations, weight);
      }
   }

   private void setupRhoTasks()
   {
      DenseMatrix64F rhoPrevious = wrenchMatrixCalculator.getRhoPrevious();
      DenseMatrix64F wRhoSmoother = wrenchMatrixCalculator.getWRhoSmoother();
      qpSolver.addRhoTask(rhoPrevious, wRhoSmoother);

      DenseMatrix64F rhoPreviousAverage = wrenchMatrixCalculator.getRhoPreviousAverage();
      DenseMatrix64F wRhoPenalizer = wrenchMatrixCalculator.getWRhoPenalizer();
      qpSolver.addRhoTask(rhoPreviousAverage, wRhoPenalizer);
   }

   private void setupWrenchesEquilibriumConstraint()
   {
      DenseMatrix64F centroidalMomentumMatrix = centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getQRho();
      DenseMatrix64F convectiveTerm = centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm();
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      qpSolver.setupWrenchesEquilibriumConstraint(centroidalMomentumMatrix, rhoJacobian, convectiveTerm, additionalExternalWrench, gravityWrench);
   }

   public void submitInverseDynamicsCommand(InverseDynamicsCommand<?> command)
   {
      switch (command.getCommandType())
      {
      case TASKSPACE:
         submitSpatialAccelerationCommand((SpatialAccelerationCommand) command);
         return;
      case POINT:
         submitPointAccelerationCommand((PointAccelerationCommand) command);
         return;
      case JOINTSPACE:
         submitJointspaceAccelerationCommand((JointspaceAccelerationCommand) command);
         return;
      case MOMENTUM:
         submitMomentumRateCommand((MomentumRateCommand) command);
         return;
      case PRIVILEGED_CONFIGURATION:
         submitPrivilegedConfigurationCommand((PrivilegedConfigurationCommand) command);
         return;
      case EXTERNAL_WRENCH:
         submitExternalWrenchCommand((ExternalWrenchCommand) command);
         return;
      case PLANE_CONTACT_STATE:
         submitPlaneContactStateCommand((PlaneContactStateCommand) command);
         return;
      case COMMAND_LIST:
         submitInverseDynamicsCommandList((InverseDynamicsCommandList) command);
         return;
      default:
         throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
      }
   }

   private void submitInverseDynamicsCommandList(InverseDynamicsCommandList command)
   {
      for (int i = 0; i < command.getNumberOfCommands(); i++)
         submitInverseDynamicsCommand(command.getCommand(i));
   }

   private void submitSpatialAccelerationCommand(SpatialAccelerationCommand command)
   {
      DenseMatrix64F selectionMatrix = command.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return;

      motionQPInput.reshape(taskSize);
      motionQPInput.setIsMotionConstraint(!command.getHasWeight());
      if (command.getHasWeight())
      {
         motionQPInput.setUseWeightScalar(true);
         motionQPInput.setWeight(command.getWeight());
      }

      SpatialAccelerationVector spatialAcceleration = command.getSpatialAcceleration();
      RigidBody base = command.getBase();
      RigidBody endEffector = command.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, spatialAcceleration.getExpressedInFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      // Compute the task Jacobian: J = S * J
      tempTaskJacobian.reshape(taskSize, jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), tempTaskJacobian);
      compactBlockToFullBlock(jacobian.getJointsInOrder(), tempTaskJacobian, motionQPInput.taskJacobian);

      // Compute the task objective: p = S * ( TDot - JDot qDot )
      convectiveTermCalculator.computeJacobianDerivativeTerm(jacobian, convectiveTerm);
      convectiveTerm.getMatrix(convectiveTermMatrix, 0);
      spatialAcceleration.getMatrix(tempTaskObjective, 0);
      CommonOps.subtractEquals(tempTaskObjective, convectiveTermMatrix);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInput.taskObjective);

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitJointspaceAccelerationCommand(JointspaceAccelerationCommand command)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(command.getJoints());

      if (taskSize == 0)
         return;

      motionQPInput.reshape(taskSize);
      motionQPInput.setIsMotionConstraint(!command.getHasWeight());
      if (command.getHasWeight())
      {
         motionQPInput.setUseWeightScalar(true);
         motionQPInput.setWeight(command.getWeight());
      }

      motionQPInput.taskJacobian.zero();

      int row = 0;
      for (int jointIndex = 0; jointIndex < command.getNumberOfJoints(); jointIndex++)
      {
         InverseDynamicsJoint joint = command.getJoint(jointIndex);
         int[] columns = columnsForJoints.get(joint);
         if (columns == null)
            return;
         for (int column : columns)
            motionQPInput.taskJacobian.set(row, column, 1.0);

         CommonOps.insert(command.getDesiredAcceleration(jointIndex), motionQPInput.taskObjective, row, 0);
      }

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitPointAccelerationCommand(PointAccelerationCommand command)
   {
      DenseMatrix64F selectionMatrix = command.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return;

      motionQPInput.reshape(taskSize);
      motionQPInput.setIsMotionConstraint(!command.getHasWeight());
      if (command.getHasWeight())
      {
         motionQPInput.setUseWeightScalar(true);
         motionQPInput.setWeight(command.getWeight());
      }

      RigidBody base = command.getBase();
      RigidBody endEffector = command.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      FramePoint bodyFixedPoint = command.getContactPoint();
      FrameVector desiredAccelerationWithRespectToBase = command.getDesiredAcceleration();

      pointJacobian.set(jacobian, bodyFixedPoint);
      pointJacobian.compute();

      desiredAccelerationWithRespectToBase.changeFrame(jacobian.getBaseFrame());

      DenseMatrix64F pointJacobianMatrix = pointJacobian.getJacobianMatrix();

      tempTaskJacobian.reshape(selectionMatrix.getNumRows(), pointJacobianMatrix.getNumCols());
      CommonOps.mult(selectionMatrix, pointJacobianMatrix, tempTaskJacobian);
      compactBlockToFullBlock(jacobian.getJointsInOrder(), tempTaskJacobian, motionQPInput.taskJacobian);

      pointJacobianConvectiveTermCalculator.compute(pointJacobian, pPointVelocity);
      pPointVelocity.scale(-1.0);
      pPointVelocity.add(desiredAccelerationWithRespectToBase);
      MatrixTools.setDenseMatrixFromTuple3d(tempPPointMatrixVelocity, pPointVelocity.getVector(), 0, 0);
      CommonOps.mult(selectionMatrix, tempPPointMatrixVelocity, motionQPInput.taskObjective);

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitMomentumRateCommand(MomentumRateCommand command)
   {
      DenseMatrix64F selectionMatrix = command.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return;

      motionQPInput.reshape(taskSize);
      motionQPInput.setUseWeightScalar(false);
      motionQPInput.setIsMotionConstraint(false);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      command.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      CommonOps.mult(selectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, selectionMatrix, motionQPInput.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      CommonOps.mult(selectionMatrix, centroidalMomentumMatrix, motionQPInput.taskJacobian);

      DenseMatrix64F momemtumRate = command.getMomentumRate();
      DenseMatrix64F convectiveTerm = centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm();

      // Compute the task objective: p = S * ( hDot - ADot qDot )
      CommonOps.subtract(momemtumRate, convectiveTerm, tempTaskObjective);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInput.taskObjective);

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      privilegedConfigurationHandler.submitPrivilegedConfigurationCommand(command);
   }

   private void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.setPlaneContactStateCommand(command);
   }

   private void submitExternalWrenchCommand(ExternalWrenchCommand command)
   {
      RigidBody rigidBody = command.getRigidBody();
      Wrench wrench = command.getExternalWrench();
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   private void compactBlockToFullBlock(InverseDynamicsJoint[] joints, DenseMatrix64F compactMatrix, DenseMatrix64F fullMatrix)
   {
      fullMatrix.zero();

      for (int index = 0; index < joints.length; index++)
      {
         InverseDynamicsJoint joint = joints[index];
         indicesIntoCompactBlock.reset();
         ScrewTools.computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            return;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }
   }

   public void setMinRho(double rhoMin)
   {
      qpSolver.setMinRho(rhoMin);
   }
}
