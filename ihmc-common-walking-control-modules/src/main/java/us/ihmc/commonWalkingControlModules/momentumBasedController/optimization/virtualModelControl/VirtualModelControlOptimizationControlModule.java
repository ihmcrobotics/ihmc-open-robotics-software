package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ExternalWrenchHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce.GroundContactForceMomentumQPSolver;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class VirtualModelControlOptimizationControlModule
{
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExternalWrenchHandler externalWrenchHandler;
   private final SpatialForce centroidalMomentumRateSolution = new SpatialForce();

   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   private final YoBoolean useWarmStart = new YoBoolean("useWarmStartInSolver", registry);
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterationsInSolver", registry);

   private final VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();

   private final WrenchMatrixCalculator wrenchMatrixCalculator;

   private final YoDouble rhoMin = new YoDouble("rhoMinGCF", registry);

   private final BasisVectorVisualizer basisVectorVisualizer;

   private final GroundContactForceMomentumQPSolver qpSolver;

   private final QPInput momentumQPInput;

   private final DenseMatrix64F identityMatrix = CommonOps.identity(SpatialForce.SIZE, SpatialForce.SIZE);
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialForce.SIZE, SpatialForce.SIZE);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialForce.SIZE, SpatialForce.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialForce.SIZE, SpatialForce.SIZE);
   private final DenseMatrix64F fullMomentumObjective = new DenseMatrix64F(SpatialForce.SIZE, 1);
   private final DenseMatrix64F totalWrench = new DenseMatrix64F(SpatialForce.SIZE, 1);

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final ReferenceFrame centerOfMassFrame;

   private final DenseMatrix64F zeroObjective = new DenseMatrix64F(0, 0);

   public VirtualModelControlOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
      this.centerOfMassFrame = toolbox.getCenterOfMassFrame();

      ControllerCoreOptimizationSettings optimizationSettings = toolbox.getOptimizationSettings();
      int rhoSize = optimizationSettings.getRhoSize();
      momentumQPInput = new QPInput(SpatialForce.SIZE);

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer = new BasisVectorVisualizer("ContactBasisVectors", rhoSize, 1.0, toolbox.getYoGraphicsListRegistry(), registry);
      else
         basisVectorVisualizer = null;

      boolean hasFloatingBase = toolbox.getRootJoint() != null;
      rhoMin.set(optimizationSettings.getRhoMin());
      ActiveSetQPSolverWithInactiveVariablesInterface activeSetQPSolver = optimizationSettings.getActiveSetQPSolver();
      qpSolver = new GroundContactForceMomentumQPSolver(activeSetQPSolver, rhoSize, hasFloatingBase, registry);
      qpSolver.setMinRho(optimizationSettings.getRhoMin());
      qpSolver.setUseWarmStart(optimizationSettings.useWarmStartInSolver());
      qpSolver.setMaxNumberOfIterations(optimizationSettings.getMaxNumberOfSolverIterations());

      externalWrenchHandler = new ExternalWrenchHandler(toolbox.getGravityZ(), centerOfMassFrame, toolbox.getTotalRobotMass(),
                                                        toolbox.getContactablePlaneBodies());

      useWarmStart.set(optimizationSettings.useWarmStartInSolver());
      maximumNumberOfIterations.set(optimizationSettings.getMaxNumberOfSolverIterations());

      zeroObjective.reshape(wrenchMatrixCalculator.getCopTaskSize(), 1);
      zeroObjective.zero();

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      externalWrenchHandler.reset();
   }

   public VirtualModelControlSolution compute() throws VirtualModelControlModuleException
   {
      wrenchMatrixCalculator.computeMatrices();
      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer.visualize(wrenchMatrixCalculator.getBasisVectors(), wrenchMatrixCalculator.getBasisVectorsOrigin());
      qpSolver.setRhoRegularizationWeight(wrenchMatrixCalculator.getRhoWeightMatrix());
      if (SETUP_RHO_TASKS)
         setupRhoTasks();

      qpSolver.setMinRho(rhoMin.getDoubleValue());
      qpSolver.setMaxRho(wrenchMatrixCalculator.getRhoMaxMatrix());
      qpSolver.setActiveRhos(wrenchMatrixCalculator.getActiveRhoMatrix());

      setupWrenchesEquilibriumConstraint();

      qpSolver.setMaxNumberOfIterations(maximumNumberOfIterations.getIntegerValue());
      if (useWarmStart.getBooleanValue() && wrenchMatrixCalculator.hasContactStateChanged())
      {
         qpSolver.setUseWarmStart(useWarmStart.getBooleanValue());
         qpSolver.notifyResetActiveSet();
      }

      NoConvergenceException noConvergenceException = null;

      try
      {
         qpSolver.solve();
      }
      catch (NoConvergenceException e)
      {

         if (!hasNotConvergedInPast.getBooleanValue())
         {
            e.printStackTrace();
            LogTools.warn("Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         noConvergenceException = e;
      }

      DenseMatrix64F rhoSolution = qpSolver.getRhos();

      Map<RigidBodyBasics, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      SpatialForceReadOnly centroidalMomentumRateSolution = computeCentroidalMomentumRateSolution(rhoSolution);
      Map<RigidBodyBasics, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBodyBasics> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();

      virtualModelControlSolution.setExternalWrenchSolution(rigidBodiesWithExternalWrench, externalWrenchSolution);
      virtualModelControlSolution.setCentroidalMomentumRateSolution(centroidalMomentumRateSolution);

      if (noConvergenceException != null)
      {
         throw new VirtualModelControlModuleException(virtualModelControlSolution);
      }

      return virtualModelControlSolution;
   }

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      boolean success = convertMomentumRateCommand(command, momentumQPInput);
      if (success)
         qpSolver.addMomentumInput(momentumQPInput);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.submitPlaneContactStateCommand(command);
   }

   public void submitCenterOfPressureCommand(CenterOfPressureCommand command)
   {
      wrenchMatrixCalculator.submitCenterOfPressureCommand(command);
   }

   public void submitContactWrenchCommand(ContactWrenchCommand command)
   {
      wrenchMatrixCalculator.submitContactWrenchCommand(command);
   }

   public void submitExternalWrench(RigidBodyBasics rigidBody, WrenchReadOnly wrench)
   {
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void submitOptimizationSettingsCommand(VirtualModelControlOptimizationSettingsCommand command)
   {
      if (command.hasRhoMin())
         rhoMin.set(command.getRhoMin());
      if (command.hasRhoWeight())
         wrenchMatrixCalculator.setRhoWeight(command.getRhoWeight());
      if (command.hasRhoRateWeight())
         wrenchMatrixCalculator.setRhoRateWeight(command.getRhoRateWeight());
      if (command.hasCenterOfPressureWeight())
         wrenchMatrixCalculator.setDesiredCoPWeight(command.getCenterOfPressureWeight());
      if (command.hasCenterOfPressureRateWeight())
         wrenchMatrixCalculator.setCoPRateWeight(command.getCenterOfPressureRateWeight());
      if (command.hasMomentumRateWeight())
         qpSolver.setMomentumRateRegularization(command.getMomentumRateWeight());
      if (command.hasMomentumAccelerationWeight())
         qpSolver.setMomentumAccelerationRegularization(command.getMomentumAccelerationWeight());
   }

   /**
    * Converts a {@link MomentumRateCommand} into a {@link QPInput}.
    *
    * @return true if the command was successfully converted.
    */
   private boolean convertMomentumRateCommand(MomentumRateCommand commandToConvert, QPInput motionQPInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialForce.SIZE, SpatialForce.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialForce.SIZE);
      DiagonalMatrixTools.postMult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);

      // Compute the task Jacobian: J = S * I
      CommonOps.mult(tempSelectionMatrix, identityMatrix, motionQPInputToPack.taskJacobian);

      // Compute the task objective: p = S * hDot
      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);
      angularMomentum.get(0, fullMomentumObjective);
      linearMomentum.get(3, fullMomentumObjective);

      CommonOps.mult(tempSelectionMatrix, fullMomentumObjective, motionQPInputToPack.taskObjective);

      return true;
   }

   private void setupWrenchesEquilibriumConstraint()
   {
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();
      qpSolver.setupWrenchesEquilibriumConstraint(identityMatrix, rhoJacobian, additionalExternalWrench, gravityWrench);
   }

   private SpatialForceReadOnly computeCentroidalMomentumRateSolution(DenseMatrix64F rhoSolution)
   {
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();

      CommonOps.mult(rhoJacobian, rhoSolution, totalWrench);
      CommonOps.addEquals(totalWrench, additionalExternalWrench);
      CommonOps.addEquals(totalWrench, gravityWrench);

      centroidalMomentumRateSolution.setIncludingFrame(centerOfMassFrame, totalWrench);

      return centroidalMomentumRateSolution;
   }

   private void setupRhoTasks()
   {
      DenseMatrix64F rhoPrevious = wrenchMatrixCalculator.getRhoPreviousMatrix();
      DenseMatrix64F rhoRateWeight = wrenchMatrixCalculator.getRhoRateWeightMatrix();
      qpSolver.addRhoTask(rhoPrevious, rhoRateWeight);

      DenseMatrix64F copRegularizationWeight = wrenchMatrixCalculator.getCoPRegularizationWeight();
      DenseMatrix64F copRegularizationJacobian = wrenchMatrixCalculator.getCoPRegularizationJacobian();
      qpSolver.addRhoTask(copRegularizationJacobian, zeroObjective, copRegularizationWeight);

      DenseMatrix64F copRateRegularizationWeight = wrenchMatrixCalculator.getCoPRateRegularizationWeight();
      DenseMatrix64F copRateRegularizationJacobian = wrenchMatrixCalculator.getCoPRateRegularizationJacobian();
      qpSolver.addRhoTask(copRateRegularizationJacobian, zeroObjective, copRateRegularizationWeight);
   }
}
