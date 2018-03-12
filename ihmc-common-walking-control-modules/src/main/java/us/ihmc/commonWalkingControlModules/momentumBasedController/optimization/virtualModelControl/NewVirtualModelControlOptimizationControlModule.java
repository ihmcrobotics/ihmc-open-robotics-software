package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ExternalWrenchHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce.NewGroundContactForceQPSolver;
import us.ihmc.commonWalkingControlModules.virtualModelControl.NewVirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;
import java.util.Map;

public class NewVirtualModelControlOptimizationControlModule
{
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExternalWrenchHandler externalWrenchHandler;
   private final SpatialForceVector centroidalMomentumRateSolution = new SpatialForceVector();

   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   private final NewVirtualModelControlSolution virtualModelControlSolution = new NewVirtualModelControlSolution();

   private final WrenchMatrixCalculator wrenchMatrixCalculator;

   private final YoDouble rhoMin = new YoDouble("rhoMinGCF", registry);

   private final BasisVectorVisualizer basisVectorVisualizer;

   private final NewGroundContactForceQPSolver qpSolver;

   private final MotionQPInput motionQPInput;

   private final DenseMatrix64F identityMatrix = CommonOps.identity(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
   private final DenseMatrix64F fullMomentumObjective = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F totalWrench = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final ReferenceFrame centerOfMassFrame;

   public NewVirtualModelControlOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
      this.centerOfMassFrame = toolbox.getCenterOfMassFrame();

      ControllerCoreOptimizationSettings optimizationSettings = toolbox.getOptimizationSettings();
      int rhoSize = optimizationSettings.getRhoSize();
      motionQPInput = new MotionQPInput(rhoSize);

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer = new BasisVectorVisualizer("ContactBasisVectors", rhoSize, 1.0, toolbox.getYoGraphicsListRegistry(), registry);
      else
         basisVectorVisualizer = null;

      boolean hasFloatingBase = toolbox.getRootJoint() != null;
      rhoMin.set(optimizationSettings.getRhoMin());
      ActiveSetQPSolver activeSetQPSolver = optimizationSettings.getActiveSetQPSolver();
      qpSolver = new NewGroundContactForceQPSolver(activeSetQPSolver, rhoSize, hasFloatingBase, registry);
      qpSolver.setMinRho(optimizationSettings.getRhoMin());

      externalWrenchHandler = new ExternalWrenchHandler(toolbox.getGravityZ(), centerOfMassFrame, toolbox.getTotalRobotMass(), toolbox.getContactablePlaneBodies());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      externalWrenchHandler.reset();
   }

   public NewVirtualModelControlSolution compute() throws NewVirtualModelControlModuleException
   {
      wrenchMatrixCalculator.computeMatrices();
      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer.visualize(wrenchMatrixCalculator.getBasisVectors(), wrenchMatrixCalculator.getBasisVectorsOrigin());
      qpSolver.setRhoRegularizationWeight(wrenchMatrixCalculator.getRhoWeightMatrix());
      if (SETUP_RHO_TASKS)
         setupRhoTasks();

      qpSolver.setMinRho(rhoMin.getDoubleValue());
      qpSolver.setMaxRho(wrenchMatrixCalculator.getRhoMaxMatrix());

      setupWrenchesEquilibriumConstraint();

      // todo add rho warm start

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
            PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         noConvergenceException = e;
      }

      DenseMatrix64F rhoSolution = qpSolver.getRhos();

      Map<RigidBody, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      SpatialForceVector centroidalMomentumRateSolution = computeCentroidalMomentumRateSolution(rhoSolution);
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBody> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();

      virtualModelControlSolution.setExternalWrenchSolution(rigidBodiesWithExternalWrench, externalWrenchSolution);
      virtualModelControlSolution.setCentroidalMomentumRateSolution(centroidalMomentumRateSolution);

      if (noConvergenceException != null)
      {
         throw new NewVirtualModelControlModuleException(noConvergenceException, virtualModelControlSolution);
      }

      return virtualModelControlSolution;
   }


   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      boolean success = convertMomentumRateCommand(command, motionQPInput);
      if (success)
         qpSolver.addMomentumInput(motionQPInput);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.submitPlaneContactStateCommand(command);
   }

   public void submitExternalWrench(RigidBody rigidBody, Wrench wrench)
   {
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   /**
    * Converts a {@link MomentumRateCommand} into a {@link MotionQPInput}.
    *
    * @return true if the command was successfully converted.
    */
   private boolean convertMomentumRateCommand(MomentumRateCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialForceVector.SIZE);
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

   private SpatialForceVector computeCentroidalMomentumRateSolution(DenseMatrix64F rhoSolution)
   {
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();

      CommonOps.mult(rhoJacobian, rhoSolution, totalWrench);
      CommonOps.addEquals(totalWrench, additionalExternalWrench);
      CommonOps.addEquals(totalWrench, gravityWrench);

      centroidalMomentumRateSolution.set(centerOfMassFrame, totalWrench);

      return centroidalMomentumRateSolution;
   }

   private void setupRhoTasks()
   {
      DenseMatrix64F rhoPrevious = wrenchMatrixCalculator.getRhoPreviousMatrix();
      DenseMatrix64F rhoRateWeight = wrenchMatrixCalculator.getRhoRateWeightMatrix();
      qpSolver.addRhoTask(rhoPrevious, rhoRateWeight);

      DenseMatrix64F copJacobian = wrenchMatrixCalculator.getCopJacobianMatrix();

      DenseMatrix64F previousCoP = wrenchMatrixCalculator.getPreviousCoPMatrix();
      DenseMatrix64F copRateWeight = wrenchMatrixCalculator.getCopRateWeightMatrix();
      qpSolver.addRhoTask(copJacobian, previousCoP, copRateWeight);

      DenseMatrix64F desiredCoP = wrenchMatrixCalculator.getDesiredCoPMatrix();
      DenseMatrix64F desiredCoPWeight = wrenchMatrixCalculator.getDesiredCoPWeightMatrix();
      qpSolver.addRhoTask(copJacobian, desiredCoP, desiredCoPWeight);
   }
}
