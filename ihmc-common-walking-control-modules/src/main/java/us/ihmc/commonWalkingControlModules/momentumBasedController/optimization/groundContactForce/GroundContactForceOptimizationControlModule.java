package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GroundContactForceOptimizationControlModule
{
   private static final boolean DEBUG = true;
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;

   private final YoDouble rhoMin = new YoDouble("rhoMinGCFOptimization", registry);

   private final BasisVectorVisualizer basisVectorVisualizer;

   private final GroundContactForceQPSolver qpSolver;
   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   private final YoFrameVector3D desiredLinearMomentumRate;
   private final YoFrameVector3D desiredAngularMomentumRate;

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();

   private final YoMatrix yoMomentumSelectionMatrix = new YoMatrix("VMCMomentumSelectionMatrix", Wrench.SIZE, Wrench.SIZE, registry);
   private final YoMatrix yoMomentumObjective = new YoMatrix("VMCMomentumObjectiveMatrix", Wrench.SIZE, 1, registry);
   private final YoMatrix yoMomentumWeight = new YoMatrix("VMCMomentumWeightMatrix", Wrench.SIZE, Wrench.SIZE, registry);
   private final DMatrixRMaj momentumSelectionMatrix = new DMatrixRMaj(Wrench.SIZE, 1);
   private final DMatrixRMaj momentumObjective = new DMatrixRMaj(Wrench.SIZE, 1);
   private final DMatrixRMaj momentumJacobian = new DMatrixRMaj(Wrench.SIZE, 1);
   private final DMatrixRMaj momentumWeight = new DMatrixRMaj(Wrench.SIZE, 1);

   private final DMatrixRMaj zeroObjective = new DMatrixRMaj(0, 0);

   public GroundContactForceOptimizationControlModule(WrenchMatrixCalculator wrenchMatrixCalculator, List<? extends ContactablePlaneBody> contactablePlaneBodies,
         ControllerCoreOptimizationSettings optimizationSettings, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.wrenchMatrixCalculator = wrenchMatrixCalculator;
      this.contactablePlaneBodies = contactablePlaneBodies;
      int rhoSize = optimizationSettings.getRhoSize();

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer = new BasisVectorVisualizer("ContactBasisVectors", rhoSize, 1.0, yoGraphicsListRegistry, registry);
      else
         basisVectorVisualizer = null;

      if (DEBUG)
      {
         desiredLinearMomentumRate = new YoFrameVector3D("desiredLinearMomentumRateToQP", null, registry);
         desiredAngularMomentumRate = new YoFrameVector3D("desiredAngularMomentumRateToQP", null, registry);
      }
      else
      {
         desiredLinearMomentumRate = null;
         desiredAngularMomentumRate = null;
      }

      rhoMin.set(optimizationSettings.getRhoMin());
      ActiveSetQPSolver activeSetQPSolver = optimizationSettings.getActiveSetQPSolver();
      qpSolver = new GroundContactForceQPSolver(activeSetQPSolver, rhoSize, registry);
      qpSolver.setMinRho(optimizationSettings.getRhoMin());

      zeroObjective.reshape(wrenchMatrixCalculator.getCopTaskSize(), 1);
      zeroObjective.zero();

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
   }

   private Map<RigidBodyBasics, Wrench> solutionWrenches;
   public void compute(Map<RigidBodyBasics, Wrench> groundReactionWrenchesToPack) throws NoConvergenceException
   {
      qpSolver.setRhoRegularizationWeight(wrenchMatrixCalculator.getRhoWeightMatrix());
      qpSolver.addRegularization();

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer.visualize(wrenchMatrixCalculator.getBasisVectors(), wrenchMatrixCalculator.getBasisVectorsOrigin());

      if (SETUP_RHO_TASKS)
         setupRhoTasks();

      qpSolver.setMinRho(rhoMin.getDoubleValue());
      qpSolver.addMotionTask(momentumJacobian, momentumObjective, momentumWeight);

      NoConvergenceException noConvergenceException = null;

      // use the force optimization algorithm
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

      DMatrixRMaj rhoSolution = qpSolver.getRhos();

      if (noConvergenceException != null)
         throw noConvergenceException;

      solutionWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactablePlaneBodies.get(i).getRigidBody();
         Wrench solutionWrench = solutionWrenches.get(rigidBody);

         if (groundReactionWrenchesToPack.containsKey(rigidBody))
            groundReactionWrenchesToPack.get(rigidBody).setIncludingFrame(solutionWrench);
         else
            groundReactionWrenchesToPack.put(rigidBody, solutionWrench);
      }
   }

   private void setupRhoTasks()
   {
      DMatrixRMaj rhoPrevious = wrenchMatrixCalculator.getRhoPreviousMatrix();
      DMatrixRMaj rhoRateWeight = wrenchMatrixCalculator.getRhoRateWeightMatrix();
      qpSolver.addRhoTask(rhoPrevious, rhoRateWeight);

      DMatrixRMaj copRegularizationWeight = wrenchMatrixCalculator.getCoPRegularizationWeight();
      DMatrixRMaj copRegularizationJacobian = wrenchMatrixCalculator.getCoPRegularizationJacobian();
      qpSolver.addRhoTask(copRegularizationJacobian, zeroObjective, copRegularizationWeight);

      DMatrixRMaj copRateRegularizationWeight = wrenchMatrixCalculator.getCoPRateRegularizationWeight();
      DMatrixRMaj copRateRegularizationJacobian = wrenchMatrixCalculator.getCoPRateRegularizationJacobian();
      qpSolver.addRhoTask(copRateRegularizationJacobian, zeroObjective, copRateRegularizationWeight);
   }

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      momentumRateCommand.set(command);
      momentumRateCommand.setWeights(command.getWeightMatrix());
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.submitPlaneContactStateCommand(command);
   }

   public void submitMomentumSelectionMatrix(DMatrixRMaj momentumSelectionMatrix)
   {
      this.momentumSelectionMatrix.set(momentumSelectionMatrix);
      yoMomentumSelectionMatrix.set(momentumSelectionMatrix);
   }

   private final DMatrixRMaj tempTaskWeight = new DMatrixRMaj(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final DMatrixRMaj tempTaskWeightSubspace = new DMatrixRMaj(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final DMatrixRMaj fullMomentumObjective = new DMatrixRMaj(SpatialAcceleration.SIZE, 1);

   public void processMomentumRateCommand(DMatrixRMaj additionalWrench)
   {
      wrenchMatrixCalculator.computeMatrices(null);

      int taskSize = momentumSelectionMatrix.getNumRows();
      momentumObjective.reshape(taskSize, 1);
      momentumWeight.reshape(taskSize, taskSize);

      if (taskSize == 0)
         return;

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
      momentumRateCommand.getWeightMatrix(tempTaskWeight);
      CommonOps_DDRM.mult(momentumSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps_DDRM.multTransB(tempTaskWeightSubspace, momentumSelectionMatrix, momentumWeight);
      yoMomentumWeight.set(momentumWeight);
      
      // Compute the task Jacobian: J = S * A
      DMatrixRMaj rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();
      momentumJacobian.reshape(taskSize, rhoJacobian.numCols);
      CommonOps_DDRM.mult(momentumSelectionMatrix, rhoJacobian, momentumJacobian);

      // Compute the task objective: p = S * (hDot - sum W_user - W_g)
      DMatrixRMaj momentumRate = momentumRateCommand.getMomentumRate();
      CommonOps_DDRM.subtract(momentumRate, additionalWrench, fullMomentumObjective);
      CommonOps_DDRM.mult(momentumSelectionMatrix, fullMomentumObjective, momentumObjective);
      yoMomentumObjective.set(momentumObjective);
      
      // get the selected objective back out
      CommonOps_DDRM.multTransA(momentumSelectionMatrix, momentumObjective, fullMomentumObjective);

      if (DEBUG)
      {
         desiredLinearMomentumRate.set(fullMomentumObjective.get(3), fullMomentumObjective.get(4), fullMomentumObjective.get(5));
         desiredAngularMomentumRate.set(fullMomentumObjective.get(0), fullMomentumObjective.get(1), fullMomentumObjective.get(2));
      }
   }

   public DMatrixRMaj getMomentumObjective()
   {
      return fullMomentumObjective;
   }
}
