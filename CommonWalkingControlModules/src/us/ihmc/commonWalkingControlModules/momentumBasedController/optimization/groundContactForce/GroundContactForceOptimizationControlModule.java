package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.List;
import java.util.Map;
import java.util.Set;

public class GroundContactForceOptimizationControlModule
{
   private static final boolean DEBUG = true;
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;

   private final DoubleYoVariable rhoMin = new DoubleYoVariable("rhoMinGCFOptimization", registry);

   private final BasisVectorVisualizer basisVectorVisualizer;

   private final GroundContactForceQPSolver qpSolver;
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable("hasNotConvergedCounts", registry);

   private final YoFrameVector desiredLinearMomentumRate;
   private final YoFrameVector desiredAngularMomentumRate;

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();

   private final YoMatrix yoMomentumSelectionMatrix = new YoMatrix("VMCMomentumSelectionMatrix", Wrench.SIZE, Wrench.SIZE, registry);
   private final YoMatrix yoMomentumObjective = new YoMatrix("VMCMomentumObjectiveMatrix", Wrench.SIZE, 1, registry);
   private final YoMatrix yoMomentumWeight = new YoMatrix("VMCMomentumWeightMatrix", Wrench.SIZE, Wrench.SIZE, registry);
   private final DenseMatrix64F momentumSelectionMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F momentumObjective = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F momentumJacobian = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F momentumWeight = new DenseMatrix64F(Wrench.SIZE, 1);

   public GroundContactForceOptimizationControlModule(WrenchMatrixCalculator wrenchMatrixCalculator, List<? extends ContactablePlaneBody> contactablePlaneBodies,
         MomentumOptimizationSettings momentumOptimizationSettings, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.wrenchMatrixCalculator = wrenchMatrixCalculator;
      this.contactablePlaneBodies = contactablePlaneBodies;
      int rhoSize = momentumOptimizationSettings.getRhoSize();

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer = new BasisVectorVisualizer("ContactBasisVectors", rhoSize, 1.0, yoGraphicsListRegistry, registry);
      else
         basisVectorVisualizer = null;

      if (DEBUG)
      {
         desiredLinearMomentumRate = new YoFrameVector("desiredLinearMomentumRateToQP", null, registry);
         desiredAngularMomentumRate = new YoFrameVector("desiredAngularMomentumRateToQP", null, registry);
      }
      else
      {
         desiredLinearMomentumRate = null;
         desiredAngularMomentumRate = null;
      }

      rhoMin.set(momentumOptimizationSettings.getRhoMin());
      qpSolver = new GroundContactForceQPSolver(rhoSize, registry);
      qpSolver.setMinRho(momentumOptimizationSettings.getRhoMin());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
   }

   private Map<RigidBody, Wrench> solutionWrenches;
   public void compute(Map<RigidBody, Wrench> groundReactionWrenchesToPack) throws NoConvergenceException
   {
      qpSolver.setRhoRegularizationWeight(wrenchMatrixCalculator.getRhoWeightMatrix());
      qpSolver.addRegularization();

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer.visualize(wrenchMatrixCalculator.getBasisVectors(), wrenchMatrixCalculator.getBasisVectorsOrigin());

      if (SETUP_RHO_TASKS)
         setupRhoTasks();

      qpSolver.setMinRho(rhoMin.getDoubleValue());
      qpSolver.addMomentumTask(momentumJacobian, momentumObjective, momentumWeight);

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
            PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         noConvergenceException = e;
      }

      DenseMatrix64F rhoSolution = qpSolver.getRhos();

      if (noConvergenceException != null)
         throw noConvergenceException;

      solutionWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBody rigidBody = contactablePlaneBodies.get(i).getRigidBody();
         Wrench solutionWrench = solutionWrenches.get(rigidBody);

         if (groundReactionWrenchesToPack.containsKey(rigidBody))
            groundReactionWrenchesToPack.get(rigidBody).set(solutionWrench);
         else
            groundReactionWrenchesToPack.put(rigidBody, solutionWrench);
      }
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

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      momentumRateCommand.set(command);
      momentumRateCommand.setWeights(command.getWeightVector());
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.submitPlaneContactStateCommand(command);
   }

   public void submitMomentumSelectionMatrix(DenseMatrix64F momentumSelectionMatrix)
   {
      this.momentumSelectionMatrix.set(momentumSelectionMatrix);
      yoMomentumSelectionMatrix.set(momentumSelectionMatrix);
   }

   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F fullMomentumObjective = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public void processMomentumRateCommand(DenseMatrix64F additionalWrench)
   {
      wrenchMatrixCalculator.computeMatrices();

      int taskSize = momentumSelectionMatrix.getNumRows();
      momentumObjective.reshape(taskSize, 1);
      momentumWeight.reshape(taskSize, taskSize);

      if (taskSize == 0)
         return;

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      momentumRateCommand.getWeightMatrix(tempTaskWeight);
      CommonOps.mult(momentumSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, momentumSelectionMatrix, momentumWeight);
      yoMomentumWeight.set(momentumWeight);
      
      // Compute the task Jacobian: J = S * A
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();
      momentumJacobian.reshape(taskSize, rhoJacobian.numCols);
      CommonOps.mult(momentumSelectionMatrix, rhoJacobian, momentumJacobian);

      // Compute the task objective: p = S * (hDot - sum W_user - W_g)
      DenseMatrix64F momentumRate = momentumRateCommand.getMomentumRate();
      CommonOps.subtract(momentumRate, additionalWrench, fullMomentumObjective);
      CommonOps.mult(momentumSelectionMatrix, fullMomentumObjective, momentumObjective);
      yoMomentumObjective.set(momentumObjective);
      
      // get the selected objective back out
      CommonOps.multTransA(momentumSelectionMatrix, momentumObjective, fullMomentumObjective);

      if (DEBUG)
      {
         desiredLinearMomentumRate.set(fullMomentumObjective.get(3), fullMomentumObjective.get(4), fullMomentumObjective.get(5));
         desiredAngularMomentumRate.set(fullMomentumObjective.get(0), fullMomentumObjective.get(1), fullMomentumObjective.get(2));
      }
   }

   public DenseMatrix64F getMomentumObjective()
   {
      return fullMomentumObjective;
   }
}
