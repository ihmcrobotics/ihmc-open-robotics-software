package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.Map;

public class GroundContactForceOptimizationControlModule
{
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final WrenchMatrixCalculator wrenchMatrixCalculator;

   private final DoubleYoVariable rhoMin = new DoubleYoVariable("rhoMinGCFOptimization", registry);

   private final BasisVectorVisualizer basisVectorVisualizer;

   private final GroundContactForceQPSolver qpSolver;
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable("hasNotConvergedCounts", registry);

   public GroundContactForceOptimizationControlModule(WrenchMatrixCalculator wrenchMatrixCalculator, MomentumOptimizationSettings momentumOptimizationSettings,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.wrenchMatrixCalculator = wrenchMatrixCalculator;
      int rhoSize = WholeBodyControlCoreToolbox.rhoSize;

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer = new BasisVectorVisualizer("ContactBasisVectors", rhoSize, 1.0, yoGraphicsListRegistry, registry);
      else
         basisVectorVisualizer = null;

      rhoMin.set(momentumOptimizationSettings.getRhoMin());
      qpSolver = new GroundContactForceQPSolver(rhoSize, registry);
      qpSolver.setMinRho(momentumOptimizationSettings.getRhoMin());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
   }

   public Map<RigidBody, Wrench> compute(DenseMatrix64F momentumObjective, DenseMatrix64F momentumJacobian, DenseMatrix64F momentumWeight) throws NoConvergenceException
   {
      wrenchMatrixCalculator.computeMatrices();

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

      return wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
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
