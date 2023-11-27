package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.matrixlib.MatrixTools;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.*;

/**
 * Static whole-body force distribution calculator using the constraints from {@link CenterOfMassStabilityMarginOptimizationModule},
 * but instead of varying CoM position and solving with an LP, it keeps the CoM fixed and solves with a QP.
 */
public class MultiContactForceDistributionCalculator
{
   private final CenterOfMassStabilityMarginOptimizationModule comOptimizationModule;
   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();
   private final double mg;

   /* Solver solution for x_force = [f_0x, ... f_nz] */
   private final DMatrixRMaj solutionForce = new DMatrixRMaj(0);
   /* Solver solution for x_rho = [rho_0, ..., rho_n] */
   private final DMatrixRMaj solutionRho = new DMatrixRMaj(0);

   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0);

   /* Number of decision variables in x_force = [f_0x, ... f_nz] */
   private int forceDecisionVariables;
   /* Number of decision variables in x_rho = [rho_0, ..., rho_n] */
   private int rhoDecisionVariables;

   /* Equality matrices for Aeq x_force = beq, where x_force = [f_0x, f_0y, ..., c_x, c_y] are the force-based decision variables */
   private final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0);

   /* Equality matrix for Aeq_rho x_rho = b, where x_rho = [rho_0, ..., rho_n] */
   private final DMatrixRMaj Aeq_rho = new DMatrixRMaj(0);

   /* Conversion from x_force = [f_0x, ... f_nz] to x_rho = [rho_0, ..., rho_n] , and x_rho > =0 */
   private final DMatrixRMaj rhoToForce = new DMatrixRMaj(0);

   /* Actuation constraint expressed in terms of x_rho */
   private final DMatrixRMaj Aact_rho = new DMatrixRMaj(0);

   /* Inequality matrices for Ain_rho x_rho <= bin where x_rho = [rho_0, ..., rho_n] */
   private final DMatrixRMaj Ain_rho = new DMatrixRMaj(0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0);

   private boolean feasibilityMode = false;

   public MultiContactForceDistributionCalculator(double robotMass)
   {
      comOptimizationModule = new CenterOfMassStabilityMarginOptimizationModule(robotMass);
      mg = robotMass * GRAVITY;
   }

   public boolean solve(WholeBodyContactStateInterface contactState, double comX, double comY)
   {
      comOptimizationModule.updateContactState(contactState);

      /* Compute nominal equality constraint to enforce static equilibrium */

      forceDecisionVariables = LINEAR_DIMENSIONS * contactState.getNumberOfContactPoints();
      Aeq.reshape(STATIC_EQUILIBRIUM_CONSTRAINTS, forceDecisionVariables);
      beq.reshape(STATIC_EQUILIBRIUM_CONSTRAINTS, 1);

      MatrixTools.setMatrixBlock(Aeq, 0, 0, comOptimizationModule.Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), 1.0);

      beq.set(2, 0, mg);
      beq.set(3, 0, comY * mg);
      beq.set(4, 0, -comX * mg);

      /* Compute map from positive x to nominal x */

      rhoDecisionVariables = NUM_BASIS_VECTORS * contactState.getNumberOfContactPoints();
      rhoToForce.reshape(forceDecisionVariables, rhoDecisionVariables);

      MatrixTools.setMatrixBlock(rhoToForce, 0, 0, comOptimizationModule.rhoToForce, 0, 0, rhoToForce.getNumRows(), rhoToForce.getNumCols(), 1.0);

      /* Compute constraint matrices */
      CommonOps_DDRM.mult(Aeq, rhoToForce, Aeq_rho);

      /* Compute inequality matrices */
      DMatrixRMaj A_actuation = contactState.getActuationConstraintMatrix();
      DMatrixRMaj b_actuation = contactState.getActuationConstraintVector();
      CommonOps_DDRM.mult(A_actuation, rhoToForce, Aact_rho);

      Ain_rho.reshape(rhoDecisionVariables + A_actuation.getNumRows(), rhoDecisionVariables);
      bin.reshape(rhoDecisionVariables + b_actuation.getNumRows(), 1);

      for (int basisIdx = 0; basisIdx < rhoDecisionVariables; basisIdx++)
      {
         Ain_rho.set(basisIdx, basisIdx, -1.0);
      }

      MatrixTools.setMatrixBlock(Ain_rho, rhoDecisionVariables, 0, A_actuation, 0, 0, A_actuation.getNumRows(), A_actuation.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, rhoDecisionVariables, 0, b_actuation, 0, 0, b_actuation.getNumRows(), b_actuation.getNumCols(), 1.0);

      /* Compute optimal force distribution */
      qpSolver.clear();
      qpSolver.resetActiveSet();

      qpSolver.setMaxNumberOfIterations(feasibilityMode ? 100 : 500);
      qpSolver.setConvergenceThreshold(feasibilityMode ? 1e-2 : 1e-9);
      qpSolver.setQuadraticCostFunction(quadraticCost, linearCost, 0.0);
      qpSolver.setLinearEqualityConstraints(Aeq_rho, beq);
      qpSolver.setLinearInequalityConstraints(Ain_rho, bin);

      qpSolver.solve(solutionRho);
      boolean success = !MatrixTools.containsNaN(solutionRho);
      if (success)
      {
         CommonOps_DDRM.mult(rhoToForce, solutionRho, solutionForce);
      }

      return success;
   }

   public void getResolvedForce(int contactIdx, Vector3DBasics resolvedForceToPack)
   {
      resolvedForceToPack.setX(solutionForce.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.X.ordinal()));
      resolvedForceToPack.setY(solutionForce.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Y.ordinal()));
      resolvedForceToPack.setZ(solutionForce.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Z.ordinal()));
   }

   public void setAsFeasibilitySolver(boolean feasibilityMode)
   {
      this.feasibilityMode = feasibilityMode;
   }

   public DMatrixRMaj getRhoSolution()
   {
      return solutionRho;
   }
}
