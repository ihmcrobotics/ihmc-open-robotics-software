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

   private final DMatrixRMaj solutionPos = new DMatrixRMaj(0);
   private final DMatrixRMaj solution = new DMatrixRMaj(0);

   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0);

   /* Number of nominal decision variables: 3 * n_contact_points */
   private int nominalDecisionVariables;
   /* Number of non-negative decision variables: n_basis_vectors */
   private int posDecisionVariables;

   /* Equality matrices for A x = b, where x = [f_0x, f_0y, ..., f_nz] */
   private final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0);

   /* Equality matrix for A x+ = b, where x+ = [rho_0 ... rho_N] */
   private final DMatrixRMaj Aeq_pos = new DMatrixRMaj(0);

   /* Conversion from nominal x, which is x = [f_0x, f_0y, ..., f_nz] to positive x+, which is x+ = [rho_0, rho_1, ..., rho_n] , and x+ > =0 */
   private final DMatrixRMaj posXToNominalX = new DMatrixRMaj(0);

   /* Actuation constraint expressed in terms of x+ */
   private final DMatrixRMaj Aact_pos = new DMatrixRMaj(0);

   /* Inequality matrix for Ain x+ < bin, where x+ = [rho_0 ... rho_N] */
   private final DMatrixRMaj Ain_pos = new DMatrixRMaj(0);
   private final DMatrixRMaj bin_pos = new DMatrixRMaj(0);

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

      nominalDecisionVariables = LINEAR_DIMENSIONS * contactState.getNumberOfContactPoints();
      Aeq.reshape(STATIC_EQUILIBRIUM_CONSTRAINTS, nominalDecisionVariables);
      beq.reshape(STATIC_EQUILIBRIUM_CONSTRAINTS, 1);

      MatrixTools.setMatrixBlock(Aeq, 0, 0, comOptimizationModule.Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), 1.0);

      beq.set(2, 0, mg);
      beq.set(3, 0, comY * mg);
      beq.set(4, 0, -comX * mg);

      /* Compute map from positive x to nominal x */

      posDecisionVariables = NUM_BASIS_VECTORS * contactState.getNumberOfContactPoints();
      posXToNominalX.reshape(nominalDecisionVariables, posDecisionVariables);

      MatrixTools.setMatrixBlock(posXToNominalX, 0, 0, comOptimizationModule.posXToNominalX, 0, 0, posXToNominalX.getNumRows(), posXToNominalX.getNumCols(), 1.0);

      /* Compute constraint matrices */
      CommonOps_DDRM.mult(Aeq, posXToNominalX, Aeq_pos);

      /* Compute inequality matrices */
      DMatrixRMaj A_actuation = contactState.getActuationConstraintMatrix();
      DMatrixRMaj b_actuation = contactState.getActuationConstraintVector();
      CommonOps_DDRM.mult(A_actuation, posXToNominalX, Aact_pos);

      Ain_pos.reshape(posDecisionVariables + A_actuation.getNumRows(), posDecisionVariables);
      bin_pos.reshape(posDecisionVariables + b_actuation.getNumRows(), 1);

      for (int basisIdx = 0; basisIdx < posDecisionVariables; basisIdx++)
      {
         Ain_pos.set(basisIdx, basisIdx, -1.0);
      }

      MatrixTools.setMatrixBlock(Ain_pos, posDecisionVariables, 0, A_actuation, 0, 0, A_actuation.getNumRows(), A_actuation.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin_pos, posDecisionVariables, 0, b_actuation, 0, 0, b_actuation.getNumRows(), b_actuation.getNumCols(), 1.0);

      /* Compute optimal force distribution */
      qpSolver.clear();
      qpSolver.resetActiveSet();

      qpSolver.setMaxNumberOfIterations(feasibilityMode ? 100 : 500);
      qpSolver.setConvergenceThreshold(feasibilityMode ? 1e-2 : 1e-9);
      qpSolver.setQuadraticCostFunction(quadraticCost, linearCost, 0.0);
      qpSolver.setLinearEqualityConstraints(Aeq_pos, beq);
      qpSolver.setLinearInequalityConstraints(Ain_pos, bin_pos);

      qpSolver.solve(solutionPos);
      boolean success = !MatrixTools.containsNaN(solutionPos);
      if (success)
      {
         CommonOps_DDRM.mult(posXToNominalX, solutionPos, solution);
      }

      return success;
   }

   public void getResolvedForce(int contactIdx, Vector3DBasics resolvedForceToPack)
   {
      resolvedForceToPack.setX(solution.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.X.ordinal()));
      resolvedForceToPack.setY(solution.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Y.ordinal()));
      resolvedForceToPack.setZ(solution.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Z.ordinal()));
   }

   public void setAsFeasibilitySolver(boolean feasibilityMode)
   {
      this.feasibilityMode = feasibilityMode;
   }

   public DMatrixRMaj getRhoSolution()
   {
      return solutionPos;
   }
}
