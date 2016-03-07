package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.OASESConstrainedQPSolver;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseDynamicsQPSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable seedFromPreviousSolution = new BooleanYoVariable("seedFromPreviousSolution", registry);
   private final OASESConstrainedQPSolver qpSolver = new OASESConstrainedQPSolver(registry);

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;

   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;

   private final DenseMatrix64F solverOutput;
   private final DenseMatrix64F solverOutput_jointAccelerations;
   private final DenseMatrix64F solverOutput_rhos;

   private final DoubleYoVariable jointAccelerationRegularization = new DoubleYoVariable("jointAccelerationRegularization", registry);
   private final DoubleYoVariable rhoRegularization = new DoubleYoVariable("rhoRegularization", registry);

   private final DenseMatrix64F tempJtW;
   private final DenseMatrix64F tempMotionTask_H;
   private final DenseMatrix64F tempMotionTask_f;
   private final DenseMatrix64F tempRhoTask_f;


   private final int numberOfDoFs;
   private final int rhoSize;
   private final int problemSize;
   private boolean hasWrenchesEquilibriumConstraintBeenSetup = false;

   public InverseDynamicsQPSolver(int numberOfDoFs, int rhoSize, YoVariableRegistry parentRegistry)
   {
      this.numberOfDoFs = numberOfDoFs;
      this.rhoSize = rhoSize;
      this.problemSize = numberOfDoFs + rhoSize;

      solverInput_H = new DenseMatrix64F(problemSize, problemSize);
      solverInput_f = new DenseMatrix64F(problemSize, 1);

      solverInput_Aeq = new DenseMatrix64F(0, problemSize);
      solverInput_beq = new DenseMatrix64F(0, 0);
      solverInput_Ain = new DenseMatrix64F(0, problemSize);
      solverInput_bin = new DenseMatrix64F(0, 0);

      solverInput_lb = new DenseMatrix64F(problemSize, 1);
      solverInput_ub = new DenseMatrix64F(problemSize, 1);

      CommonOps.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      tempJtW = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      tempMotionTask_H = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      tempMotionTask_f = new DenseMatrix64F(numberOfDoFs, 1);

      tempRhoTask_f = new DenseMatrix64F(rhoSize, 1);

      jointAccelerationRegularization.set(0.05);
      rhoRegularization.set(0.001);

      solverOutput = new DenseMatrix64F(problemSize, 1);
      solverOutput_jointAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
      solverOutput_rhos = new DenseMatrix64F(rhoSize, 1);

      parentRegistry.addChild(registry);
   }

   public void setAccelerationRegularizationWeight(double weight)
   {
      jointAccelerationRegularization.set(weight);
   }

   public void setRhoRegularizationWeight(double weight)
   {
      rhoRegularization.set(weight);
   }

   public void reset()
   {
      solverInput_H.zero();
      for (int i = 0; i < numberOfDoFs; i++)
         solverInput_H.set(i, i, jointAccelerationRegularization.getDoubleValue());
      for (int i = numberOfDoFs; i < problemSize; i++)
         solverInput_H.set(i, i, rhoRegularization.getDoubleValue());

      solverInput_f.zero();

      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 0);
   }

   public void addMotionInput(InverseDynamicsMotionQPInput input)
   {
      if (input.isMotionConstraint())
         addMotionConstraint(input.taskJacobian, input.taskObjective);
      else if (input.useWeightScalar())
         addMotionTask(input.taskJacobian, input.taskObjective, input.getWeightScalar());
      else
         addMotionTask(input.taskJacobian, input.taskObjective, input.taskWeightMatrix);
   }

   public void addMotionTask(DenseMatrix64F taskJ, DenseMatrix64F taskObjective, double taskWeight)
   {
      int taskSize = taskJ.getNumRows();

      // J^T W
      tempJtW.reshape(numberOfDoFs, taskSize);
      MatrixTools.scaleTranspose(taskWeight, taskJ, tempJtW);
      
      addMotionTaskInternal(tempJtW, taskJ, taskObjective);
   }

   public void addMotionTask(DenseMatrix64F taskJ, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      int taskSize = taskJ.getNumRows();

      // J^T W
      tempJtW.reshape(numberOfDoFs, taskSize);
      CommonOps.multTransA(taskJ, taskWeight, tempJtW);
      
      addMotionTaskInternal(tempJtW, taskJ, taskObjective);
   }

   private void addMotionTaskInternal(DenseMatrix64F taskJtW, DenseMatrix64F taskJ, DenseMatrix64F taskObjective)
   {
      // Compute: H += J^T W J
      tempMotionTask_H.reshape(numberOfDoFs, numberOfDoFs);
      CommonOps.mult(taskJtW, taskJ, tempMotionTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempMotionTask_H, 0, 0, numberOfDoFs, numberOfDoFs, 1.0);

      // Compute: f += - J^T W xDot
      tempMotionTask_f.reshape(numberOfDoFs, 1);
      CommonOps.mult(taskJtW, taskObjective, tempMotionTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempMotionTask_f, 0, 0, numberOfDoFs, 1, -1.0);
   }

   /**
    * Need to be called before {@link #solve()}.
    * It sets up the constraint that ensures that the solution is dynamically feasible:
    * <p>
    * <li> hDot = &sum;W<sub>ext</sub>
    * <li> A * qDDot + ADot * qDot = Q * &rho; + &sum;W<sub>user</sub> + W<sub>gravity</sub>
    * <li> -A * qDDot - ADot * qDot = - Q * &rho; - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * <li> -A * qDDot + Q * &rho; = ADot * qDot - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * <li> [-A Q] * [qDDot<sup>T</sup> &rho;<sup>T</sup>]<sup>T</sup> = ADot * qDot - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * </p>
    * @param centroidalMomentumMatrix refers to A in the equation.
    * @param rhoJacobian refers to Q in the equation. Q&rho; represents external wrench to be optimized for.
    * @param convectiveTerm refers to ADot * qDot in the equation.
    * @param additionalExternalWrench refers to &sum;W<sub>user</sub> in the equation. These are constant wrenches usually used for compensating for the weight of an object that the robot is holding.
    * @param gravityWrench refers to W<sub>gravity</sub> in the equation. It the wrench induced by the wieght of the robot.
    */
   public void setupWrenchesEquilibriumConstraint(DenseMatrix64F centroidalMomentumMatrix, DenseMatrix64F rhoJacobian, DenseMatrix64F convectiveTerm,
         DenseMatrix64F additionalExternalWrench, DenseMatrix64F gravityWrench)
   {
      int constraintSize = Wrench.SIZE;
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + constraintSize, problemSize, true);
      solverInput_beq.reshape(previousSize + constraintSize, 1, true);

      MatrixTools.setMatrixBlock(solverInput_Aeq, previousSize, 0, centroidalMomentumMatrix, 0, 0, constraintSize, numberOfDoFs, -1.0);
      CommonOps.insert(rhoJacobian, solverInput_Aeq, previousSize, numberOfDoFs);

      tempWrenchEquilibriumRightHandSide.set(convectiveTerm);
      CommonOps.subtractEquals(tempWrenchEquilibriumRightHandSide, additionalExternalWrench);
      CommonOps.subtractEquals(tempWrenchEquilibriumRightHandSide, gravityWrench);
      CommonOps.insert(tempWrenchEquilibriumRightHandSide, solverInput_beq, previousSize, 0);

      hasWrenchesEquilibriumConstraintBeenSetup = true;
   }

   private final DenseMatrix64F tempWrenchEquilibriumRightHandSide = new DenseMatrix64F(Wrench.SIZE, 1);

   public void addMotionConstraint(DenseMatrix64F taskJ, DenseMatrix64F taskObjective)
   {
      int taskSize = taskJ.getNumRows();
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, problemSize, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps.insert(taskJ, solverInput_Aeq, previousSize, 0);
      CommonOps.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   public void addRhoTask(DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      MatrixTools.addMatrixBlock(solverInput_H, numberOfDoFs, numberOfDoFs, taskWeight, 0, 0, rhoSize, rhoSize, 1.0);

      CommonOps.mult(taskWeight, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, numberOfDoFs, 0, tempRhoTask_f, 0, 0, rhoSize, 1, 1.0);
   }

   public void solve() throws NoConvergenceException
   {
      if (!hasWrenchesEquilibriumConstraintBeenSetup)
         throw new RuntimeException("The wrench equilibrium constraint has to be setup before calling solve().");

      boolean firstCall = !seedFromPreviousSolution.getBooleanValue();

      DenseMatrix64F H = solverInput_H;
      DenseMatrix64F f = solverInput_f;
      DenseMatrix64F Aeq = solverInput_Aeq;
      DenseMatrix64F beq = solverInput_beq;
      DenseMatrix64F Ain = solverInput_Ain;
      DenseMatrix64F bin = solverInput_bin;
      DenseMatrix64F lb = solverInput_lb;
      DenseMatrix64F ub = solverInput_ub;
      DenseMatrix64F output = solverOutput;

      NoConvergenceException noConvergenceException;
      try
      {
         qpSolver.solve(H, f, Aeq, beq, Ain, bin, lb, ub, output, firstCall);
         noConvergenceException = null;
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
      }

      CommonOps.extract(solverOutput, 0, numberOfDoFs, 0, 1, solverOutput_jointAccelerations, 0, 0);
      CommonOps.extract(solverOutput, numberOfDoFs, problemSize, 0, 1, solverOutput_rhos, 0, 0);

      seedFromPreviousSolution.set(true);
      hasWrenchesEquilibriumConstraintBeenSetup = false;

      if (noConvergenceException != null)
         throw noConvergenceException;
   }

   public DenseMatrix64F getJointAccelerations()
   {
      return solverOutput_jointAccelerations;
   }

   public DenseMatrix64F getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinJointAccelerations(DenseMatrix64F qDDotMin)
   {
      CommonOps.insert(qDDotMin, solverInput_lb, 0, 0);
   }

   public void setMaxJointAccelerations(DenseMatrix64F qDDotMax)
   {
      CommonOps.insert(qDDotMax, solverInput_ub, 0, 0);
   }

   public void setMinRho(double rhoMin)
   {
      for (int i = numberOfDoFs; i < problemSize; i++)
         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DenseMatrix64F rhoMin)
   {
      CommonOps.insert(rhoMin, solverInput_lb, numberOfDoFs, 0);
   }

   public void setMaxRho(DenseMatrix64F rhoMax)
   {
      CommonOps.insert(rhoMax, solverInput_ub, numberOfDoFs, 0);
   }
}
