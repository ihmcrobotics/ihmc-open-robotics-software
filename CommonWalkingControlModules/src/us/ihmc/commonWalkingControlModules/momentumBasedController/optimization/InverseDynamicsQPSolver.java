package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.OASESConstrainedQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleInefficientActiveSetQPSolver;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseDynamicsQPSolver
{
   private static final boolean HACK_RHO_LOWER_BOUND = false;
   private static final boolean SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE = true;
   private static final boolean DEBUG = true;
   private static final boolean USE_JERRY_SOLVER = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoFrameVector wrenchEquilibriumForceError;
   private final YoFrameVector wrenchEquilibriumTorqueError;

   private final BooleanYoVariable seedFromPreviousSolution = new BooleanYoVariable("seedFromPreviousSolution", registry);
   private final OASESConstrainedQPSolver qpSolver = new OASESConstrainedQPSolver(registry);
   private final SimpleInefficientActiveSetQPSolver jerryQPSolver = new SimpleInefficientActiveSetQPSolver();

   private final IntegerYoVariable numberOfTicksBeforeDeactivatingRhoMin = new IntegerYoVariable("numberOfTicksBeforeDeactivatingRhoMin", registry);
   private final BooleanYoVariable[] activeRhos;
   private final IntegerYoVariable[] countersActiveRhoMin;
   private final DenseMatrix64F rhoMin;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;

   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F jerry_solverInput_lb;
   private final DenseMatrix64F solverInput_ub;
   private final DenseMatrix64F jerry_solverInput_ub;

   private final DenseMatrix64F solverInput_lagrangeEqualityConstraintMultipliers;
   private final DenseMatrix64F solverInput_lagrangeInequalityConstraintMultipliers;

   private final DenseMatrix64F solverOutput;
   private final DenseMatrix64F solverOutput_jointAccelerations;
   private final DenseMatrix64F solverOutput_rhos;

   private final IntegerYoVariable numberOfIterations = new IntegerYoVariable("numberOfIterations", registry);
   private final IntegerYoVariable numberOfConstraints = new IntegerYoVariable("numberOfConstraints", registry);
   private final DoubleYoVariable jointAccelerationRegularization = new DoubleYoVariable("jointAccelerationRegularization", registry);
   private final DoubleYoVariable jointJerkRegularization = new DoubleYoVariable("jointJerkRegularization", registry);
   private final DoubleYoVariable rhoRegularization = new DoubleYoVariable("rhoRegularization", registry);

   private final DenseMatrix64F tempIdentity;
   private final DenseMatrix64F tempNegativeIdentity;
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
      jerry_solverInput_lb = new DenseMatrix64F(problemSize, 1);
      jerry_solverInput_ub = new DenseMatrix64F(problemSize, 1);

      CommonOps.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverInput_lagrangeEqualityConstraintMultipliers = new DenseMatrix64F(problemSize, 1);
      solverInput_lagrangeInequalityConstraintMultipliers = new DenseMatrix64F(problemSize, 1);

      solverOutput = new DenseMatrix64F(problemSize, 1);
      solverOutput_jointAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
      solverOutput_rhos = new DenseMatrix64F(rhoSize, 1);

      tempIdentity = CommonOps.identity(problemSize);
      tempNegativeIdentity = CommonOps.identity(problemSize);
      CommonOps.scale(-1.0, tempNegativeIdentity);
      tempJtW = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      tempMotionTask_H = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      tempMotionTask_f = new DenseMatrix64F(numberOfDoFs, 1);

      tempRhoTask_f = new DenseMatrix64F(rhoSize, 1);

      jointAccelerationRegularization.set(0.005);
      jointJerkRegularization.set(0.1);
      rhoRegularization.set(0.00001);

      pseudoInverseSolver = new DampedLeastSquaresSolver(numberOfDoFs, 0.000001);

      numberOfTicksBeforeDeactivatingRhoMin.set(10);
      countersActiveRhoMin = new IntegerYoVariable[rhoSize];
      activeRhos = new BooleanYoVariable[rhoSize];

      for (int i = 0; i < rhoSize; i++)
      {
         countersActiveRhoMin[i] = new IntegerYoVariable("counterActiveRhoMin_" + i, registry);
         countersActiveRhoMin[i].set(-1);
         activeRhos[i] = new BooleanYoVariable("activeRho_" + i, registry);
      }
      rhoMin = new DenseMatrix64F(rhoSize, 1);
      

      if (DEBUG)
      {
         wrenchEquilibriumForceError = new YoFrameVector("wrenchEquilibriumForceError", null, registry);
         wrenchEquilibriumTorqueError = new YoFrameVector("wrenchEquilibriumTorqueError", null, registry);
      }
      else
      {
         wrenchEquilibriumForceError = null;
         wrenchEquilibriumTorqueError = null;
      }

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

      jAugmented.reshape(0, numberOfDoFs);

      if (seedFromPreviousSolution.getBooleanValue())
         addJointJerkRegularization();
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

   public void addMotionTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(numberOfDoFs, taskSize);
      CommonOps.multTransA(taskJacobian, taskWeight, tempJtW);
      
      addMotionTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   private void addMotionTaskInternal(DenseMatrix64F taskJtW, DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      jAugmented.reshape(jAugmented.getNumRows() + taskSize, numberOfDoFs);
      CommonOps.insert(taskJacobian, jAugmented, jAugmented.getNumRows() - taskSize, 0);

      // Compute: H += J^T W J
      tempMotionTask_H.reshape(numberOfDoFs, numberOfDoFs);
      CommonOps.mult(taskJtW, taskJacobian, tempMotionTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempMotionTask_H, 0, 0, numberOfDoFs, numberOfDoFs, 1.0);

      // Compute: f += - J^T W Objective
      tempMotionTask_f.reshape(numberOfDoFs, 1);
      CommonOps.mult(taskJtW, taskObjective, tempMotionTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempMotionTask_f, 0, 0, numberOfDoFs, 1, -1.0);
   }

   public void addMotionConstraint(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      jAugmented.reshape(jAugmented.getNumRows() + taskSize, numberOfDoFs);
      CommonOps.insert(taskJacobian, jAugmented, jAugmented.getNumRows() - taskSize, 0);

      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, problemSize, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps.insert(taskJacobian, solverInput_Aeq, previousSize, 0);
      CommonOps.insert(taskObjective, solverInput_beq, previousSize, 0);
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

      tempWrenchConstraint_RHS.set(convectiveTerm);
      CommonOps.subtractEquals(tempWrenchConstraint_RHS, additionalExternalWrench);
      CommonOps.subtractEquals(tempWrenchConstraint_RHS, gravityWrench);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         tempWrenchConstraint_J.reshape(Wrench.SIZE, problemSize);
         MatrixTools.setMatrixBlock(tempWrenchConstraint_J, 0, 0, centroidalMomentumMatrix, 0, 0, Wrench.SIZE, numberOfDoFs, -1.0);
         CommonOps.insert(rhoJacobian, tempWrenchConstraint_J, 0, numberOfDoFs);
         
         tempWrenchConstraintJtW.reshape(problemSize, Wrench.SIZE);
         CommonOps.transpose(tempWrenchConstraint_J, tempWrenchConstraintJtW);
         double weight = 150.0;
         CommonOps.scale(weight, tempWrenchConstraintJtW);
         tempWrenchConstraint_H.reshape(problemSize, problemSize);
         CommonOps.mult(tempWrenchConstraintJtW, tempWrenchConstraint_J, tempWrenchConstraint_H);
         CommonOps.addEquals(solverInput_H, tempWrenchConstraint_H);

         tempWrenchConstraint_f.reshape(problemSize, 1);
         CommonOps.mult(tempWrenchConstraintJtW, tempWrenchConstraint_RHS, tempWrenchConstraint_f);
         CommonOps.subtractEquals(solverInput_f, tempWrenchConstraint_f);
      }
      else
      {
         int constraintSize = Wrench.SIZE;
         int previousSize = solverInput_beq.getNumRows();
         
         // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
         solverInput_Aeq.reshape(previousSize + constraintSize, problemSize, true);
         solverInput_beq.reshape(previousSize + constraintSize, 1, true);
         
         MatrixTools.setMatrixBlock(solverInput_Aeq, previousSize, 0, centroidalMomentumMatrix, 0, 0, constraintSize, numberOfDoFs, -1.0);
         CommonOps.insert(rhoJacobian, solverInput_Aeq, previousSize, numberOfDoFs);
         
         CommonOps.insert(tempWrenchConstraint_RHS, solverInput_beq, previousSize, 0);
      }

      hasWrenchesEquilibriumConstraintBeenSetup = true;
   }

   private final DenseMatrix64F tempWrenchConstraint_H = new DenseMatrix64F(200, 200);
   private final DenseMatrix64F tempWrenchConstraint_J = new DenseMatrix64F(Wrench.SIZE, 200);
   private final DenseMatrix64F tempWrenchConstraint_f = new DenseMatrix64F(Wrench.SIZE, 200);
   private final DenseMatrix64F tempWrenchConstraintJtW = new DenseMatrix64F(200, Wrench.SIZE);
   private final DenseMatrix64F tempWrenchConstraint_LHS = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tempWrenchConstraint_RHS = new DenseMatrix64F(Wrench.SIZE, 1);

   public void addRhoTask(DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      MatrixTools.addMatrixBlock(solverInput_H, numberOfDoFs, numberOfDoFs, taskWeight, 0, 0, rhoSize, rhoSize, 1.0);

      CommonOps.mult(taskWeight, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, numberOfDoFs, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void addJointJerkRegularization()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         solverInput_H.add(i, i, jointJerkRegularization.getDoubleValue());
         solverInput_f.add(i, 0, - jointJerkRegularization.getDoubleValue() * solverOutput_jointAccelerations.get(i, 0));
      }
   }

   private final DenseMatrix64F jAugmented = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F jInverseAugmented = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F nullspaceProjector = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F jacobianForPrivilegedJointAccelerations = new DenseMatrix64F(1, 1);

   private final DampedLeastSquaresSolver pseudoInverseSolver;

   public void projectPrivilegedJointAccelerationsInNullspaceOfPreviousTasks(DenseMatrix64F selectionMatrix, DenseMatrix64F privilegedJointAccelerations,
         DenseMatrix64F weight)
   {
      jInverseAugmented.reshape(numberOfDoFs, jAugmented.getNumRows());
      pseudoInverseSolver.setA(jAugmented);
      pseudoInverseSolver.invert(jInverseAugmented);

      nullspaceProjector.reshape(numberOfDoFs, numberOfDoFs);
      // I - J^* J
      CommonOps.mult(-1.0, jInverseAugmented, jAugmented, nullspaceProjector);
      for (int i = 0; i < numberOfDoFs; i++)
         nullspaceProjector.add(i, i, 1.0);

      jacobianForPrivilegedJointAccelerations.reshape(privilegedJointAccelerations.getNumRows(), numberOfDoFs);
      CommonOps.mult(selectionMatrix, nullspaceProjector, jacobianForPrivilegedJointAccelerations);

      addMotionTask(jacobianForPrivilegedJointAccelerations, privilegedJointAccelerations, weight);
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

      numberOfConstraints.set(Aeq.getNumRows() + Ain.getNumRows());

      if (!HACK_RHO_LOWER_BOUND)
         CommonOps.insert(rhoMin, solverInput_lb, numberOfDoFs, 0);

      NoConvergenceException noConvergenceException = null;

      qpSolverTimer.startMeasurement();

      if (USE_JERRY_SOLVER)
      {
         jerry_solverInput_lb.set(solverInput_lb);
         jerry_solverInput_ub.set(solverInput_ub);
         CommonOps.scale(-1.0, solverInput_lb, jerry_solverInput_lb);
         tempNegativeIdentity.reshape(problemSize, problemSize);
         CommonOps.setIdentity(tempNegativeIdentity);
         CommonOps.scale(-1.0, tempNegativeIdentity);
         tempIdentity.reshape(problemSize, problemSize);
         CommonOps.setIdentity(tempIdentity);

         int row = problemSize - 1;
         while(row >= 0)
         {
            if (Double.isInfinite(jerry_solverInput_lb.get(row, 0)))
            {
               MatrixTools.removeRow(jerry_solverInput_lb, row);
               MatrixTools.removeRow(tempNegativeIdentity, row);
            }
            row--;
         }

         row = problemSize - 1;
         while(row >= 0)
         {
            if (Double.isInfinite(jerry_solverInput_ub.get(row, 0)))
            {
               MatrixTools.removeRow(jerry_solverInput_ub, row);
               MatrixTools.removeRow(tempIdentity, row);
            }
            row--;
         }

         solverInput_Ain.reshape(jerry_solverInput_lb.getNumRows() + jerry_solverInput_ub.getNumRows(), problemSize);
         solverInput_bin.reshape(jerry_solverInput_lb.getNumRows() + jerry_solverInput_ub.getNumRows(), 1);

         CommonOps.insert(tempNegativeIdentity, solverInput_Ain, 0, 0);
         CommonOps.insert(tempIdentity, solverInput_Ain, tempNegativeIdentity.getNumRows(), 0);
         CommonOps.insert(jerry_solverInput_lb, solverInput_bin, 0, 0);
         CommonOps.insert(jerry_solverInput_ub, solverInput_bin, jerry_solverInput_lb.getNumRows(), 0);

         jerryQPSolver.clear();
         jerryQPSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
         jerryQPSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
         numberOfIterations.set(jerryQPSolver.solve(solverOutput, solverInput_lagrangeEqualityConstraintMultipliers, solverInput_lagrangeInequalityConstraintMultipliers));
      }
      else
      {
         try
         {
            numberOfIterations.set(qpSolver.solve(H, f, Aeq, beq, Ain, bin, lb, ub, output, firstCall));
         }
         catch (NoConvergenceException e)
         {
            noConvergenceException = e;
         }
      }

      qpSolverTimer.stopMeasurement();

      CommonOps.extract(solverOutput, 0, numberOfDoFs, 0, 1, solverOutput_jointAccelerations, 0, 0);
      CommonOps.extract(solverOutput, numberOfDoFs, problemSize, 0, 1, solverOutput_rhos, 0, 0);

      seedFromPreviousSolution.set(true);
      hasWrenchesEquilibriumConstraintBeenSetup = false;

      if (DEBUG)
      {
         CommonOps.mult(tempWrenchConstraint_J, output, tempWrenchConstraint_LHS);
         int index = 0;
         wrenchEquilibriumTorqueError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         wrenchEquilibriumTorqueError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         wrenchEquilibriumTorqueError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         wrenchEquilibriumForceError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         wrenchEquilibriumForceError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         wrenchEquilibriumForceError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
      }

      if (HACK_RHO_LOWER_BOUND)
      {
         for (int i = 0; i < rhoSize; i++)
         {
            IntegerYoVariable counter = countersActiveRhoMin[i];

            if (!activeRhos[i].getBooleanValue())
            {
               counter.set(-1);
            }
            else if (solverOutput_rhos.get(i, 0) - 1.0e-5 <= rhoMin.get(i, 0))
            {
               counter.set(numberOfTicksBeforeDeactivatingRhoMin.getIntegerValue());
            }
            else
            {
               counter.set(Math.max(counter.getIntegerValue() - 1, -1));
            }

            if (counter.getIntegerValue() > 0)
               solverInput_lb.set(numberOfDoFs + i, 0, rhoMin.get(i, 0));
            else
               solverInput_lb.set(numberOfDoFs + i, 0, Double.NEGATIVE_INFINITY);
         }
      }

      if (!USE_JERRY_SOLVER)
      {
         if (noConvergenceException != null)
            throw noConvergenceException;
      }
   }

   private void printForJerry()
   {
      printMatrix("H", solverInput_H);
      printVector("f", solverInput_f);
      printVector("C", solverInput_Ain);
      printVector("d", solverInput_bin);
      printVector("solution", solverOutput);
   }

   private void printMatrix(String name, DenseMatrix64F m)
   {
      String str = "double[][] " + name + " = new double[][]{";

      for (int row = 0; row < m.getNumRows(); row ++)
      {
         str += "new double[]{";
         for (int col = 0; col < m.getNumCols() - 1; col++)
         {
            str += m.get(col, 0);
            if (col < m.getNumCols() - 1)
               str += ", ";
         }
         str += m.get(row, m.getNumCols() - 1) + "}";
         if (row < m.getNumRows() - 1)
            str += ", ";
      }
      str += "};";
      
      System.out.println(str);
   }

   private void printVector(String name, DenseMatrix64F v)
   {
      String str = "double[] " + name + " = new double[]{";

      for (int i = 0; i < v.getNumRows(); i++)
      {
         str += v.get(i, 0);
         if (i < v.getNumRows() - 1)
            str += ", ";
      }
      str += "};";
      System.out.println(str);
   }

   public DenseMatrix64F getJointAccelerations()
   {
      return solverOutput_jointAccelerations;
   }

   public DenseMatrix64F getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinJointAccelerations(double qDDotMin)
   {
      for (int i = 4; i < numberOfDoFs; i++)
         solverInput_lb.set(i, 0, qDDotMin);
   }

   public void setMinJointAccelerations(DenseMatrix64F qDDotMin)
   {
      CommonOps.insert(qDDotMin, solverInput_lb, 0, 0);
   }

   public void setMaxJointAccelerations(double qDDotMax)
   {
      for (int i = 4; i < numberOfDoFs; i++)
         solverInput_ub.set(i, 0, qDDotMax);
   }

   public void setMaxJointAccelerations(DenseMatrix64F qDDotMax)
   {
      CommonOps.insert(qDDotMax, solverInput_ub, 0, 0);
   }

   public void setMinRho(double rhoMin)
   {
      CommonOps.fill(this.rhoMin, rhoMin);
//      for (int i = numberOfDoFs; i < problemSize; i++)
//         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DenseMatrix64F rhoMin)
   {
      this.rhoMin.set(rhoMin);
//      CommonOps.insert(rhoMin, solverInput_lb, numberOfDoFs, 0);
   }

   public void setMaxRho(double rhoMax)
   {
//      for (int i = numberOfDoFs; i < problemSize; i++)
//         solverInput_ub.set(i, 0, rhoMax);
   }

   public void setMaxRho(DenseMatrix64F rhoMax)
   {
//      CommonOps.insert(rhoMax, solverInput_ub, numberOfDoFs, 0);
   }

   public void setActiveRhos(boolean[] activeRhos)
   {
      for (int i = 0; i < rhoSize; i++)
         this.activeRhos[i].set(activeRhos[i]);
   }
}
