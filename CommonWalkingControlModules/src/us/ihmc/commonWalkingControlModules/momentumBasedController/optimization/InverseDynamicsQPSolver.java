package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class InverseDynamicsQPSolver
{
   private static final boolean SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoFrameVector wrenchEquilibriumForceError;
   private final YoFrameVector wrenchEquilibriumTorqueError;

   private final YoBoolean firstCall = new YoBoolean("firstCall", registry);
   private final ActiveSetQPSolver qpSolver;

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

   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final YoDouble jointAccelerationRegularization = new YoDouble("jointAccelerationRegularization", registry);
   private final YoDouble jointJerkRegularization = new YoDouble("jointJerkRegularization", registry);
   private final YoDouble jointTorqueWeight = new YoDouble("jointTorqueWeight", registry);
   private final DenseMatrix64F regularizationMatrix;

   private final DenseMatrix64F tempJtW;
   private final DenseMatrix64F tempMotionTask_H;
   private final DenseMatrix64F tempMotionTask_f;
   private final DenseMatrix64F tempRhoTask_H;
   private final DenseMatrix64F tempRhoTask_f;
   private final DenseMatrix64F tempTorqueTask_H;

   private final int numberOfDoFs;
   private final int rhoSize;
   private final int problemSize;
   private final boolean hasFloatingBase;
   private boolean hasWrenchesEquilibriumConstraintBeenSetup = false;

   public InverseDynamicsQPSolver(ActiveSetQPSolver qpSolver, int numberOfDoFs, int rhoSize, boolean hasFloatingBase, YoVariableRegistry parentRegistry)
   {
      this.qpSolver = qpSolver;
      this.numberOfDoFs = numberOfDoFs;
      this.rhoSize = rhoSize;
      this.hasFloatingBase = hasFloatingBase;
      this.problemSize = numberOfDoFs + rhoSize;

      firstCall.set(true);

      solverInput_H = new DenseMatrix64F(problemSize, problemSize);
      solverInput_f = new DenseMatrix64F(problemSize, 1);

      solverInput_Aeq = new DenseMatrix64F(0, problemSize);
      solverInput_beq = new DenseMatrix64F(0, 1);
      solverInput_Ain = new DenseMatrix64F(0, problemSize);
      solverInput_bin = new DenseMatrix64F(0, 1);

      solverInput_lb = new DenseMatrix64F(problemSize, 1);
      solverInput_ub = new DenseMatrix64F(problemSize, 1);

      CommonOps.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverOutput = new DenseMatrix64F(problemSize, 1);
      solverOutput_jointAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
      solverOutput_rhos = new DenseMatrix64F(rhoSize, 1);

      tempJtW = new DenseMatrix64F(problemSize, problemSize);
      tempMotionTask_H = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      tempMotionTask_f = new DenseMatrix64F(numberOfDoFs, 1);

      tempRhoTask_H = new DenseMatrix64F(rhoSize, rhoSize);
      tempRhoTask_f = new DenseMatrix64F(rhoSize, 1);

      tempTorqueTask_H = new DenseMatrix64F(numberOfDoFs, problemSize);

      jointAccelerationRegularization.set(0.005);
      jointJerkRegularization.set(0.1);
      jointTorqueWeight.set(0.001);
      regularizationMatrix = new DenseMatrix64F(problemSize, problemSize);

      for (int i = 0; i < numberOfDoFs; i++)
         regularizationMatrix.set(i, i, jointAccelerationRegularization.getDoubleValue());
      double defaultRhoRegularization = 0.00001;
      for (int i = numberOfDoFs; i < problemSize; i++)
         regularizationMatrix.set(i, i, defaultRhoRegularization);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
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

   public void setJerkRegularizationWeight(double weight)
   {
      jointJerkRegularization.set(weight);
   }

   public void setJointTorqueWeight(double weight)
   {
      jointTorqueWeight.set(weight);
   }

   public void setRhoRegularizationWeight(DenseMatrix64F weight)
   {
      CommonOps.insert(weight, regularizationMatrix, numberOfDoFs, numberOfDoFs);
   }

   public void reset()
   {
      for (int i = 0; i < numberOfDoFs; i++)
         regularizationMatrix.set(i, i, jointAccelerationRegularization.getDoubleValue());

      solverInput_H.zero();

      solverInput_f.zero();

      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 1);

      if (!firstCall.getBooleanValue())
         addJointJerkRegularization();
   }

   private void addRegularization()
   {
      CommonOps.addEquals(solverInput_H, regularizationMatrix);
   }

   private void addJointJerkRegularization()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         solverInput_H.add(i, i, jointJerkRegularization.getDoubleValue());
         solverInput_f.add(i, 0, -jointJerkRegularization.getDoubleValue() * solverOutput_jointAccelerations.get(i, 0));
      }
   }

   public void addMotionInput(MotionQPInput input)
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
      // Compute: H += J^T W J
      CommonOps.mult(taskJtW, taskJacobian, tempMotionTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempMotionTask_H, 0, 0, numberOfDoFs, numberOfDoFs, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps.mult(taskJtW, taskObjective, tempMotionTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempMotionTask_f, 0, 0, numberOfDoFs, 1, -1.0);
   }

   public void addMotionConstraint(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, problemSize, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps.insert(taskJacobian, solverInput_Aeq, previousSize, 0);
      CommonOps.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   public void addTorqueMinimizationObjective(DenseMatrix64F torqueJacobian, DenseMatrix64F torqueObjective)
   {
      int taskSize = torqueObjective.getNumRows();
      int controlSize = torqueJacobian.getNumCols();

      // J^T W
      tempJtW.reshape(controlSize, taskSize);
      MatrixTools.scaleTranspose(jointTorqueWeight.getDoubleValue(), torqueJacobian, tempJtW);

      // Compute: H += J^T W J
      CommonOps.multAdd(tempJtW, torqueJacobian, solverInput_H);

      // Compute: f += - J^T W Objective
      CommonOps.multAdd(-1.0, tempJtW, torqueObjective, solverInput_f);
   }

   public void addTorqueMinimizationObjective(DenseMatrix64F torqueQddotJacobian, DenseMatrix64F torqueRhoJacobian, DenseMatrix64F torqueObjective)
   {
      int taskSize = torqueObjective.getNumRows();

      tempTorqueTask_H.reshape(taskSize, problemSize);
      CommonOps.insert(torqueQddotJacobian, tempTorqueTask_H, 0, 0);
      CommonOps.insert(torqueRhoJacobian, tempTorqueTask_H, 0, numberOfDoFs);

      addTorqueMinimizationObjective(tempTorqueTask_H, torqueObjective);
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
      if (!hasFloatingBase)
      {
         hasWrenchesEquilibriumConstraintBeenSetup = true;
         return;
      }

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

   public void addRhoTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();
      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      CommonOps.multTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: H += J^T W J
      CommonOps.mult(tempJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, numberOfDoFs, numberOfDoFs, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps.mult(tempJtW, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, numberOfDoFs, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void solve() throws NoConvergenceException
   {
      if (!hasWrenchesEquilibriumConstraintBeenSetup)
         throw new RuntimeException("The wrench equilibrium constraint has to be setup before calling solve().");

      addRegularization();

      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setVariableBounds(solverInput_lb, solverInput_ub);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

      numberOfIterations.set(qpSolver.solve(solverOutput));

      qpSolverTimer.stopMeasurement();
      
      hasWrenchesEquilibriumConstraintBeenSetup = false;

      if (MatrixTools.containsNaN(solverOutput))
         throw new NoConvergenceException(numberOfIterations.getIntegerValue());

      CommonOps.extract(solverOutput, 0, numberOfDoFs, 0, 1, solverOutput_jointAccelerations, 0, 0);
      CommonOps.extract(solverOutput, numberOfDoFs, problemSize, 0, 1, solverOutput_rhos, 0, 0);

      firstCall.set(false);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         if (hasFloatingBase)
         {
            CommonOps.mult(tempWrenchConstraint_J, solverOutput, tempWrenchConstraint_LHS);
            int index = 0;
            wrenchEquilibriumTorqueError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         }
      }
   }

   private void printForJerry()
   {
      MatrixTools.printJavaForConstruction("H", solverInput_H);
      MatrixTools.printJavaForConstruction("f", solverInput_f);
      MatrixTools.printJavaForConstruction("lowerBounds", solverInput_lb);
      MatrixTools.printJavaForConstruction("upperBounds", solverInput_ub);
      MatrixTools.printJavaForConstruction("solution", solverOutput);
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
      for (int i = numberOfDoFs; i < problemSize; i++)
         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DenseMatrix64F rhoMin)
   {
      CommonOps.insert(rhoMin, solverInput_lb, numberOfDoFs, 0);
   }

   public void setMaxRho(double rhoMax)
   {
      for (int i = numberOfDoFs; i < problemSize; i++)
         solverInput_ub.set(i, 0, rhoMax);
   }

   public void setMaxRho(DenseMatrix64F rhoMax)
   {
      CommonOps.insert(rhoMax, solverInput_ub, numberOfDoFs, 0);
   }
}
