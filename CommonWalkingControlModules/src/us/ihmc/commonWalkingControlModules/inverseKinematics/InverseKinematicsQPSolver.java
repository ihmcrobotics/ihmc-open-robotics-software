package us.ihmc.commonWalkingControlModules.inverseKinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.OASESConstrainedQPSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseKinematicsQPSolver
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

   private final DenseMatrix64F desiredJointVelocities;

   private final IntegerYoVariable numberOfIterations = new IntegerYoVariable("numberOfIterations", registry);
   private final IntegerYoVariable numberOfConstraints = new IntegerYoVariable("numberOfConstraints", registry);
   private final DoubleYoVariable jointVelocityRegularization = new DoubleYoVariable("jointVelocityRegularization", registry);

   private final int numberOfDoFs;

   public InverseKinematicsQPSolver(int numberOfDoFs, YoVariableRegistry parentRegistry)
   {
      this.numberOfDoFs = numberOfDoFs;

      solverInput_H = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      solverInput_f = new DenseMatrix64F(numberOfDoFs, 1);
      solverInput_lb = new DenseMatrix64F(numberOfDoFs, 1);
      solverInput_ub = new DenseMatrix64F(numberOfDoFs, 1);

      solverInput_Aeq = new DenseMatrix64F(0, numberOfDoFs);
      solverInput_beq = new DenseMatrix64F(0, 0);
      solverInput_Ain = new DenseMatrix64F(0, numberOfDoFs);
      solverInput_bin = new DenseMatrix64F(0, 0);

      CommonOps.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      jointVelocityRegularization.set(0.1);

      desiredJointVelocities = new DenseMatrix64F(numberOfDoFs, 1);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      solverInput_H.zero();
      for (int i = 0; i < numberOfDoFs; i++)
         solverInput_H.set(i, i, jointVelocityRegularization.getDoubleValue());
      solverInput_f.reshape(numberOfDoFs, 1);
      solverInput_f.zero();

      solverInput_Aeq.reshape(0, numberOfDoFs);
      solverInput_beq.reshape(0, 0);
   }

   private final DenseMatrix64F tempJtW = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tempTask_H = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tempTask_f = new DenseMatrix64F(1, 1);

   public void addMotionInput(MotionQPInput input)
   {
      if (input.isMotionConstraint())
         addMotionConstraint(input.taskJacobian, input.taskObjective);
      else if (input.useWeightScalar())
         addMotionTask(input.taskJacobian, input.taskObjective, input.getWeightScalar());
      else
         addMotionTask(input.taskJacobian, input.taskObjective, input.taskWeightMatrix);
   }

   public void addMotionTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, double taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(numberOfDoFs, taskSize);
      MatrixTools.scaleTranspose(taskWeight, taskJacobian, tempJtW);

      addMotionTaskInternal(tempJtW, taskJacobian, taskObjective);
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
      tempTask_H.reshape(numberOfDoFs, numberOfDoFs);
      CommonOps.mult(taskJtW, taskJacobian, tempTask_H);
      CommonOps.addEquals(solverInput_H, tempTask_H);

      // Compute: f += - J^T W xDot
      tempTask_f.reshape(numberOfDoFs, 1);
      CommonOps.mult(taskJtW, taskObjective, tempTask_f);
      CommonOps.subtractEquals(solverInput_f, tempTask_f);
   }

   public void addMotionConstraint(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, numberOfDoFs, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps.insert(taskJacobian, solverInput_Aeq, previousSize, 0);
      CommonOps.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   public void solve() throws NoConvergenceException
   {
      boolean firstCall = !seedFromPreviousSolution.getBooleanValue();

      DenseMatrix64F H = solverInput_H;
      DenseMatrix64F f = solverInput_f;
      DenseMatrix64F Aeq = solverInput_Aeq;
      DenseMatrix64F beq = solverInput_beq;
      DenseMatrix64F Ain = solverInput_Ain;
      DenseMatrix64F bin = solverInput_bin;
      DenseMatrix64F lb = solverInput_lb;
      DenseMatrix64F ub = solverInput_ub;
      DenseMatrix64F output = desiredJointVelocities;

      numberOfConstraints.set(Aeq.getNumRows() + Ain.getNumRows());

      numberOfIterations.set(qpSolver.solve(H, f, Aeq, beq, Ain, bin, lb, ub, output, firstCall));

      seedFromPreviousSolution.set(true);
   }

   public DenseMatrix64F getJointVelocities()
   {
      return desiredJointVelocities;
   }

   public void setRegularizationWeight(double weight)
   {
      jointVelocityRegularization.set(weight);
   }

   public void setMinJointVelocities(DenseMatrix64F qDotMin)
   {
      solverInput_lb.set(qDotMin);
   }

   public void setMaxJointVelocities(DenseMatrix64F qDotMax)
   {
      solverInput_ub.set(qDotMax);
   }
}
