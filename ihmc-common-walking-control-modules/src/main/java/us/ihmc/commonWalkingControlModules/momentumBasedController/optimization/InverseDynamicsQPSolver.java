package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.NativeActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * It is assumed that the cost function has the form
 *
 * <pre>
 * f(x) = 0.5 * x<sup>T</sup> H x + f<sup>T</sup>x
 * </pre>
 */
public class InverseDynamicsQPSolver
{
   private static final boolean SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoFrameVector3D wrenchEquilibriumForceError;
   private final YoFrameVector3D wrenchEquilibriumTorqueError;

   private final YoBoolean addRateRegularization = new YoBoolean("AddRateRegularization", registry);
   private final NativeActiveSetQPSolverWithInactiveVariablesInterface qpSolver;

   private final NativeQPVariableSubstitution accelerationVariablesSubstitution = new NativeQPVariableSubstitution();

   private final NativeMatrix solver_H;
   private final NativeMatrix solver_f;

   private final NativeMatrix solver_H_previous;
   private final NativeMatrix solver_f_previous;

   private final NativeMatrix solver_Aeq;
   private final NativeMatrix solver_beq;
   private final NativeMatrix solver_Ain;
   private final NativeMatrix solver_bin;

   private final NativeMatrix solver_lb;
   private final NativeMatrix solver_ub;

   private final NativeMatrix solverInput_lb_previous;
   private final NativeMatrix solverInput_ub_previous;

   private final NativeMatrix solverInput_activeIndices;

   private final NativeMatrix solverOutput;
   private final NativeMatrix solverOutput_jointAccelerations;
   private final NativeMatrix solverOutput_rhos;

   private final NativeMatrix tempJacobian = new NativeMatrix(0, 0);
   private final NativeMatrix tempObjective = new NativeMatrix(0, 0);
   private final NativeMatrix tempWeight = new NativeMatrix(0, 0);

   private final YoInteger numberOfActiveVariables = new YoInteger("numberOfActiveVariables", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final YoDouble jointAccelerationRegularization = new YoDouble("jointAccelerationRegularization", registry);
   private final YoDouble jointJerkRegularization = new YoDouble("jointJerkRegularization", registry);
   private final YoDouble jointTorqueWeight = new YoDouble("jointTorqueWeight", registry);
   private final NativeMatrix regularizationMatrix;

   private final NativeMatrix tempJtW = new NativeMatrix(0, 0);

   private final int numberOfDoFs;
   private final int rhoSize;
   private final int problemSize;
   private final boolean hasFloatingBase;
   private boolean hasWrenchesEquilibriumConstraintBeenSetup = false;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   private final double dt;

   public InverseDynamicsQPSolver(NativeActiveSetQPSolverWithInactiveVariablesInterface qpSolver,
                                  int numberOfDoFs,
                                  int rhoSize,
                                  boolean hasFloatingBase,
                                  double dt,
                                  YoRegistry parentRegistry)
   {
      this.qpSolver = qpSolver;
      this.numberOfDoFs = numberOfDoFs;
      this.rhoSize = rhoSize;
      this.hasFloatingBase = hasFloatingBase;
      this.problemSize = numberOfDoFs + rhoSize;
      this.dt = dt;

      addRateRegularization.set(false);

      solver_H = qpSolver.getCostHessianUnsafe();
      solver_f = qpSolver.getCostGradientUnsafe();
      solver_H.reshape(problemSize, problemSize);
      solver_f.reshape(problemSize, 1);

      solver_H_previous = new NativeMatrix(problemSize, problemSize);
      solver_f_previous = new NativeMatrix(problemSize, 1);

      solver_Aeq = qpSolver.getAeqUnsafe();
      solver_beq = qpSolver.getBeqUnsafe();
      solver_Aeq.reshape(problemSize, problemSize);
      solver_beq.reshape(problemSize, 1);
      solver_Ain = qpSolver.getAinUnsafe();
      solver_bin = qpSolver.getBinUnsafe();
      solver_Ain.reshape(problemSize, problemSize);
      solver_bin.reshape(problemSize, 1);

      solver_lb = qpSolver.getLowerBoundsUnsafe();
      solver_ub = qpSolver.getUpperBoundsUnsafe();
      solver_lb.reshape(problemSize, 1);
      solver_ub.reshape(problemSize, 1);

      solverInput_lb_previous = new NativeMatrix(problemSize, 1);
      solverInput_ub_previous = new NativeMatrix(problemSize, 1);

      solver_lb.fill(Double.NEGATIVE_INFINITY);
      solver_ub.fill(Double.POSITIVE_INFINITY);

      solverInput_activeIndices = new NativeMatrix(problemSize, 1);
      solverInput_activeIndices.fill(1.0);

      solverOutput = new NativeMatrix(problemSize, 1);
      solverOutput_jointAccelerations = new NativeMatrix(numberOfDoFs, 1);
      solverOutput_rhos = new NativeMatrix(rhoSize, 1);

      jointAccelerationRegularization.set(0.005);
      jointJerkRegularization.set(0.1);
      jointTorqueWeight.set(0.001);
      regularizationMatrix = new NativeMatrix(problemSize, problemSize);

//      for (int i = 0; i < numberOfDoFs; i++)
//         regularizationMatrix.set(i, i, jointAccelerationRegularization.getDoubleValue());
      double defaultRhoRegularization = 0.00001;
//      for (int i = numberOfDoFs; i < problemSize; i++)
//         regularizationMatrix.set(i, i, defaultRhoRegularization);
      regularizationMatrix.zero();
      regularizationMatrix.addDiagonal(0, 0, numberOfDoFs, jointAccelerationRegularization.getDoubleValue());
      regularizationMatrix.addDiagonal(numberOfDoFs, numberOfDoFs, problemSize - numberOfDoFs, defaultRhoRegularization);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         wrenchEquilibriumForceError = new YoFrameVector3D("wrenchEquilibriumForceError", null, registry);
         wrenchEquilibriumTorqueError = new YoFrameVector3D("wrenchEquilibriumTorqueError", null, registry);
      }
      else
      {
         wrenchEquilibriumForceError = null;
         wrenchEquilibriumTorqueError = null;
      }

      accelerationVariablesSubstitution.setIgnoreBias(true);

      parentRegistry.addChild(registry);
   }

   private int getNumberOfVariables(QPInputDomain inputDomain)
   {
      switch (inputDomain)
      {
         case MOTION:
            return numberOfDoFs;
         case RHO:
            return rhoSize;
         case MOTION_AND_RHO:
            return numberOfDoFs + rhoSize;
         default:
            throw new RuntimeException("Unknown constraint domain " + inputDomain);
      }
   }

   private int getVariableOffset(QPInputDomain inputDomain)
   {
      switch (inputDomain)
      {
         case MOTION:
         case MOTION_AND_RHO:
            return 0;
         case RHO:
            return numberOfDoFs;
         default:
            throw new RuntimeException("Unknown constraint domain " + inputDomain);
      }
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

   public void setRhoRegularizationWeight(DMatrixRMaj weight)
   {
//      CommonOps_DDRM.insert(weight, regularizationMatrix, numberOfDoFs, numberOfDoFs);

      regularizationMatrix.insert(weight, numberOfDoFs, numberOfDoFs);
   }

   public void setUseWarmStart(boolean useWarmStart)
   {
      this.useWarmStart = useWarmStart;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void notifyResetActiveSet()
   {
      this.resetActiveSet = true;
   }

   private boolean pollResetActiveSet()
   {
      boolean ret = resetActiveSet;
      resetActiveSet = false;
      return ret;
   }

   public void reset()
   {
      regularizationMatrix.fillDiagonal(0, 0, numberOfDoFs, jointAccelerationRegularization.getDoubleValue());

      accelerationVariablesSubstitution.reset();

      solver_H.zero();
      solver_f.zero();

      solver_Aeq.reshape(0, problemSize);
      solver_beq.reshape(0, 1);

      solver_Ain.reshape(0, problemSize);
      solver_bin.reshape(0, 1);

      solverInput_activeIndices.fill(1.0);
   }

   private void addRegularization()
   {
      solver_H.addEquals(regularizationMatrix);

      if (addRateRegularization.getBooleanValue())
      {
         addJointJerkRegularization();
      }
   }

   public void resetRateRegularization()
   {
      addRateRegularization.set(false);
   }

   private void addJointJerkRegularization()
   {
      double factor = dt * dt / jointJerkRegularization.getDoubleValue();
      solver_H.addDiagonal(0, 0, numberOfDoFs, 1.0 / factor);
      solver_f.addBlock(solverOutput_jointAccelerations, 0, 0, 0, 0, numberOfDoFs, 1, -1.0 / factor);
   }

   public void addQPInput(NativeQPInputTypeA input, QPInputDomain inputDomain)
   {
      switch (input.getConstraintType())
      {
         case OBJECTIVE:
            if (input.useWeightScalar())
               addQPTask(input.taskJacobian, input.taskObjective, input.getWeightScalar(), inputDomain);
            else
               addQPTask(input.taskJacobian, input.taskObjective, input.taskWeightMatrix, inputDomain);
            break;
         case EQUALITY:
            addEqualityConstraint(input.taskJacobian, input.taskObjective, inputDomain);
            break;
         case LEQ_INEQUALITY:
            addLesserOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective, inputDomain);
            break;
         case GEQ_INEQUALITY:
            addGreaterOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective, inputDomain);
            break;
         default:
            throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }

   public void addQPInput(NativeQPInputTypeB input, QPInputDomain inputDomain)
   {
      if (input.useWeightScalar())
         addQPTask(input.taskJacobian, input.taskConvectiveTerm, input.getWeightScalar(), input.directCostHessian, input.directCostGradient, inputDomain);
      else
         addQPTask(input.taskJacobian, input.taskConvectiveTerm, input.taskWeightMatrix, input.directCostHessian, input.directCostGradient, inputDomain);
   }

   /**
    * Sets up an objective.
    * <p>
    * min (J x - b)^T * W * (J x - b)
    * </p>
    *
    * @param taskJacobian jacobian to map qddot to the objective space. J in the above equation.
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to bediagonal.
    * @param inputDomain whether the decision variable x is accelerations, rho, or both.
    */
   public void addQPTask(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight, QPInputDomain inputDomain)
   {
      if (taskJacobian.getNumCols() != getNumberOfVariables(inputDomain))
      {
         throw new RuntimeException("Invalid task size. Expected " + getNumberOfVariables(inputDomain) + " but received " + taskJacobian.getNumCols());
      }
      addTaskInternal(taskJacobian, taskObjective, taskWeight, getVariableOffset(inputDomain));
   }

   /**
    * Sets up an objective.
    * <p>
    * min (J x - b)^T * W * (J x - b)
    * </p>
    *
    * @param taskJacobian jacobian to map qddot to the objective space. J in the above equation.
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be diagonal.
    * @param inputDomain whether the decision variable x is accelerations, rho, or both.
    */
   public void addQPTask(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight, QPInputDomain inputDomain)
   {
      if (taskJacobian.getNumCols() != getNumberOfVariables(inputDomain))
      {
         throw new RuntimeException("Invalid task size. Expected " + getNumberOfVariables(inputDomain) + " but received " + taskJacobian.getNumCols());
      }
      addTaskInternal(taskJacobian, taskObjective, taskWeight, getVariableOffset(inputDomain));
   }

   public void addQPTask(NativeMatrix taskJacobian,
                         NativeMatrix taskConvectiveTerm,
                         double taskWeight,
                         NativeMatrix directCostHessian,
                         NativeMatrix directCostGradient,
                         QPInputDomain inputDomain)
   {
      if (taskJacobian.getNumCols() != getNumberOfVariables(inputDomain))
      {
         throw new RuntimeException("Invalid task size. Expected " + getNumberOfVariables(inputDomain) + " but received " + taskJacobian.getNumCols());
      }
      addTaskInternal(taskJacobian, taskConvectiveTerm, taskWeight,directCostHessian, directCostGradient, getVariableOffset(inputDomain));
   }

   public void addQPTask(NativeMatrix taskJacobian,
                         NativeMatrix taskConvectiveTerm,
                         NativeMatrix taskWeight,
                         NativeMatrix directCostHessian,
                         NativeMatrix directCostGradient,
                         QPInputDomain inputDomain)
   {
      if (taskJacobian.getNumCols() != getNumberOfVariables(inputDomain))
      {
         throw new RuntimeException("Invalid task size. Expected " + getNumberOfVariables(inputDomain) + " but received " + taskJacobian.getNumCols());
      }
      addTaskInternal(taskJacobian, taskConvectiveTerm, taskWeight, directCostHessian, directCostGradient, getVariableOffset(inputDomain));
   }

   /**
    * Sets up an objective similarly to {@link #addQPTask} but with an identity Jacobian, J=I:
    * <p>
    * min (x - b)^T * W * (x - b)
    * </p>
    *
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be diagonal.
    * @param inputDomain whether the decision variable x is accelerations, rho, or both.
    */
   public void addIdentityJacobianTask(DMatrixRMaj taskObjective, DMatrixRMaj taskWeight, QPInputDomain inputDomain)
   {
      tempObjective.set(taskObjective);
      tempWeight.set(taskWeight);
      addTaskInternal(tempObjective, tempWeight, getVariableOffset(inputDomain), getNumberOfVariables(inputDomain));
   }

   public void addTaskInternal(NativeMatrix taskObjective, NativeMatrix taskWeight, int offset, int variables)
   {
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      solver_H.addBlock(taskWeight, offset, offset, 0, 0, variables, variables);
      solver_f.multAddBlock(-1.0, taskWeight, taskObjective, offset, 0);
   }

   private void addTaskInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight, int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // J^T W
      tempJtW.multTransA(taskJacobian, taskWeight);

      // Compute: H += J^T W J
      solver_H.multAddBlock(tempJtW, taskJacobian, offset, offset);

      // Compute: f += - J^T W Objective
      solver_f.multAddBlock(-1.0, tempJtW, taskObjective, offset, 0);
   }

   private void addTaskInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight, int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: H += J^T W J
      solver_H.multAddBlockTransA(taskWeight, taskJacobian, taskJacobian, offset, offset);

      // Compute: f += - J^T W Objective
      solver_f.multAddBlockTransA(-taskWeight, taskJacobian, taskObjective, offset, 0);
   }

   private void addTaskInternal(NativeMatrix taskJacobian,
                                NativeMatrix taskConvectiveTerm,
                                NativeMatrix taskWeight,
                                NativeMatrix directCostHessian,
                                NativeMatrix directCostGradient,
                                int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // J^T Q
      tempJtW.multTransA(taskJacobian, taskWeight);

      // Compute: f += J^T Q g
      solver_f.multAddBlock(tempJtW, directCostGradient, offset, 0);

      // J^T (Q + H)
      tempJtW.multAddTransA(taskJacobian, directCostHessian);

      // Compute: H += J^T (H + Q) J
      solver_H.multAddBlock(tempJtW, taskJacobian, offset, offset);

      // Compute: f += J^T (Q + H) b
      solver_f.multAddBlock(tempJtW, taskConvectiveTerm, offset, 0);
   }

   private void addTaskInternal(NativeMatrix taskJacobian,
                                NativeMatrix taskConvectiveTerm,
                                double taskWeight,
                                NativeMatrix directCostHessian,
                                NativeMatrix directCostGradient,
                                int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: f += J^T W g
      solver_f.multAddBlock(taskWeight, taskJacobian, directCostGradient, offset, 0);

      // J^T (Q + H)
      tempJtW.multTransA(taskWeight, taskJacobian, directCostHessian);

      // Compute: H += J^T (H + Q) J
      solver_H.multAddBlock(tempJtW, taskJacobian, offset, offset);

      // Compute: f += J^T (Q + H) b
      solver_f.multAddBlock(tempJtW, taskConvectiveTerm, offset, 0);
   }

   public void addEqualityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, QPInputDomain inputDomain)
   {
      if (taskJacobian.getNumCols() != getNumberOfVariables(inputDomain))
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      addEqualityConstraintInternal(taskJacobian, taskObjective, getVariableOffset(inputDomain));
   }

   private void addEqualityConstraintInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solver_beq.getNumRows();

      solver_Aeq.growRows(taskSize);
      solver_Aeq.insert(taskJacobian, previousSize, offset);

      solver_beq.growRows(taskSize);
      solver_beq.insert(taskObjective, previousSize, 0);
   }

   public void addLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, QPInputDomain inputDomain)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, 1.0, inputDomain);
   }

   public void addGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, QPInputDomain inputDomain)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, -1.0, inputDomain);
   }

   private void addInequalityConstraintInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, double sign, QPInputDomain inputDomain)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, sign, getVariableOffset(inputDomain));
   }

   private void addInequalityConstraintInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, double sign, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solver_bin.getNumRows();

      solver_Ain.growRows(taskSize);
      solver_Ain.insertScaled(taskJacobian, previousSize, offset, sign);

      solver_bin.growRows(taskSize);
      solver_bin.insertScaled(taskObjective, previousSize, 0, sign);
   }

   public void addTorqueMinimizationObjective(NativeMatrix torqueJacobian, NativeMatrix torqueObjective)
   {
      // Compute: H += J^T W J
      solver_H.multAddTransA(jointTorqueWeight.getDoubleValue(), torqueJacobian, torqueJacobian);

      // Compute: f += - J^T W Objective
      solver_f.multAddTransA(-jointTorqueWeight.getDoubleValue(), torqueJacobian, torqueObjective);
   }

   public void addTorqueMinimizationObjective(DMatrixRMaj torqueQddotJacobian, DMatrixRMaj torqueRhoJacobian, DMatrixRMaj torqueObjective)
   {
      int taskSize = torqueObjective.getNumRows();

      tempJtW.reshape(taskSize, problemSize);
      tempJtW.insert(torqueQddotJacobian, 0, 0);
      tempJtW.insert(torqueRhoJacobian, 0, numberOfDoFs);

      tempObjective.set(torqueObjective);

      addTorqueMinimizationObjective(tempJtW, tempObjective);
   }

   /**
    * Need to be called before {@link #solve()}. It sets up the constraint that ensures that the
    * solution is dynamically feasible:
    * <p>
    * <li>hDot = &sum;W<sub>ext</sub>
    * <li>A * qDDot + ADot * qDot = Q * &rho; + &sum;W<sub>user</sub> + W<sub>gravity</sub>
    * <li>-A * qDDot - ADot * qDot = - Q * &rho; - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * <li>-A * qDDot + Q * &rho; = ADot * qDot - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * <li>[-A Q] * [qDDot<sup>T</sup> &rho;<sup>T</sup>]<sup>T</sup> = ADot * qDot -
    * &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * </p>
    *
    * @param centroidalMomentumMatrix refers to A in the equation.
    * @param rhoJacobian refers to Q in the equation. Q&rho; represents external wrench to
    *       be optimized for.
    * @param convectiveTerm refers to ADot * qDot in the equation.
    * @param additionalExternalWrench refers to &sum;W<sub>user</sub> in the equation. These are
    *       constant wrenches usually used for compensating for the weight of
    *       an object that the robot is holding.
    * @param gravityWrench refers to W<sub>gravity</sub> in the equation. It the wrench
    *       induced by the weight of the robot.
    */
   public void setupWrenchesEquilibriumConstraint(DMatrixRMaj centroidalMomentumMatrix,
                                                  DMatrixRMaj rhoJacobian,
                                                  DMatrixRMaj convectiveTerm,
                                                  DMatrixRMaj additionalExternalWrench,
                                                  DMatrixRMaj gravityWrench)
   {
      if (!hasFloatingBase)
      {
         hasWrenchesEquilibriumConstraintBeenSetup = true;
         return;
      }

      tempWrenchConstraint_RHS.set(convectiveTerm);
      CommonOps_DDRM.subtractEquals(tempWrenchConstraint_RHS, additionalExternalWrench);
      CommonOps_DDRM.subtractEquals(tempWrenchConstraint_RHS, gravityWrench);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         tempWrenchConstraint_J.reshape(Wrench.SIZE, problemSize);
         tempWrenchConstraint_J.insertScaled(centroidalMomentumMatrix, 0, Wrench.SIZE, 0, numberOfDoFs, 0, 0, -1.0);
         tempWrenchConstraint_J.insert(rhoJacobian, 0, numberOfDoFs);

         nativeTempWrenchConstraint_RHS.set(tempWrenchConstraint_RHS);

         double weight = 150.0;
         solver_H.multAddTransA(weight, tempWrenchConstraint_J, tempWrenchConstraint_J);
         solver_f.multAddTransA(-weight, tempWrenchConstraint_J, nativeTempWrenchConstraint_RHS);
      }
      else
      {
         int constraintSize = Wrench.SIZE;
         int previousSize = solver_beq.getNumRows();

         solver_Aeq.growRows(constraintSize);

         solver_Aeq.insertScaled(centroidalMomentumMatrix, 0, constraintSize, 0, numberOfDoFs, previousSize, 0, -1.0);
         solver_Aeq.insert(rhoJacobian, previousSize, numberOfDoFs);

         solver_beq.growRows(constraintSize);
         solver_beq.insert(tempWrenchConstraint_RHS, previousSize, 0);
      }

      hasWrenchesEquilibriumConstraintBeenSetup = true;
   }

   public void addAccelerationSubstitution(NativeQPVariableSubstitution substitution)
   {
      this.accelerationVariablesSubstitution.concatenate(substitution);
   }

   private final NativeMatrix tempWrenchConstraint_J = new NativeMatrix(Wrench.SIZE, 200);
   private final NativeMatrix tempWrenchConstraint_LHS = new NativeMatrix(Wrench.SIZE, 1);
   private final DMatrixRMaj tempWrenchConstraint_RHS = new DMatrixRMaj(Wrench.SIZE, 1);
   private final NativeMatrix nativeTempWrenchConstraint_RHS = new NativeMatrix(Wrench.SIZE, 1);

   public boolean solve()
   {
      if (!hasWrenchesEquilibriumConstraintBeenSetup)
         throw new RuntimeException("The wrench equilibrium constraint has to be setup before calling solve().");

      addRegularization();

      numberOfEqualityConstraints.set(solver_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solver_Ain.getNumRows());
      numberOfConstraints.set(numberOfEqualityConstraints.getIntegerValue() + numberOfInequalityConstraints.getIntegerValue());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setUseWarmStart(useWarmStart);
      qpSolver.setMaxNumberOfIterations(maxNumberOfIterations);
      if (useWarmStart && pollResetActiveSet())
         qpSolver.resetActiveSet();

      TIntArrayList inactiveIndices = applySubstitution(); // This needs to be done right before configuring the QP and solving.

      if (inactiveIndices != null)
      {
         for (int i = 0; i < inactiveIndices.size(); i++)
         {
            solverInput_activeIndices.set(inactiveIndices.get(i), 0, 0.0);
         }
      }

      numberOfActiveVariables.set((int) solverInput_activeIndices.sum());
      qpSolver.setActiveVariables(solverInput_activeIndices);

      numberOfIterations.set(qpSolver.solve(solverOutput));
      removeSubstitution(); // This needs to be done right after solving.

      qpSolverTimer.stopMeasurement();

      hasWrenchesEquilibriumConstraintBeenSetup = false;

      if (solverOutput.containsNaN())
      {
         return false;
      }

      solverOutput_jointAccelerations.insert(solverOutput, 0, numberOfDoFs, 0, 1, 0, 0);
      solverOutput_rhos.insert(solverOutput, numberOfDoFs, problemSize, 0, 1, 0, 0);

      addRateRegularization.set(true);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         if (hasFloatingBase)
         {
            tempWrenchConstraint_LHS.mult(tempWrenchConstraint_J, solverOutput);
            int index = 0;
            wrenchEquilibriumTorqueError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         }
      }

      solver_H_previous.set(solver_H);
      solver_f_previous.set(solver_f);

      solverInput_lb_previous.set(solver_lb);
      solverInput_ub_previous.set(solver_ub);

      return true;
   }

   private TIntArrayList applySubstitution()
   {
      if (accelerationVariablesSubstitution.isEmpty())
         return null;

      accelerationVariablesSubstitution.applySubstitutionToObjectiveFunction(solver_H, solver_f);
      accelerationVariablesSubstitution.applySubstitutionToLinearConstraint(solver_Aeq, solver_beq);
      accelerationVariablesSubstitution.applySubstitutionToLinearConstraint(solver_Ain, solver_bin);
      accelerationVariablesSubstitution.applySubstitutionToBounds(solver_lb, solver_ub, solver_Ain, solver_bin);
      return accelerationVariablesSubstitution.getInactiveIndices();
   }

   private void removeSubstitution()
   {
      if (accelerationVariablesSubstitution.isEmpty())
         return;

      accelerationVariablesSubstitution.removeSubstitutionToSolution(solverOutput);
   }

   public NativeMatrix getJointAccelerations()
   {
      return solverOutput_jointAccelerations;
   }

   public NativeMatrix getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinJointAccelerations(double qDDotMin)
   {
      solver_lb.fillBlock(0, 0, numberOfDoFs, 1, qDDotMin);
   }

   public void setMinJointAccelerations(DMatrixRMaj qDDotMin)
   {
      solver_lb.insert(qDDotMin, 0, 0);
   }

   public void setMaxJointAccelerations(double qDDotMax)
   {
      solver_ub.fillBlock(0, 0, numberOfDoFs, 1, qDDotMax);
   }

   public void setMaxJointAccelerations(DMatrixRMaj qDDotMax)
   {
      solver_ub.insert(qDDotMax, 0, 0);
   }

   public void setMinRho(double rhoMin)
   {
      if (rhoSize > 0)
      {
         solver_lb.fillBlock(numberOfDoFs, 0, problemSize - numberOfDoFs, 1, rhoMin);
      }
   }

   public void setMinRho(DMatrixRMaj rhoMin)
   {
      if (rhoSize > 0)
      {
         solver_lb.insert(rhoMin, numberOfDoFs, 0);
      }
   }

   public void setMaxRho(double rhoMax)
   {
      if (rhoSize > 0)
      {
         solver_ub.fillBlock(numberOfDoFs, 0, problemSize - numberOfDoFs, 1, rhoMax);
      }
   }

   public void setMaxRho(DMatrixRMaj rhoMax)
   {
      if (rhoSize > 0)
      {
         solver_ub.insert(rhoMax, numberOfDoFs, 0);
      }
   }

   public void setActiveDoF(int dofIndex, boolean active)
   {
      solverInput_activeIndices.set(dofIndex, 0, active ? 1.0 : 0.0);
   }

   public void setActiveRhos(DMatrixRMaj activeRhoMatrix)
   {
      solverInput_activeIndices.insert(activeRhoMatrix, numberOfDoFs, 0);
   }

   /**
    * Parameterizes which variable set a task describes.
    */
   public enum QPInputDomain
   {
      /**
       * The task is a function of the desired accelerations q_dd, such that the Jacobian J multiplies the accelerations: <br>
       * J q_dd
       */
      MOTION,

      /**
       * The task is a function of the rho, the generalized contact forces. The Jacobian J multiplies the rho vector: <br>
       * J rho
       */
      RHO,

      /**
       * The task is a function of the desired accelerations and rho, the generalized contact forces. The Jacobian J multiplies the combined acceleration and rho vector: <br>
       * J [q_dd^T rho^T]^T
       */
      MOTION_AND_RHO
   }

}
