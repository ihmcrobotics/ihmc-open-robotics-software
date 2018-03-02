package us.ihmc.commonWalkingControlModules.controlModules.flight;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.AbstractSimpleActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Uses a QP to determine the appropriate whole body inertia rate of change in order to achieve a 
 * desired angular velocity. 
 * <p> This is done by minimizing the error angular velocity energy function with the 
 * various entries of the angular inertia matrix's derivative being the decision variables. Details of 
 * the  problem formulation are given below: </p>
 * 
 * <li> Decision variables: Rate of change of inertia, dI = (dIxx, dIyy, dIzz, dIxy = dIyx, dIyz = dIzy, dIxz = dIzx) in order </li>
 * <li> Inputs: 
 *    <ol>
 *    <li> w<sub>0</sub>: Commanded centroidal angular velocity
 *    <li> w: Current centroidal angular velocity
 *    <li> e: Current error computed as w - w<sub>0</sub> </ol>
 *    <li> I: the current inertia tensor
 * <li> Objective: The objective is to maximize the rate of descent of the Lyapunov function obtained from the error velocity 
 * energy function V = 0.5 * e<sup>T</sup> I e. This ensures that we are taking the largest step to minimize the velocity error
 * dV = e<sup>T<sup> 
 * <li> Constraints: 
 *    <ol> 
 *       <li> Valid inertia tensor constraints: Ensure that the determined inertia rate of change results in a valid inertia 
 *       tensor for the next iteration
 *       <ul>
 *          <li> Positive diagonal elements: The diagonal elements are given a minimum value (e.g. for the x axis Ixx + dt * dIxx > Ixxmin)
 *          <li> Triangle inequality constraints: The resultant diagonal elements must satisfy the triangle inequality 
 *          (e.g. Ixx + Iyy - Izz + dt * (dIxx + dIyy - dIzz) > 0)
 *       </ul>
 *       <li> Rate of change constraints: 1st norm of the decision variables is restricted. This is done by imposing all
 *       sign combinations (2<sup>6</sup>) of the 1st norm inequality constraint. Can use a more efficient implementation.
 *       Note that the off-diagonal elements appear twice in the norm and hence their coefficients have 
 *       twice the numerical value as the diagonal elements
 *       <li> Maximum inertia constraints: The diagonal elements of the inertia tensor are limited in their maximum values
 *       <ul>
 *       </ul>
 *    </ol>
 * <li> Tuning parameters: Regularization weights q<sub>i</sub> formulated into the quadratic cost Q = diag(q<sub>i</sub>)
 * @author Apoorv Shrivastava
 */

// TODO: A lot of the RHS can be computed and stored before hand since they do not change at runtime. Evaluate whether this is preferred
public class FlightWholeBodyAngularVelocityRegulatorQPSolver
{
   private static final int problem_size = 6;
   private static final int angularDimensions = 3;
   // Create large arrays to that dense matrix resizing dosen't need to copy large data at runtime
   private static final int initialConstraintVectorSize = 100;
   private final ExecutionTimer qpTimer;
   private final AbstractSimpleActiveSetQPSolver qpSolver;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;
   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;
   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;

   private final DenseMatrix64F qpSolution;

   private ReferenceFrame controlFrame;
   private final DenseMatrix64F currentVelocity;
   private final DenseMatrix64F commandedVelocity;
   private final DenseMatrix64F inertia;
   private final DenseMatrix64F velocityError;
   private final DenseMatrix64F deltaControl;

   private final DenseMatrix64F regularizationWeights;
   private final double controllerDT;
   private double minPrincipalInertia;
   private double maxPrincipalInertia;
   private double maxInertiaRateOfChange;

   private final DenseMatrix64F tempConstraintMatrixLHS = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempConstraintMatrixRHS = new DenseMatrix64F(0, 1);

   public FlightWholeBodyAngularVelocityRegulatorQPSolver(double controllerDT, YoVariableRegistry registry)
   {
      qpTimer = new ExecutionTimer("AngularVelocityRegu", registry);
      qpSolver = new JavaQuadProgSolver();
      this.controllerDT = controllerDT;
      solverInput_H = new DenseMatrix64F(problem_size, problem_size);
      solverInput_f = new DenseMatrix64F(problem_size, 1);
      solverInput_Aeq = new DenseMatrix64F(initialConstraintVectorSize, problem_size);
      solverInput_beq = new DenseMatrix64F(initialConstraintVectorSize, 1);
      solverInput_Ain = new DenseMatrix64F(initialConstraintVectorSize, problem_size);
      solverInput_bin = new DenseMatrix64F(initialConstraintVectorSize, 1);
      solverInput_lb = new DenseMatrix64F(problem_size, 1);
      solverInput_ub = new DenseMatrix64F(problem_size, 1);
      qpSolution = new DenseMatrix64F(problem_size, 1);

      currentVelocity = new DenseMatrix64F(angularDimensions, 1);
      commandedVelocity = new DenseMatrix64F(angularDimensions, 1);
      inertia = new DenseMatrix64F(angularDimensions, angularDimensions);
      velocityError = new DenseMatrix64F(angularDimensions, 1);
      deltaControl = new DenseMatrix64F(angularDimensions, 1);

      regularizationWeights = new DenseMatrix64F(problem_size, problem_size);
      reset();
   }

   public void initialize(ReferenceFrame controlFrame)
   {
      this.controlFrame = controlFrame;
   }

   public void reset()
   {
      qpSolver.resetActiveConstraints();
      solverInput_H.reshape(problem_size, problem_size);
      solverInput_f.reshape(problem_size, 1);
      solverInput_ub.reshape(problem_size, 1);
      solverInput_lb.reshape(problem_size, 1);
      solverInput_Aeq.reshape(0, 1);
      solverInput_beq.reshape(0, 1);
      solverInput_Ain.reshape(0, 1);
      solverInput_bin.reshape(0, 1);
      regularizationWeights.reshape(problem_size, problem_size);
   }

   public void compute()
   {
      computeCostFunctionToOptimize();
      computeLinearInequalityConstraints();
      computeLinearEqualityConstraints();
      setupBounds();
      solve();
   }

   public void getDesiredInertiaRateOfChange(DenseMatrix64F inertiaRateOfChange)
   {
      // Map the decision variables to the inertia tensor
      for (int i = 0; i < angularDimensions; i++)
         inertiaRateOfChange.set(i, i, qpSolution.get(i, 0));
      for (int i = angularDimensions; i < problem_size; i++)
      {
         inertiaRateOfChange.set(i % angularDimensions, (i + 1) % angularDimensions, qpSolution.get(i, 0));
         inertiaRateOfChange.set((i + 1) % angularDimensions, i % angularDimensions, qpSolution.get(i, 0));
      }
   }

   public void setRegularizationWeight(double scalar)
   {
      regularizationWeights.zero();
      for (int i = 0; i < problem_size; i++)
         regularizationWeights.set(i, i, scalar);
   }

   public void setRegularizationWeights(double... scalars)
   {
      if (scalars.length == 0)
         throw new RuntimeException(getClass().getSimpleName() + ": Invalid regularization weights");
      else if (scalars.length == 1)
         setRegularizationWeight(scalars[0]);
      else
      {
         regularizationWeights.zero();
         for (int i = 0; i < problem_size; i++)
            regularizationWeights.set(i, i, scalars[i]);
      }
   }

   public void setRegularizationWeights(DenseMatrix64F weightMatrix)
   {
      if (weightMatrix.getNumCols() != problem_size && weightMatrix.getNumRows() != problem_size)
         return;
      else
         regularizationWeights.set(weightMatrix);
   }

   public void setVelocityCommand(FrameVector3D desiredCoMAngularVelocity)
   {
      controlFrame.checkReferenceFrameMatch(desiredCoMAngularVelocity);
      desiredCoMAngularVelocity.get(commandedVelocity);
   }

   public void setCurrentVelocityEstimate(FrameVector3D currentCoMAngularVelocity)
   {
      controlFrame.checkReferenceFrameMatch(currentCoMAngularVelocity);
      currentCoMAngularVelocity.get(currentVelocity);
   }

   public void setCurrentCentroidalInertiaTensor(DenseMatrix64F centroidalInertia)
   {
      // Map the inertia tensor to the decision variables 
      //      this.inertia.reshape(problem_size, 1);
      //      for (int i = 0; i < problem_size; i++)
      //         inertia.set(i, 0, centroidalInertia.get(i % angularDimensions, (i + 1) % angularDimensions));
      inertia.set(centroidalInertia);
   }

   public void setMaxInertiaRateOfChange(double inertiaRateOfChange)
   {
      this.maxInertiaRateOfChange = inertiaRateOfChange;
   }

   public void setMaxPrincipalInertia(double maxPrincipalInertia)
   {
      this.maxPrincipalInertia = maxPrincipalInertia;
   }

   public void setMinPrincipalInertia(double minPrincipalInertia)
   {
      this.minPrincipalInertia = minPrincipalInertia;
   }

   public ReferenceFrame getControlFrame()
   {
      return controlFrame;
   }

   /**
    * This function computes the Lyapunov derivative and stores it in the linear portion of the cost to optimize
    * The quadratic portion of the cost is set to the regularization terms
    */
   private void computeCostFunctionToOptimize()
   {
      // Since the quadratic term is only a result of the regularization set it directly 
      // TODO: Evaluate whether each of the terms deserve to have their own independent regularization weight
      solverInput_H.zero();
      CommonOps.addEquals(solverInput_H, regularizationWeights);

      // Compute the terms required for the linear term (this is from the Lyapunov derivative
      CommonOps.subtract(commandedVelocity, currentVelocity, velocityError);
      for (int i = 0; i < angularDimensions; i++)
         deltaControl.set(i, 0, (3 * currentVelocity.get(i, 0) - commandedVelocity.get(i, 0)) * 0.5);

      // Compute the linear terms 
      for (int i = 0; i < angularDimensions; i++)
      {
         solverInput_f.set(i, 0, velocityError.get(i, 0) * deltaControl.get(i, 0));
         solverInput_f.set(i + angularDimensions, 0, velocityError.get(i, 0) * deltaControl.get((i + 1) % angularDimensions, 0)
               + velocityError.get((i + 1) % angularDimensions, 0) * deltaControl.get(i, 0));
      }
   }

   private void setupBounds()
   {
      for (int i = 0; i < problem_size; i++)
      {
         solverInput_lb.set(i, Double.NEGATIVE_INFINITY);
         solverInput_ub.set(i, Double.POSITIVE_INFINITY);
      }
   }

   private void computeLinearInequalityConstraints()
   {
      setDiagonalElementNonnegativityConstraints();
      setOffDiagonalElementConstraints();
      setDiagonalElementMaxConstraints();
      setRateLimitConstraint();
   }

   private void setDiagonalElementNonnegativityConstraints()
   {
      int currentConstraintSize = solverInput_Ain.getNumRows();

      solverInput_Ain.reshape(currentConstraintSize + angularDimensions, problem_size);
      solverInput_bin.reshape(currentConstraintSize + angularDimensions, 1);

      tempConstraintMatrixLHS.reshape(angularDimensions, problem_size);
      tempConstraintMatrixRHS.reshape(angularDimensions, 1);

      tempConstraintMatrixLHS.zero();
      for (int i = 0; i < angularDimensions; i++)
      {
         tempConstraintMatrixLHS.set(i, i, -controllerDT);
         tempConstraintMatrixRHS.set(i, 0, inertia.get(i, i) - minPrincipalInertia);
      }
      CommonOps.insert(tempConstraintMatrixLHS, solverInput_Ain, currentConstraintSize, 0);
      CommonOps.insert(tempConstraintMatrixRHS, solverInput_bin, currentConstraintSize, 0);
   }

   // Note: This also imposes the triangle inequality constraints on the 
   private void setOffDiagonalElementConstraints()
   {
      int currentConstraintSize = solverInput_Ain.getNumRows();

      solverInput_Ain.reshape(currentConstraintSize + angularDimensions, problem_size);
      solverInput_bin.reshape(currentConstraintSize + angularDimensions, 1);

      tempConstraintMatrixLHS.reshape(angularDimensions, problem_size);
      tempConstraintMatrixRHS.reshape(angularDimensions, 1);

      tempConstraintMatrixLHS.zero();
      for (int i = 0; i < angularDimensions; i++)
      {
         tempConstraintMatrixLHS.set(i, i, -controllerDT);
         tempConstraintMatrixLHS.set(i, (i + 1) % angularDimensions + angularDimensions, controllerDT * 2);
         tempConstraintMatrixRHS.set(i, 0, inertia.get(i, i) - 2 * inertia.get((i + 1) % angularDimensions, (i + 2) % angularDimensions));
      }
      CommonOps.insert(tempConstraintMatrixLHS, solverInput_Ain, currentConstraintSize, 0);
      CommonOps.insert(tempConstraintMatrixRHS, solverInput_bin, currentConstraintSize, 0);
   }

   private void setDiagonalElementMaxConstraints()
   {
      int currentConstraintSize = solverInput_Ain.getNumRows();

      solverInput_Ain.reshape(currentConstraintSize + angularDimensions, problem_size);
      solverInput_bin.reshape(currentConstraintSize + angularDimensions, 1);

      tempConstraintMatrixLHS.reshape(angularDimensions, problem_size);
      tempConstraintMatrixRHS.reshape(angularDimensions, 1);

      tempConstraintMatrixLHS.zero();
      for (int i = 0; i < angularDimensions; i++)
      {
         tempConstraintMatrixLHS.set(i, i, controllerDT);
         tempConstraintMatrixRHS.set(i, 0, maxPrincipalInertia - inertia.get(i, i));
      }
      CommonOps.insert(tempConstraintMatrixLHS, solverInput_Ain, currentConstraintSize, 0);
      CommonOps.insert(tempConstraintMatrixRHS, solverInput_bin, currentConstraintSize, 0);
   }

   private void setRateLimitConstraint()
   {
      int currentConstraintSize = solverInput_Ain.getNumRows();
      int taskSize = (int) Math.pow(2.0, problem_size); // Why is this so large :/

      solverInput_Ain.reshape(currentConstraintSize + taskSize, problem_size);
      solverInput_bin.reshape(currentConstraintSize + taskSize, 1);

      tempConstraintMatrixLHS.reshape(taskSize, problem_size);
      tempConstraintMatrixRHS.reshape(taskSize, 1);

      tempConstraintMatrixLHS.zero();
      for (int i = 0; i < taskSize; i++)
      {
         int n = i;
         for (int j = 0; j < angularDimensions; j++)
         {
            tempConstraintMatrixLHS.set(i, j, Math.pow(-1, n % 2) * 1.0);
            n = n / 2;
         }
         for (int j = angularDimensions; j < problem_size; j++)
         {
            tempConstraintMatrixLHS.set(i, j, Math.pow(-1, n % 2) * 2.0);
            n = n / 2;
         }
         tempConstraintMatrixRHS.set(i, 0, maxInertiaRateOfChange);
      }
      CommonOps.insert(tempConstraintMatrixLHS, solverInput_Ain, currentConstraintSize, 0);
      CommonOps.insert(tempConstraintMatrixRHS, solverInput_bin, currentConstraintSize, 0);
   }

   private void computeLinearEqualityConstraints()
   {
      // No equality constraints. May be useful later
      solverInput_Aeq.reshape(0, problem_size);
      solverInput_beq.reshape(0, 1);
   }

   private String printProblem()
   {
      String ret = "Minimize " + solverInput_H.toString() + " " + solverInput_f.toString() + " subject to: " + solverInput_Aeq.toString() + " "
            + solverInput_beq.toString() + solverInput_Ain.toString() + " " + solverInput_bin.toString() + ", bounds: " + solverInput_lb.toString() + solverInput_ub.toString();

      return ret;
   }

   private void solve()
   {
      qpSolver.clear();
      qpTimer.startMeasurement();

      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLowerBounds(solverInput_lb);
      qpSolver.setUpperBounds(solverInput_ub);

      qpSolver.solve(qpSolution);
      qpTimer.stopMeasurement();
   }
}
