package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.DenseMatrixBool;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.*;
import us.ihmc.convexOptimization.quadraticProgram.*;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.exceptions.NoConvergenceException;

/**
 * Class that sets up the actual optimization framework and handles the inputs to generate an optimized solution
 * designed to stabilize ICP based walking trajectories using both CMP feedback and step adjustment. Designed to
 * work inside the {@link ICPAdjustmentOptimizationController}.
 */
public class ICPQPOptimizationSolver
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double deltaInside = 0.0001;

   /** Index handler that manages the indices for the objectives and solutions in the quadratic program. */
   private final ICPQPIndexHandler indexHandler;
   /** Input calculator that formulates the different objectives and handles adding them to the full program. */
   private final ICPQPInputCalculator inputCalculator;

   /**
    * Has the form 0.5 x<sup>T</sup> H x + h x
    */
   /** Total quadratic cost matrix for the quadratic program. */
   private final DenseMatrix64F solverInput_H;
   /** Total linear cost vector for the quadratic program. */
   private final DenseMatrix64F solverInput_h;
   /** Total scalar cost for the quadratic program. */
   private final DenseMatrix64F solverInputResidualCost;

   /**
    * Has the form A<sub>eq</sub> x = b<sub>eq</sub>
    */
   /** Total linear equality constraint matrix for the quadratic program. */
   private final DenseMatrix64F solverInput_Aeq;
   /** Total linear equality constraint objective vector for the quadratic program. */
   private final DenseMatrix64F solverInput_beq;

   /**
    * Has the form A<sub>ineq</sub> x >= b<sub>ineq</sub>
    */
   /** Total linear inequality constraint matrix for the quadratic program. */
   private final DenseMatrix64F solverInput_Aineq;
   /** Total linear inequality constraint objective vector for the quadratic program. */
   private final DenseMatrix64F solverInput_bineq;

   /** Lower bound on the free variables for the quadratic program. */
   private final DenseMatrix64F solverInput_Lb;
   /** Upper bound on the free variables for the quadratic program. */
   private final DenseMatrix64F solverInput_Ub;


   /** QP Objective to minimize the amount of feedback action. Also contains feedback regularization. */
   private final ICPQPInput feedbackTaskInput;
   /** QP Objective to minimize the dynamic relaxation magnitude. */
   private final ICPQPInput dynamicRelaxationTask;
   /** QP Objective to minimize the amount of step adjustment. Also contains step adjustment. */
   private final ICPQPInput footstepTaskInput;
   /** QP Objective to minimize the angular momentum magnitude. */
   private final ICPQPInput angularMomentumMinimizationTask;

   /** Constraint that encodes the recursive ICP dynamics. */
   private final ICPEqualityConstraintInput dynamicsConstraintInput;
   /** Constraint on the CoP location to the support polygon. */
   private final ConstraintToConvexRegion copLocationConstraint;
   /** Constraint on the footstep location to the reachable region. */
   private final ConstraintToConvexRegion reachabilityConstraint;

   /** List of recursion multipliers for the steps that encode the recursive ICP dynamics. */
   private final ArrayList<DenseMatrix64F> footstepRecursionMultipliers = new ArrayList<>();
   /** Location of the desired footsteps from a high level footstep planner. */
   private final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();

   /** Recursion of the final ICP location for the recursive ICP dynamics. */
   private final DenseMatrix64F finalICPRecursion = new DenseMatrix64F(2, 1);
   /** Effects of the CMP offsets in the upcoming feet and the stance CMP locations in the recursive ICP dynamics. */
   private final DenseMatrix64F cmpConstantEffect = new DenseMatrix64F(2, 1);
   /** Current ICP location. */
   private final DenseMatrix64F currentICP = new DenseMatrix64F(2, 1);
   /** Reference CMP from the ICP plan. */
   private final DenseMatrix64F perfectCMP = new DenseMatrix64F(2, 1);

   /** List of weights for tracking the different footsteps. */
   private final ArrayList<DenseMatrix64F> footstepWeights = new ArrayList<>();
   /** Weight for the footstep regularization task. */
   private final DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);
   /** Weight minimizing the CMP feedback action. */
   private final DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
   /** Weight regularizing the CMP feedback action. */
   private final DenseMatrix64F feedbackRegularizationWeight = new DenseMatrix64F(2, 2);
   /** Weight minimizing the dynamic relaxation magnitude. */
   private final DenseMatrix64F dynamicRelaxationWeight = new DenseMatrix64F(2, 2);
   /** Weight minimizing the angular momentum magnitude. */
   private final DenseMatrix64F angularMomentumMinimizationWeight = new DenseMatrix64F(2, 2);
   /** Proportional gain on the ICP feedback controller. */
   private final DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);

   /** Flag to use the quad prog QP solver vs. the active set QP solver. **/
   private final JavaQuadProgSolver solver = new JavaQuadProgSolver();

   /** Full solution vector to the quadratic program. */
   private final DenseMatrix64F solution;
   /** Footstep location solution vector to the quadratic program. */
   private final DenseMatrix64F footstepLocationSolution;
   /** Feedback action solution to the quadratic program. */
   private final DenseMatrix64F feedbackDeltaSolution;
   /** Dynamic relaxation solution to the quadratic program. */
   private final DenseMatrix64F dynamicRelaxationSolution;
   /** Angular momentum solution to the quadratic program. */
   private final DenseMatrix64F angularMomentumSolution;

   /** Previous solution for the feedback action, used in the feedback regularization objective. */
   private final DenseMatrix64F previousFeedbackDeltaSolution;
   private final ArrayList<DenseMatrix64F> previousFootstepLocations = new ArrayList<>();

   /** Cost to go for the entire quadratic program. */
   private final DenseMatrix64F costToGo;
   /** Cost to go for the step adjustment minimization objective. */
   private final DenseMatrix64F footstepCostToGo;
   /** Cost to go for the feedback action minimization objective. */
   private final DenseMatrix64F feedbackCostToGo;
   /** Cost to go for the dynamic reachability minimization objective. */
   private final DenseMatrix64F dynamicRelaxationCostToGo;
   /** Cost to go for the angular momentum minimization objective. */
   private final DenseMatrix64F angularMomentumMinimizationCostToGo;

   /** Maximum number of footsteps that the quadratic program will ever consider. Used for clearing and storing data. */
   private final int maximumNumberOfFootstepsToConsider;
   /** Maximum number of vertices in the reachability polygon that the quadratic program will ever consider. Used for clearing and storing data. */
   private static final int maximumNumberOfReachabilityVertices = 4;

   /** Number of iterations required for the active set solver to find a solution. */
   private int numberOfIterations;

   private int currentEqualityConstraintIndex;
   private int currentInequalityConstraintIndex;

   /** boolean to determine whether or not to compute the cost to go. specified at compile time. */
   private final boolean computeCostToGo;
   private final boolean autoSetPreviousSolution;

   /** boolean indicating whether or not the footstep regularization term has been added and can be used. */
   private boolean hasFootstepRegularizationTerm = false;
   /** boolean indicating whether or not the feedback regularization term has been added and can be used. */
   private boolean hasFeedbackRegularizationTerm = false;

   /** Minimum allowable weight on the step adjustment task. */
   private final double minimumFootstepWeight;
   /** Minimum allowable weight on the feedback task. */
   private final double minimumFeedbackWeight;

   private final FramePoint tmpPoint = new FramePoint();
   private final DenseMatrix64F tmpCost;
   private final DenseMatrix64F tmpFootstepCost;
   private final DenseMatrix64F tmpFeedbackCost;

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);


   /**
    * Creates the ICP Optimization Solver. Refer to the class documentation: {@link ICPQPOptimizationSolver}.
    *
    * @param icpOptimizationParameters parameters to be used by in the optimization.
    * @param maximumNumberOfCMPVertices maximum number of vertices to be considered by the CoP location constraint.
    * @param computeCostToGo whether or not to compute the cost to go.
    */
   public ICPQPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices, boolean computeCostToGo)
   {
      this(icpOptimizationParameters, maximumNumberOfCMPVertices, computeCostToGo, true);
   }

   public ICPQPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices, boolean computeCostToGo,
         boolean autoSetPreviousSolution)
   {
      this.computeCostToGo = computeCostToGo;
      this.autoSetPreviousSolution = autoSetPreviousSolution;

      indexHandler = new ICPQPIndexHandler();
      inputCalculator = new ICPQPInputCalculator(indexHandler);

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();

      minimumFootstepWeight = icpOptimizationParameters.getMinimumFootstepWeight();
      minimumFeedbackWeight = icpOptimizationParameters.getMinimumFeedbackWeight();

      int maximumNumberOfFreeVariables = 2 * maximumNumberOfFootstepsToConsider + maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices + 4;
      int maximumNumberOfLagrangeMultipliers = 8;

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      solverInputResidualCost = new DenseMatrix64F(1, 1);

      feedbackTaskInput = new ICPQPInput(2);
      footstepTaskInput = new ICPQPInput(2 * maximumNumberOfFootstepsToConsider);
      dynamicRelaxationTask = new ICPQPInput(2);
      angularMomentumMinimizationTask = new ICPQPInput(2);

      dynamicsConstraintInput = new ICPEqualityConstraintInput(maximumNumberOfFreeVariables);
      copLocationConstraint = new ConstraintToConvexRegion(maximumNumberOfCMPVertices);
      reachabilityConstraint = new ConstraintToConvexRegion(maximumNumberOfReachabilityVertices);

      solverInput_Aeq = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, maximumNumberOfFreeVariables);
      solverInput_beq = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);

      solverInput_Aineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices);
      solverInput_bineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, 1);

      solverInput_Lb = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      solverInput_Ub = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      CommonOps.fill(solverInput_Lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_Ub, Double.POSITIVE_INFINITY);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.add(new DenseMatrix64F(2, 1));
         previousFootstepLocations.add(new DenseMatrix64F(2, 1));

         footstepRecursionMultipliers.add(new DenseMatrix64F(2, 2));
         footstepWeights.add(new DenseMatrix64F(2, 2));
      }

      solution = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      footstepLocationSolution = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      feedbackDeltaSolution = new DenseMatrix64F(2, 1);
      dynamicRelaxationSolution = new DenseMatrix64F(2, 1);
      angularMomentumSolution = new DenseMatrix64F(2, 1);

      previousFeedbackDeltaSolution = new DenseMatrix64F(2, 1);

      tmpCost = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      tmpFootstepCost = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      tmpFeedbackCost = new DenseMatrix64F(2, 1);
      costToGo = new DenseMatrix64F(1, 1);
      footstepCostToGo = new DenseMatrix64F(1, 1);
      feedbackCostToGo = new DenseMatrix64F(1, 1);
      dynamicRelaxationCostToGo = new DenseMatrix64F(1, 1);
      angularMomentumMinimizationCostToGo = new DenseMatrix64F(1, 1);

      /*
      if (!useQuadProg)
         activeSetSolver.setUseWarmStart(icpOptimizationParameters.useWarmStartInSolver());
      */
   }

   /**
    * Resets the constraint on the CoP location. This constraint requires that the CoP not exit the convex hull of the support polygon.
    */
   public void resetCoPLocationConstraint()
   {
      copLocationConstraint.reset();
   }


   /**
    * Adds a vertex of the support polygon to the CoP location constraint. The CoP is constrained to the inside of the convex hull described
    * by these vertices. The vertex location is offset in the x and y directions in the {@param frame} reference frame by the parameters {@param xBuffer} and
    * {@param yBuffer}, respectively.
    *
    * @param vertexLocation location of the vertex.
    * @param frame reference frame of the support polygon.
    * @param xBuffer offset of the vertex in the x direction in the {@param frame} reference frame.
    * @param yBuffer offset of the vetex in the y direction in the {@param frame} reference frame.
    */
   public void addSupportPolygonVertex(FramePoint2d vertexLocation, ReferenceFrame frame, double xBuffer, double yBuffer)
   {
      tmpPoint.setToZero(frame);
      tmpPoint.setXY(vertexLocation);

      if (tmpPoint.getX() > 0.0)
         tmpPoint.setX(tmpPoint.getX() + xBuffer);
      else
         tmpPoint.setX(tmpPoint.getX() - xBuffer);

      if (tmpPoint.getY() > 0.0)
         tmpPoint.setY(tmpPoint.getY() + yBuffer);
      else
         tmpPoint.setY(tmpPoint.getY() - yBuffer);

      tmpPoint.changeFrame(worldFrame);

      copLocationConstraint.addVertex(tmpPoint);
   }

   /**
    * Adds a convex polygon to the CoP location constraint. The CoP is constrained to the inside of the convex hull described
    * by this polygon.
    *
    * @param polygon polygon to add.
    */
   public void addSupportPolygon(FrameConvexPolygon2d polygon)
   {
      polygon.changeFrame(worldFrame);
      copLocationConstraint.addPolygon(polygon);
   }

   /**
    * Resets the reachability constraint on the upcoming footstep location. This constraint requires that the footstep lies in the convex hull of this polygon.
    */
   public void resetReachabilityConstraint()
   {
      reachabilityConstraint.reset();
   }

   /**
    * Adds a vertex of the reachability region to the reachability constraint. The footstep is constrained to the inside of the convex hull
    * described by these vertices.
    *
    * @param vertexLocation location of the vertex.
    * @param frame reference frame of the support polygon.
    */
   public void addReachabilityVertex(FramePoint2d vertexLocation, ReferenceFrame frame)
   {
      tmpPoint.setToZero(frame);
      tmpPoint.setXY(vertexLocation);
      tmpPoint.changeFrame(worldFrame);

      reachabilityConstraint.addVertex(tmpPoint);
   }

   public void addReachabilityPolygon(FrameConvexPolygon2d polygon)
   {
      polygon.changeFrame(worldFrame);
      reachabilityConstraint.addPolygon(polygon);
   }

   /**
    * Zeros all the pertinent scalars, vectors, and matrices for the solver. Should be called at the beginning of every computation tick.
    */
   private void reset()
   {
      solverInput_H.zero();
      solverInput_h.zero();
      solverInputResidualCost.zero();

      solverInput_Aeq.zero();
      solverInput_beq.zero();

      solverInput_Aineq.zero();
      solverInput_bineq.zero();

      dynamicsConstraintInput.reset();

      dynamicRelaxationTask.reset();
      angularMomentumMinimizationTask.reset();
      footstepTaskInput.reset();
      feedbackTaskInput.reset();

      finalICPRecursion.zero();
      cmpConstantEffect.zero();
      currentICP.zero();
      perfectCMP.zero();

      solution.zero();
      footstepLocationSolution.zero();
      feedbackDeltaSolution.zero();
      dynamicRelaxationSolution.zero();
      angularMomentumSolution.zero();

      currentEqualityConstraintIndex = 0;
      currentInequalityConstraintIndex = 0;
   }

   /**
    * Reshapes all the vectors and matrices to the appropriate size, based on the number of footsteps to handle. Should be called after {@link #reset()}
    * at the beginning of every computation tick.
    */
   private void reshape()
   {
      int problemSize = indexHandler.getNumberOfFreeVariables();
      int numberOfEqualityConstraints = indexHandler.getNumberOfEqualityConstraints();
      int numberOfFootstepsToConsider = indexHandler.getNumberOfFootstepsToConsider();
      int numberOfInequalityConstraints;

      copLocationConstraint.setPolygon();
      reachabilityConstraint.setPolygon();
      numberOfInequalityConstraints = copLocationConstraint.getInequalityConstraintSize() + reachabilityConstraint.getInequalityConstraintSize();

      solverInput_H.reshape(problemSize, problemSize);
      solverInput_h.reshape(problemSize, 1);

      feedbackTaskInput.reshape(2);
      dynamicRelaxationTask.reshape(2);
      angularMomentumMinimizationTask.reshape(2);
      footstepTaskInput.reshape(2 * numberOfFootstepsToConsider);

      solverInput_Aeq.reshape(numberOfEqualityConstraints, problemSize);
      solverInput_beq.reshape(numberOfEqualityConstraints, 1);

      solverInput_Aineq.reshape(numberOfInequalityConstraints, problemSize);
      solverInput_bineq.reshape(numberOfInequalityConstraints, 1);

      dynamicsConstraintInput.reshape(problemSize);

      solution.reshape(problemSize, 1);
      footstepLocationSolution.reshape(2 * numberOfFootstepsToConsider, 1);
   }

   /**
    * Resets the controller conditions on the feedback minimization task, the feedback gains, and the dynamic relaxation minimization task.
    * Also sets that the controller is not to attempt to regularize the feedback minimization task.
    *
    * Should be called before calling {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)} and before calling
    * {@link #setFeedbackConditions(double, double, double, double, double)} every control tick.
    */
   public void resetFeedbackConditions()
   {
      feedbackWeight.zero();
      feedbackGain.zero();
      dynamicRelaxationWeight.zero();

      hasFeedbackRegularizationTerm = false;
   }

   /**
    * Resets the controller conditions for the minimization of angular momentum task, and also sets it so that the controller will not attempt to utilize
    * angular momentum to stabilize the ICP dynamics.
    *
    * Should be called before calling {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)} and before calling
    * {@link #setAngularMomentumConditions(double, boolean)} every control tick.
    */
   public void resetAngularMomentumConditions()
   {
      angularMomentumMinimizationWeight.zero();
      indexHandler.setUseAngularMomentum(false);
   }

   /**
    * Resets the footstep plan tracking conditions for the controller. This includes resetting all the reference footstep locations and the
    * footstep recursion multipliers. Also sets that the controller is not to attempt to regularize the footstep locations.
    *
    * Should be called before calling {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)} and before calling
    * {@link #setFootstepAdjustmentConditions(int, double, double, double, FramePoint2d)} every control tick.
    */
   public void resetFootstepConditions()
   {
      indexHandler.resetFootsteps();

      footstepRegularizationWeight.zero();

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         footstepRecursionMultipliers.get(i).zero();
         footstepWeights.get(i).zero();
      }

      hasFootstepRegularizationTerm = false;
   }

   /**
    * Sets the conditions for the footstep adjustment task. This includes the weight of tracking the specified footstep by the optimization algorithm,
    * the reference location of the footstep, and the recursion multiplier of that footstep for the ICP dynamics.
    *
    * Should be called after calling {@link #resetFootstepConditions()} and before calling
    * {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)}.
    *
    * @param footstepIndex index of the current footstep.
    * @param recursionMultiplier recursion multiplier for the footstep for the ICP dynamics.
    * @param weight weight on tracking the reference footstep location in the solver.
    * @param referenceFootstepLocation location of the desired reference footstep.
    */
   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double weight, FramePoint2d referenceFootstepLocation)
   {
      this.setFootstepAdjustmentConditions(footstepIndex, recursionMultiplier, weight, weight, referenceFootstepLocation);
   }

   /**
    * Sets the conditions for the footstep adjustment task. This includes the weight of tracking the specified footstep by the optimization algorithm,
    * the reference location of the footstep, and the recursion multiplier of that footstep for the ICP dynamics.
    *
    * Should be called after calling {@link #resetFootstepConditions()} and before calling
    * {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)}.
    *
    * @param footstepIndex index of the current footstep.
    * @param recursionMultiplier recursion multiplier for the footstep for the ICP dynamics.
    * @param xWeight weight on tracking the reference footstep location in the solver in the Cartesian x coordinate.
    * @param yWeight weight on tracking the reference footstep location in the solver in the Cartesian y coordinate.
    * @param referenceFootstepLocation location of the desired reference footstep.
    */
   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double xWeight, double yWeight, FramePoint2d referenceFootstepLocation)
   {
      CommonOps.setIdentity(identity);
      MatrixTools.setMatrixBlock(footstepRecursionMultipliers.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, recursionMultiplier);

      xWeight = Math.max(minimumFootstepWeight, xWeight);
      yWeight = Math.max(minimumFootstepWeight, yWeight);

      identity.zero();
      identity.set(0, 0, xWeight);
      identity.set(1, 1, yWeight);
      MatrixTools.setMatrixBlock(footstepWeights.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, 1.0);

      referenceFootstepLocation.changeFrame(worldFrame);
      referenceFootstepLocations.get(footstepIndex).set(0, 0, referenceFootstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, 0, referenceFootstepLocation.getY());

      indexHandler.registerFootstep();
   }

   /**
    * Sets the conditions for the minimization of the angular momentum task. This includes whether or not to utilize angular momentum to help stabilize
    * the ICP dynamics, as well as the weight on its minimization.
    *
    * Should be called after calling {@link #resetAngularMomentumConditions()} and before calling
    * {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)}.
    *
    * @param angularMomentumMinimizationWeight weight on minimizing angular momentum.
    * @param useAngularMomentum whether or not to use angular momentum in the problem.
    */
   public void setAngularMomentumConditions(double angularMomentumMinimizationWeight, boolean useAngularMomentum)
   {
      CommonOps.setIdentity(identity);

      MatrixTools.setMatrixBlock(this.angularMomentumMinimizationWeight, 0, 0, identity, 0, 0, 2, 2, angularMomentumMinimizationWeight);
      indexHandler.setUseAngularMomentum(useAngularMomentum);
   }

   /**
    * Enables the use of footstep regularization in the solver, and also sets the weight on it. This task minimizes the differences between solutions of the
    * footstep location.
    *
    * @param regularizationWeight weight placed on changes in the footstep location solution.
    */
   public void setFootstepRegularizationWeight(double regularizationWeight)
   {
      CommonOps.setIdentity(footstepRegularizationWeight);
      CommonOps.scale(regularizationWeight, footstepRegularizationWeight);

      hasFootstepRegularizationTerm = true;
   }

   /**
    * Resets the footstep regularization objectives. This is important to call at the start of every new step, if using footstep regularization.
    *
    * @param footstepIndex index of footstep to reset
    * @param previousFootstepLocation new location of the previous footstep location to try and minimize against.
    */
   public void resetFootstepRegularization(int footstepIndex, FramePoint2d previousFootstepLocation)
   {
      previousFootstepLocation.changeFrame(worldFrame);
      previousFootstepLocations.get(footstepIndex).set(0, 0, previousFootstepLocation.getX());
      previousFootstepLocations.get(footstepIndex).set(1, 0, previousFootstepLocation.getY());
   }




   /**
    * Sets the conditions for the feedback minimization task and the dynamic relaxation minimization task. This task minimizes the difference between
    * the nominal CMP location and the one used to control the ICP dynamics. The dynamic relaxation allows the ICP recursive dynamics to be violated by a
    * small magnitude, which is critical to not overconstraining the problem.
    *
    * Should be called before calling after {@link #resetFeedbackConditions()} and before calling
    * {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)}.
    *
    * @param feedbackWeight weight on the minimization of the feedback action for the solver.
    * @param feedbackGain ICP controller proportional gain.
    * @param dynamicRelaxationWeight weight on the minimization of the dynamic relaxation for the solver.
    */
   public void setFeedbackConditions(double feedbackWeight, double feedbackGain, double dynamicRelaxationWeight)
   {
      this.setFeedbackConditions(feedbackWeight, feedbackWeight, feedbackGain, feedbackGain, dynamicRelaxationWeight);
   }

   /**
    * Sets the conditions for the feedback minimization task and the dynamic relaxation minimization task. This task minimizes the difference between
    * the nominal CMP location and the one used to control the ICP dynamics. The dynamic relaxation allows the ICP recursive dynamics to be violated by a
    * small magnitude, which is critical to not overconstraining the problem.
    *
    * Should be called before calling after {@link #resetFeedbackConditions()} and before calling
    * {@link #compute(FramePoint2d, FramePoint2d, FramePoint2d, FramePoint2d)}.
    *
    * @param feedbackXWeight weight on the minimization of the feedback action for the solver in the Cartesian x coordinate direction.
    * @param feedbackYWeight weight on the minimization of the feedback action for the solver in the Cartesian y coordinate direction.
    * @param feedbackXGain ICP controller proportional gain in the Cartesian x coordinate direction.
    * @param feedbackYGain ICP controller proportional gain in the Cartesian y coordinate direction.
    * @param dynamicRelaxationWeight weight on the minimization of the dynamic relaxation for the solver.
    */
   public void setFeedbackConditions(double feedbackXWeight, double feedbackYWeight, double feedbackXGain, double feedbackYGain, double dynamicRelaxationWeight)
   {
      feedbackXWeight = Math.max(feedbackXWeight, minimumFeedbackWeight);
      feedbackYWeight = Math.max(feedbackYWeight, minimumFeedbackWeight);

      this.feedbackWeight.zero();
      this.feedbackWeight.set(0, 0, feedbackXWeight);
      this.feedbackWeight.set(1, 1, feedbackYWeight);

      this.feedbackGain.zero();
      this.feedbackGain.set(0, 0, feedbackXGain);
      this.feedbackGain.set(1, 1, feedbackYGain);

      CommonOps.setIdentity(this.dynamicRelaxationWeight);
      CommonOps.scale(dynamicRelaxationWeight, this.dynamicRelaxationWeight);
   }

   /**
    * Enables the use of feedback regularization in the solver, and also sets the weight on it. This task minimizes the differences between solutions of the
    * amount of CMP feedback to stabilize the ICP dynamics.
    *
    * @param regularizationWeight weight placed on changes in the CMP feedback solution.
    */
   public void setFeedbackRegularizationWeight(double regularizationWeight)
   {
      CommonOps.setIdentity(feedbackRegularizationWeight);
      CommonOps.scale(regularizationWeight, feedbackRegularizationWeight);

      hasFeedbackRegularizationTerm = true;
   }

   /**
    * Resets the previous feedback solution to zero.
    */
   public void resetFeedbackRegularization()
   {
      previousFeedbackDeltaSolution.zero();
   }

   /**
    * If using the active set solver, resets the active constraints. This only has an impact if using a warm start. Should be called everytime
    * there is a contact change, as this is when the number of constraints changes.
    */
   public void resetOnContactChange()
   {
      /*
      if (!useQuadProg)
         activeSetSolver.resetActiveConstraints();
         */
   }

   /**
    * Solves a linearly constrained quadratic program that computes the desired CMP feedback action combined with the desired step adjustment to stabilize the
    * ICP dynamics. This problem attempts to minimize the magnitude of CMP feedback while minimizing the amount of step adjustment. This is achieved by noting
    * that the current desired ICP location is a linear transformation of the upcoming step locations and the final desired ICP location.
    *
    * All the tasks must be set every tick before calling this method.
    *
    * @param finalICPRecursion recursion of the final desired ICP location.
    * @param cmpConstantEffect combined projection of the CMP offsets in the upcoming footsteps and teh stance CMP locations.
    * @param currentICP current location of the ICP
    * @param perfectCMP current desired value of the CMP based on the nominal ICP location.
    * @throws NoConvergenceException whether or not a solution was found. If it is thrown, the previous valid problem solution is used.
    */
   public void compute(FramePoint2d finalICPRecursion, FramePoint2d cmpConstantEffect, FramePoint2d currentICP, FramePoint2d perfectCMP) throws NoConvergenceException
   {
      indexHandler.computeProblemSize();

      reset();
      reshape();

      finalICPRecursion.changeFrame(worldFrame);
      cmpConstantEffect.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);
      perfectCMP.changeFrame(worldFrame);

      this.finalICPRecursion.set(0, 0, finalICPRecursion.getX());
      this.finalICPRecursion.set(1, 0, finalICPRecursion.getY());

      this.currentICP.set(0, 0, currentICP.getX());
      this.currentICP.set(1, 0, currentICP.getY());

      this.perfectCMP.set(0, 0, perfectCMP.getX());
      this.perfectCMP.set(1, 0, perfectCMP.getY());

      this.cmpConstantEffect.set(0, 0, cmpConstantEffect.getX());
      this.cmpConstantEffect.set(1, 0, cmpConstantEffect.getY());

      addFeedbackTask();
      addDynamicConstraint();
      addDynamicRelaxationTask();

      if (copLocationConstraint.getInequalityConstraintSize() > 0)
         addCoPLocationConstraint();

      if (reachabilityConstraint.getInequalityConstraintSize() > 0)
         addReachabilityConstraint();

      if (indexHandler.useStepAdjustment())
         addStepAdjustmentTask();

      if (indexHandler.useAngularMomentum())
         addAngularMomentumMinimizationTask();

      NoConvergenceException noConvergenceException = null;
      try
      {
         solve(solution);
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
      }

      if (noConvergenceException == null)
      {
         if (indexHandler.useStepAdjustment())
         {
            extractFootstepSolutions(footstepLocationSolution);
            if (autoSetPreviousSolution)
               setPreviousFootstepSolution(footstepLocationSolution);
         }

         extractFeedbackDeltaSolution(feedbackDeltaSolution);
         if (autoSetPreviousSolution)
            setPreviousFeedbackDeltaSolution(feedbackDeltaSolution);

         extractDynamicRelaxationSolution(dynamicRelaxationSolution);
         extractAngularMomentumSolution(angularMomentumSolution);

         computeWholeCostToGo();
         if (computeCostToGo)
            computeCostToGo();
      }
      else
      {
         throw noConvergenceException;
      }
   }

   /**
    * Adds the minimization of step adjustment task to the quadratic program.<br>
    * Also adds the regularization of the footstep adjustment,  if enabled.
    */
   private void addStepAdjustmentTask()
   {
      for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
      {
         inputCalculator.computeFootstepTask(i, footstepTaskInput, footstepWeights.get(i), referenceFootstepLocations.get(i));

         if (hasFootstepRegularizationTerm)
            inputCalculator.computeFootstepRegularizationTask(i, footstepTaskInput, footstepRegularizationWeight, previousFootstepLocations.get(i));
      }

      inputCalculator.submitFootstepTask(footstepTaskInput, solverInput_H, solverInput_h, solverInputResidualCost);
   }

   /**
    * Adds the minimization of feedback task to the quadratic program's cost objectives.<br>
    * Also adds the regularization of the feedback term, if enabled.
    */
   private void addFeedbackTask()
   {
      inputCalculator.computeFeedbackTask(feedbackTaskInput, feedbackWeight);

      if (hasFeedbackRegularizationTerm)
         inputCalculator.computeFeedbackRegularizationTask(feedbackTaskInput, feedbackRegularizationWeight, previousFeedbackDeltaSolution);

      inputCalculator.submitFeedbackTask(feedbackTaskInput, solverInput_H, solverInput_h, solverInputResidualCost);
   }

   /**
    * Adds the minimization of the dynamic relaxation to the quadratic program's cost objectives.
    */
   private void addDynamicRelaxationTask()
   {
      inputCalculator.computeDynamicRelaxationTask(dynamicRelaxationTask, dynamicRelaxationWeight);
      inputCalculator.submitDynamicRelaxationTask(dynamicRelaxationTask, solverInput_H, solverInput_h, solverInputResidualCost);
   }

   /**
    * Adds the minimization of angular momentum to the quadratic program's cost objectives.
    */
   private void addAngularMomentumMinimizationTask()
   {
      inputCalculator.computeAngularMomentumMinimizationTask(angularMomentumMinimizationTask, angularMomentumMinimizationWeight);
      inputCalculator.submitAngularMomentumMinimizationTask(angularMomentumMinimizationTask, solverInput_H, solverInput_h, solverInputResidualCost);
   }

   /**
    * Adds the convex CoP location constraint that requires the CoP to be in the support polygon.
    *
    * <p>
    * Takes the form Ax <= b.
    * </p>
    */
   private void addCoPLocationConstraint()
   {
      copLocationConstraint.setPositionOffset(perfectCMP);
      copLocationConstraint.setDeltaInside(deltaInside);
      copLocationConstraint.formulateConstraint();

      int constraintSize = copLocationConstraint.getInequalityConstraintSize();
      MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getFeedbackCMPIndex(), copLocationConstraint.Aineq, 0, 0, constraintSize, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, copLocationConstraint.bineq, 0, 0, constraintSize, 1, 1.0);

      if (indexHandler.useAngularMomentum())
         MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getAngularMomentumIndex(), copLocationConstraint.Aineq, 0, 0, constraintSize, 2, -1.0);

      currentInequalityConstraintIndex += constraintSize;
   }

   /**
    * Adds a convex location constraint on the footstep location that requires the footstep to be in the linear reachable region.
    *
    * <p>
    * Takes the form Ax <= b.
    * </p>
    */
   private void addReachabilityConstraint()
   {
      reachabilityConstraint.setDeltaInside(deltaInside);
      reachabilityConstraint.formulateConstraint();

      int constraintSize = reachabilityConstraint.getInequalityConstraintSize();
      MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getFootstepStartIndex(), reachabilityConstraint.Aineq, 0, 0, constraintSize, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, reachabilityConstraint.bineq, 0, 0, constraintSize, 1, 1.0);

      currentInequalityConstraintIndex += constraintSize;
   }

   /**
    * Adds the recursive dynamics as an equality constraint to the optimization. Takes the form
    *
    * <p>
    *    &delta; = k<sub>p</sub> ( x<sub>icp</sub>; - x<sub>icp,r</sub> ),
    * </p>
    * where
    * <p>
    *    x<sub>icp,r</sub> = &Phi;<sub>f</sub> + &Phi;<sub>const</sub> + &gamma;<sub>f,i</sub> r<sub>f,i</sub>,
    *
    * </p>
    * <p>
    * where
    *    <li>&Phi;<sub>f</sub> is the final ICP Recursion</li>
    *    <li>&Phi;<sub>const</sub> encodes the effects of the recursive CMP offsets in the upcoming footsteps and the stance CMP locations</li>
    *    <li>&gamma;<sub>f,i</sub> is the recursion multiplier of the i<sup>th</sup> footstep</li>
    *    <li>r<sub>f,i</sub> is the location of the i<sup>th</sup> footstep</li>
    * </p>
    */
   private void addDynamicConstraint()
   {
      inputCalculator.computeDynamicsConstraint(dynamicsConstraintInput, currentICP, finalICPRecursion, cmpConstantEffect, feedbackGain,
            footstepRecursionMultipliers);

      MatrixTools.setMatrixBlock(solverInput_Aeq, currentEqualityConstraintIndex, 0, dynamicsConstraintInput.Aeq, 0, 0, 2, indexHandler.getNumberOfFreeVariables(), 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, dynamicsConstraintInput.beq, 0, 0, 2, 1, 1.0);

      currentEqualityConstraintIndex += 2;
   }

   /**
    * Internal call to solves the quadratic program. Adds all the objectives and constraints to the problem and then solves it.
    *
    * @param solutionToPack solution of the QP.
    * @throws NoConvergenceException whether or not a solution was found. If it is thrown, the previous valid problem solution is used.
    */
   private void solve(DenseMatrix64F solutionToPack) throws NoConvergenceException
   {
      CommonOps.scale(-1.0, solverInput_h);

      for (int i = 0; i < solverInput_H.numCols; i++)
      {
         if (solverInput_H.get(i, i) < 0.0)
            throw new RuntimeException("Hey this is bad.");
      }

      solver.clear();
      solver.setQuadraticCostFunction(solverInput_H, solverInput_h, solverInputResidualCost.get(0, 0));
      solver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);
      solver.setLinearInequalityConstraints(solverInput_Aineq, solverInput_bineq);

      numberOfIterations = solver.solve(solutionToPack);


      if (MatrixTools.containsNaN(solutionToPack))
         throw new NoConvergenceException(numberOfIterations);
   }

   /**
    * Extracts the footstep locations from the solution vector.
    *
    * @param footstepLocationSolutionToPack 2d footstep location. Modified.
    */
   private void extractFootstepSolutions(DenseMatrix64F footstepLocationSolutionToPack)
   {
      MatrixTools.setMatrixBlock(footstepLocationSolutionToPack, 0, 0, solution, 0, 0, indexHandler.getNumberOfFootstepVariables(), 1, 1.0);
   }

   /**
    * Extracts the amount of CMP feedback from the solution vector
    *
    * @param feedbackSolutionToPack 2d feedback solution. Modified.
    */
   private void extractFeedbackDeltaSolution(DenseMatrix64F feedbackSolutionToPack)
   {
      MatrixTools.setMatrixBlock(feedbackSolutionToPack, 0, 0, solution, indexHandler.getFeedbackCMPIndex(), 0, 2, 1, 1.0);
   }

   /**
    * Extracts the dynamic relaxation magnitude from the solution vector.
    *
    * @param dynamicRelaxationSolutionToPack dynamic relaxation solution. Modified.
    */
   private void extractDynamicRelaxationSolution(DenseMatrix64F dynamicRelaxationSolutionToPack)
   {
      MatrixTools.setMatrixBlock(dynamicRelaxationSolutionToPack, 0, 0, solution, indexHandler.getDynamicRelaxationIndex(), 0, 2, 1, 1.0);
   }

   /**
    * Extracts the difference between the CMP and CoP from the solution vector.
    *
    * @param angularMomentumSolutionToPack difference between the CMP and CoP. Modified.
    */
   private void extractAngularMomentumSolution(DenseMatrix64F angularMomentumSolutionToPack)
   {
      if (indexHandler.useAngularMomentum())
         MatrixTools.setMatrixBlock(angularMomentumSolutionToPack, 0, 0, solution, indexHandler.getAngularMomentumIndex(), 0, 2, 1, 1.0);
   }

   /**
    * Sets the location of the previous footstep location for the footstep regularization task.
    *
    * @param footstepLocationSolution location of the footstep solution.
    */
   private void setPreviousFootstepSolution(DenseMatrix64F footstepLocationSolution)
   {
      for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
         MatrixTools.setMatrixBlock(previousFootstepLocations.get(i), 0, 0, footstepLocationSolution, 2 * i, 0, 2, 1, 1.0);
   }

   public void setPreviousFootstepSolutionFromCurrent()
   {
      if (indexHandler.useStepAdjustment())
      {
         for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
            MatrixTools.setMatrixBlock(previousFootstepLocations.get(i), 0, 0, footstepLocationSolution, 2 * i, 0, 2, 1, 1.0);
      }
   }

   /**
    * Sets the location of the previous CMP feedback for the feedback regularization task.
    *
    * @param feedbackDeltaSolution amount of CMP feedback.
    */
   private void setPreviousFeedbackDeltaSolution(DenseMatrix64F feedbackDeltaSolution)
   {
      previousFeedbackDeltaSolution.set(feedbackDeltaSolution);
   }

   public void setPreviousFeedbackDeltaSolutionFromCurrent()
   {
      previousFeedbackDeltaSolution.set(feedbackDeltaSolution);
   }

   /**
    * Internal method to compute the cost to go of all the tasks.
    */
   private void computeWholeCostToGo()
   {
      tmpCost.zero();
      tmpCost.reshape(indexHandler.getNumberOfFreeVariables(), 1);

      costToGo.zero();

      CommonOps.mult(solverInput_H, solution, tmpCost);
      CommonOps.multTransA(solution, tmpCost, costToGo);
      CommonOps.scale(0.5, costToGo);

      CommonOps.multAddTransA(solverInput_h, solution, costToGo); // already scaled by -1.0
      CommonOps.addEquals(costToGo, solverInputResidualCost);
   }

   /**
    * Internal method to compute the cost to go of all the tasks.
    */
   private void computeCostToGo()
   {
      tmpFootstepCost.zero();
      tmpFeedbackCost.zero();

      tmpFootstepCost.reshape(indexHandler.getNumberOfFootstepVariables(), 1);
      tmpFeedbackCost.reshape(2, 1);

      footstepCostToGo.zero();
      feedbackCostToGo.zero();
      dynamicRelaxationCostToGo.zero();
      angularMomentumMinimizationCostToGo.zero();

      // feedback cost:
      CommonOps.mult(feedbackTaskInput.quadraticTerm, feedbackDeltaSolution, tmpFeedbackCost);
      CommonOps.multTransA(feedbackDeltaSolution, tmpFeedbackCost, feedbackCostToGo);
      CommonOps.scale(0.5, feedbackCostToGo);

      CommonOps.multAddTransA(-1.0, feedbackTaskInput.linearTerm, feedbackDeltaSolution, feedbackCostToGo);
      CommonOps.addEquals(feedbackCostToGo, feedbackTaskInput.residualCost);

      // dynamic relaxation cost:
      CommonOps.mult(dynamicRelaxationTask.quadraticTerm, dynamicRelaxationSolution, tmpFeedbackCost);
      CommonOps.multTransA(dynamicRelaxationSolution, tmpFeedbackCost, dynamicRelaxationCostToGo);
      CommonOps.scale(0.5, dynamicRelaxationCostToGo);

      CommonOps.multAddTransA(-1.0, dynamicRelaxationTask.linearTerm, dynamicRelaxationSolution, dynamicRelaxationCostToGo);
      CommonOps.addEquals(dynamicRelaxationCostToGo, dynamicRelaxationTask.residualCost);

      if (indexHandler.useStepAdjustment())
      { // footstep cost:
         CommonOps.mult(footstepTaskInput.quadraticTerm, footstepLocationSolution, tmpFootstepCost);
         CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepCostToGo);
         CommonOps.scale(0.5, footstepCostToGo);

         CommonOps.multAddTransA(-1.0, footstepTaskInput.linearTerm, footstepLocationSolution, footstepCostToGo);
         CommonOps.addEquals(footstepCostToGo, footstepTaskInput.residualCost);
      }

      if (indexHandler.useAngularMomentum())
      { // angular momentum cost:
         CommonOps.mult(angularMomentumMinimizationTask.quadraticTerm, angularMomentumSolution, tmpFeedbackCost);
         CommonOps.multTransA(angularMomentumSolution, tmpFeedbackCost, angularMomentumMinimizationCostToGo);
         CommonOps.scale(0.5, angularMomentumMinimizationCostToGo);

         CommonOps.multAddTransA(-1.0, angularMomentumMinimizationTask.linearTerm, angularMomentumSolution, angularMomentumMinimizationCostToGo);
         CommonOps.addEquals(angularMomentumMinimizationCostToGo, angularMomentumMinimizationTask.residualCost);
      }
   }

   /**
    * Gets the footstep location solution for the step adjustment problem.
    *
    * @param footstepIndex index of footstep to get.
    * @param footstepLocationToPack location of the footstep in the world frame.
    */
   public void getFootstepSolutionLocation(int footstepIndex, FramePoint2d footstepLocationToPack)
   {
      footstepLocationToPack.setToZero(worldFrame);
      footstepLocationToPack.setX(footstepLocationSolution.get(2 * footstepIndex, 0));
      footstepLocationToPack.setY(footstepLocationSolution.get(2 * footstepIndex + 1, 0));
   }

   /**
    * Gets the CMP Feedback difference solution for the ICP Proportional feedback problem.
    *
    * @param cmpFeedbackDifferenceToPack difference between the nominal CMP and the desired CMP.
    */
   public void getCMPFeedbackDifference(FrameVector2d cmpFeedbackDifferenceToPack)
   {
      cmpFeedbackDifferenceToPack.setToZero(worldFrame);
      cmpFeedbackDifferenceToPack.setX(feedbackDeltaSolution.get(0, 0));
      cmpFeedbackDifferenceToPack.setY(feedbackDeltaSolution.get(1, 0));
   }

   /**
    * Gets the magnitude of the dynamic relaxation that is a slack variable in the recursive ICP dynamics.
    *
    * @param dynamicRelaxationToPack magnitude of the slack variable. Modified.
    */
   public void getDynamicRelaxation(FramePoint2d dynamicRelaxationToPack)
   {
      dynamicRelaxationToPack.setToZero(worldFrame);
      dynamicRelaxationToPack.setX(dynamicRelaxationSolution.get(0, 0));
      dynamicRelaxationToPack.setY(dynamicRelaxationSolution.get(1, 0));
   }

   /**
    * Gets the difference between the CMP and the CoP. This is equivalent to a scaled version of
    * the angular momentum of the system.
    *
    * @param differenceToPack difference between the two points. Modified.
    */
   public void getCMPDifferenceFromCoP(FramePoint2d differenceToPack)
   {
      differenceToPack.setToZero(worldFrame);
      differenceToPack.setX(angularMomentumSolution.get(0, 0));
      differenceToPack.setY(angularMomentumSolution.get(1, 0));
   }

   /**
    * Gets residual cost to go of the optimization problem. This value is included in the return of {@link #getCostToGo()}
    * return cost to go
    */
   public double getResidualCostToGo()
   {
      return solverInputResidualCost.get(0, 0);
   }

   /**
    * Gets the total cost to go of the optimization problem.
    * @return cost to go
    */
   public double getCostToGo()
   {
      return costToGo.get(0, 0);
   }

   /**
    * Gets the cost to go of the footstep adjustment task.
    * @return cost to go
    */
   public double getFootstepCostToGo()
   {
      return footstepCostToGo.get(0, 0);
   }

   /**
    * Gets the cost to go of the feedback minimization task.
    * @return cost to go
    */
   public double getFeedbackCostToGo()
   {
      return feedbackCostToGo.get(0, 0);
   }

   /**
    * Gets the cost to go of the dynamic relaxation minimization task.
    * @return cost to go
    */
   public double getDynamicRelaxationCostToGo()
   {
      return dynamicRelaxationCostToGo.get(0, 0);
   }

   /**
    * Gets the cost to go of the angular momentum minimization task.
    * @return cost to go
    */
   public double getAngularMomentumMinimizationCostToGo()
   {
      return angularMomentumMinimizationCostToGo.get(0, 0);
   }

   /**
    * Gets the number of iterations required to solve by the active set solver. Will return 1 if using the Quad Prog solver.
    * @return number of iterations
    */
   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }
}
