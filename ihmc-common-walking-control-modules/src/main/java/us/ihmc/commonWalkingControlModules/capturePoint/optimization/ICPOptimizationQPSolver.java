package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPIndexHandler;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPInputCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ConstraintToConvexRegion;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPInput;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.tools.exceptions.NoConvergenceException;

/**
 * Class that sets up the actual optimization framework and handles the inputs to generate an optimized solution
 * designed to stabilize ICP based walking trajectories using both CMP feedback and step adjustment. Designed to
 * work inside the {@link ICPOptimizationController}.
 */
public class ICPOptimizationQPSolver
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // FIXME this cannot be true until we setup resetting the active set based on state changes
   private static final boolean useWarmStart = false;
   private static final int maxNumberOfIterations = 10;
   private static final double convergenceThreshold = 1.0e-20;

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
   //private final DenseMatrix64F solverInput_Aeq;
   /** Total linear equality constraint objective vector for the quadratic program. */
   //private final DenseMatrix64F solverInput_beq;

   /**
    * Has the form A<sub>ineq</sub> x >= b<sub>ineq</sub>
    */
   /** Total linear inequality constraint matrix for the quadratic program. */
   private final DenseMatrix64F solverInput_Aineq;
   /** Total linear inequality constraint objective vector for the quadratic program. */
   private final DenseMatrix64F solverInput_bineq;


   /** QP Objective to minimize the amount of feedback action. Also contains feedback rate. */
   private final ICPQPInput feedbackTaskInput;
   /** QP Objective to minimize the amount of step adjustment. Also contains step adjustment. */
   private final ICPQPInput footstepTaskInput;
   /** QP Objective to minimize the angular momentum magnitude. */
   private final ICPQPInput angularMomentumMinimizationTask;
   /** QP Objective to minimize the difference between the dynamics */
   private final ICPQPInput dynamicsTaskInput;

   /** Constraint on the CoP location to the support polygon. */
   private final ConstraintToConvexRegion copLocationConstraint;
   /** Constraint on the CMP location to the bounds. */
   private final ConstraintToConvexRegion cmpLocationConstraint;
   /** Constraint on the footstep location to the reachable region. */
   private final ConstraintToConvexRegion reachabilityConstraint;
   /** Constraint on the footstep location to the planar region. */
   private final ConstraintToConvexRegion planarRegionConstraint;

   /** Recursion multipliers for the steps that encode the recursive ICP dynamics. */
   private double footstepRecursionMultiplier;
   private double footstepAdjustmentSafetyFactor;
   /** Location of the desired footsteps from a high level footstep planner. */
   private final DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);

   /** Current ICP Error location. */
   private final DenseMatrix64F perfectCMP = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);

   /** List of weights for tracking the different footsteps. */
   private final DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
   /** Weight for the footstep rate task. */
   private final DenseMatrix64F footstepRateWeight = new DenseMatrix64F(2, 2);
   /** Weight minimizing the CMP feedback action. */
   private final DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
   /** Weight regularizing the CMP feedback action. */
   private final DenseMatrix64F feedbackRateWeight = new DenseMatrix64F(2, 2);
   /** Weight minimizing the dynamic relaxation magnitude. */
   private final DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
   /** Weight minimizing the angular momentum magnitude. */
   private final DenseMatrix64F angularMomentumMinimizationWeight = new DenseMatrix64F(2, 2);
   /** Proportional gain on the ICP feedback controller. */
   private final DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);

   /** Flag to use the quad prog QP solver vs. the active set QP solver. **/
   private final SimpleEfficientActiveSetQPSolver solver = new SimpleEfficientActiveSetQPSolver();

   /** Full solution vector to the quadratic program. */
   private final DenseMatrix64F solution;
   /** Footstep location solution vector to the quadratic program. */
   private final DenseMatrix64F footstepLocationSolution;
   /** Feedback action solution to the quadratic program. */
   private final DenseMatrix64F feedbackDeltaSolution;
   /** Angular momentum solution to the quadratic program. */
   private final DenseMatrix64F angularMomentumSolution;

   /** Previous solution for the feedback action, used in the feedback rate objective. */
   private final DenseMatrix64F previousFeedbackDeltaSolution;
   private final DenseMatrix64F previousFootstepLocation = new DenseMatrix64F(2, 1);

   /** Cost to go for the entire quadratic program. */
   private final DenseMatrix64F costToGo;
   /** Cost to go for the step adjustment minimization objective. */
   private final DenseMatrix64F footstepCostToGo;
   /** Cost to go for the feedback action minimization objective. */
   private final DenseMatrix64F feedbackCostToGo;
   /** Cost to go for the angular momentum minimization objective. */
   private final DenseMatrix64F angularMomentumMinimizationCostToGo;

   /** Maximum number of vertices in the reachability polygon that the quadratic program will ever consider. Used for clearing and storing data. */
   private static final int maximumNumberOfReachabilityVertices = 4;

   /** Number of iterations required for the active set solver to find a solution. */
   private int numberOfIterations;

   private int currentInequalityConstraintIndex;

   /** boolean to determine whether or not to compute the cost to go. specified at compile time. */
   private final boolean computeCostToGo;
   private final boolean autoSetPreviousSolution;

   /** boolean indicating whether or not the footstep rate term has been added and can be used. */
   private boolean hasFootstepRateTerm = false;
   /** boolean indicating whether or not the feedback rate term has been added and can be used. */
   private boolean hasFeedbackRateTerm = false;

   /** Minimum allowable weight on the step adjustment task. */
   private final double minimumFootstepWeight;
   /** Minimum allowable weight on the feedback task. */
   private final double minimumFeedbackWeight;

   private final DenseMatrix64F tmpCost;
   private final DenseMatrix64F tmpFootstepCost;
   private final DenseMatrix64F tmpFeedbackCost;

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);

   private double copSafeDistanceToEdge = 0.0001;
   private double cmpSafeDistanceFromEdge = Double.POSITIVE_INFINITY;

   private boolean hasPlanarRegionConstraint = false;
   private double planarRegionDistanceFromEdge = 0.0;

   /**
    * Creates the ICP Optimization Solver. Refer to the class documentation: {@link ICPOptimizationQPSolver}.
    *
    * @param icpOptimizationParameters parameters to be used by in the optimization.
    * @param maximumNumberOfCMPVertices maximum number of vertices to be considered by the CoP location constraint.
    * @param computeCostToGo whether or not to compute the cost to go.
    */
   public ICPOptimizationQPSolver(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices, boolean computeCostToGo)
   {
      this(icpOptimizationParameters, maximumNumberOfCMPVertices, computeCostToGo, true);
   }


   public ICPOptimizationQPSolver(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices, boolean computeCostToGo,
                                  boolean autoSetPreviousSolution)
   {
      this(icpOptimizationParameters.getMinimumFootstepWeight(), icpOptimizationParameters.getMinimumFeedbackWeight(), maximumNumberOfCMPVertices,
           computeCostToGo, autoSetPreviousSolution);
   }

   public ICPOptimizationQPSolver(double minimumFootstepWeight, double minimumFeedbackWeight, int maximumNumberOfCMPVertices, boolean computeCostToGo)
   {
      this(minimumFootstepWeight, minimumFeedbackWeight, maximumNumberOfCMPVertices, computeCostToGo, true);
   }

   public ICPOptimizationQPSolver(double minimumFootstepWeight, double minimumFeedbackWeight, int maximumNumberOfCMPVertices, boolean computeCostToGo,
                                  boolean autoSetPreviousSolution)
   {
      this.computeCostToGo = computeCostToGo;
      this.autoSetPreviousSolution = autoSetPreviousSolution;

      indexHandler = new ICPQPIndexHandler();
      inputCalculator = new ICPQPInputCalculator(indexHandler);

      this.minimumFootstepWeight = minimumFootstepWeight;
      this.minimumFeedbackWeight = minimumFeedbackWeight;

      int maximumNumberOfFreeVariables = 6;
      int maximumNumberOfLagrangeMultipliers = 8;

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      solverInputResidualCost = new DenseMatrix64F(1, 1);

      feedbackTaskInput = new ICPQPInput(2);
      footstepTaskInput = new ICPQPInput(2);
      angularMomentumMinimizationTask = new ICPQPInput(2);
      dynamicsTaskInput = new ICPQPInput(6);

      copLocationConstraint = new ConstraintToConvexRegion(maximumNumberOfCMPVertices);
      cmpLocationConstraint = new ConstraintToConvexRegion(maximumNumberOfCMPVertices);
      reachabilityConstraint = new ConstraintToConvexRegion(maximumNumberOfReachabilityVertices);
      planarRegionConstraint = new ConstraintToConvexRegion(20);

      solverInput_Aineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices);
      solverInput_bineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, 1);

      solution = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      footstepLocationSolution = new DenseMatrix64F(2, 1);
      feedbackDeltaSolution = new DenseMatrix64F(2, 1);
      angularMomentumSolution = new DenseMatrix64F(2, 1);

      previousFeedbackDeltaSolution = new DenseMatrix64F(2, 1);

      tmpCost = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      tmpFootstepCost = new DenseMatrix64F(2, 1);
      tmpFeedbackCost = new DenseMatrix64F(2, 1);
      costToGo = new DenseMatrix64F(1, 1);
      footstepCostToGo = new DenseMatrix64F(1, 1);
      feedbackCostToGo = new DenseMatrix64F(1, 1);
      angularMomentumMinimizationCostToGo = new DenseMatrix64F(1, 1);

      solver.setConvergenceThreshold(convergenceThreshold);
      solver.setMaxNumberOfIterations(maxNumberOfIterations);
      solver.setUseWarmStart(useWarmStart);
   }

   /**
    * Sets whether or not the footstep adjustment should have knowledge of trying to use angular momentum for balance.
    * By default, this is true.
    */
   public void setConsiderAngularMomentumInAdjustment(boolean considerAngularMomentumInAdjustment)
   {
      inputCalculator.setConsiderAngularMomentumInAdjustment(considerAngularMomentumInAdjustment);
   }

   /**
    * Sets whether or not the footstep adjustment should have knowledge of trying to use cop feedback for balance.
    * By default, this is true.
    */
   public void setConsiderFeedbackInAdjustment(boolean considerFeedbackInAdjustment)
   {
      inputCalculator.setConsiderFeedbackInAdjustment(considerFeedbackInAdjustment);
   }

   /**
    * Sets the maximum number of iterations to be used by the active set solver.
    */
   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      solver.setMaxNumberOfIterations(maxNumberOfIterations);
   }

   /**
    * Resets the constraint on the CoP location. This constraint requires that the CoP not exit the convex hull of the support polygon.
    */
   public void resetCoPLocationConstraint()
   {
      copLocationConstraint.reset();
      cmpLocationConstraint.reset();
   }

   public void setCopSafeDistanceToEdge(double copSafeDistanceToEdge)
   {
      this.copSafeDistanceToEdge = copSafeDistanceToEdge;
   }

   public void setMaxCMPDistanceFromEdge(double maxCMPDistance)
   {
      this.cmpSafeDistanceFromEdge = maxCMPDistance;
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
      cmpLocationConstraint.addPolygon(polygon);
   }

   /**
    * Resets the reachability constraint on the upcoming footstep location. This constraint requires that the footstep lies in the convex hull of this polygon.
    */
   public void resetReachabilityConstraint()
   {
      reachabilityConstraint.reset();
   }

   public void resetPlanarRegionConstraint()
   {
      planarRegionConstraint.reset();
      hasPlanarRegionConstraint = false;
   }

   public void addReachabilityPolygon(FrameConvexPolygon2d polygon)
   {
      polygon.changeFrame(worldFrame);
      polygon.update();
      reachabilityConstraint.addPolygon(polygon);
   }

   public void setPlanarRegionConstraint(ConvexPolygon2D convexPolygon, double planarRegionDistanceFromEdge)
   {
      hasPlanarRegionConstraint = planarRegionConstraint.addPlanarRegion(convexPolygon, planarRegionDistanceFromEdge);
   }

   public void setPlanarRegionConstraint(ConvexPolygon2D convexPolygon)
   {
      hasPlanarRegionConstraint = planarRegionConstraint.addPlanarRegion(convexPolygon);
   }

   /**
    * Zeros all the pertinent scalars, vectors, and matrices for the solver. Should be called at the beginning of every computation tick.
    */
   private void reset()
   {
      solverInput_H.zero();
      solverInput_h.zero();
      solverInputResidualCost.zero();

      solverInput_Aineq.zero();
      solverInput_bineq.zero();

      angularMomentumMinimizationTask.reset();
      footstepTaskInput.reset();
      feedbackTaskInput.reset();
      dynamicsTaskInput.reset();

      solution.zero();
      footstepLocationSolution.zero();
      feedbackDeltaSolution.zero();
      angularMomentumSolution.zero();

      currentInequalityConstraintIndex = 0;
   }

   /**
    * Reshapes all the vectors and matrices to the appropriate size, based on the number of footsteps to handle. Should be called after {@link #reset()}
    * at the beginning of every computation tick.
    */
   private void reshape()
   {
      int problemSize = indexHandler.getNumberOfFreeVariables();
      int numberOfFootstepsToConsider = indexHandler.getNumberOfFootstepsToConsider();
      int numberOfInequalityConstraints;

      copLocationConstraint.setPolygon();
      cmpLocationConstraint.setPolygon();
      reachabilityConstraint.setPolygon();
      if (hasPlanarRegionConstraint)
         planarRegionConstraint.setPolygon();

      numberOfInequalityConstraints = copLocationConstraint.getInequalityConstraintSize();
      if (indexHandler.useStepAdjustment())
      {
         numberOfInequalityConstraints += reachabilityConstraint.getInequalityConstraintSize();
         if (hasPlanarRegionConstraint)
            numberOfInequalityConstraints += planarRegionConstraint.getInequalityConstraintSize();
      }
      if (indexHandler.useAngularMomentum() && Double.isFinite(cmpSafeDistanceFromEdge))
         numberOfInequalityConstraints += cmpLocationConstraint.getInequalityConstraintSize();

      solverInput_H.reshape(problemSize, problemSize);
      solverInput_h.reshape(problemSize, 1);

      feedbackTaskInput.reshape(2);
      dynamicsTaskInput.reshape(problemSize);
      angularMomentumMinimizationTask.reshape(2);
      footstepTaskInput.reshape(2 * numberOfFootstepsToConsider);

      solverInput_Aineq.reshape(numberOfInequalityConstraints, problemSize);
      solverInput_bineq.reshape(numberOfInequalityConstraints, 1);

      solution.reshape(problemSize, 1);
      footstepLocationSolution.reshape(2 * numberOfFootstepsToConsider, 1);
   }

   /**
    * Resets the controller conditions on the feedback minimization task, the feedback gains, and the dynamic relaxation minimization task.
    * Also sets that the controller is not to attempt to regularize the feedback minimization task.
    *
    */
   public void resetFeedbackConditions()
   {
      feedbackWeight.zero();
      feedbackGain.zero();
      dynamicsWeight.zero();

      hasFeedbackRateTerm = false;
   }

   /**
    * Resets the controller conditions for the minimization of angular momentum task, and also sets it so that the controller will not attempt to utilize
    * angular momentum to stabilize the ICP dynamics.
    *
    */
   public void resetAngularMomentumConditions()
   {
      angularMomentumMinimizationWeight.zero();
      indexHandler.setUseAngularMomentum(false);
   }

   /**
    * Resets the footstep plan tracking conditions for the controller. This includes resetting all the reference footstep locations and the
    * footstep recursion multipliers. Also sets that the controller is not to attempt to regularize the footstep locations.
    */
   public void resetFootstepConditions()
   {
      indexHandler.resetFootsteps();

      footstepRateWeight.zero();

      referenceFootstepLocation.zero();
      footstepRecursionMultiplier = 0.0;
      footstepWeight.zero();

      hasFootstepRateTerm = false;
   }

   /**
    * Sets the conditions for the footstep adjustment task. This includes the weight of tracking the specified footstep by the optimization algorithm,
    * the reference location of the footstep, and the recursion multiplier of that footstep for the ICP dynamics.
    *
    * @param recursionMultiplier recursion multiplier for the footstep for the ICP dynamics.
    * @param weight weight on tracking the reference footstep location in the solver.
    */
   public void setFootstepAdjustmentConditions(double recursionMultiplier, double weight, FramePoint2D referenceFootstepLocation)
   {
      this.setFootstepAdjustmentConditions(recursionMultiplier, weight, 1.0, referenceFootstepLocation);
   }

   /**
    * Sets the conditions for the footstep adjustment task. This includes the weight of tracking the specified footstep by the optimization algorithm,
    * the reference location of the footstep, and the recursion multiplier of that footstep for the ICP dynamics.
    *
    * @param recursionMultiplier recursion multiplier for the footstep for the ICP dynamics.
    * @param weight weight on tracking the reference footstep location in the solver.
    */
   public void setFootstepAdjustmentConditions(double recursionMultiplier, double weight, double safetyFactor, FramePoint2D referenceFootstepLocation)
   {
      this.setFootstepAdjustmentConditions(recursionMultiplier, weight, weight, safetyFactor, referenceFootstepLocation);
   }

   /**
    * Sets the conditions for the footstep adjustment task. This includes the weight of tracking the specified footstep by the optimization algorithm,
    * the reference location of the footstep, and the recursion multiplier of that footstep for the ICP dynamics.
    *
    * @param recursionMultiplier recursion multiplier for the footstep for the ICP dynamics.
    * @param xWeight weight on tracking the reference footstep location in the solver in the Cartesian x coordinate.
    * @param yWeight weight on tracking the reference footstep location in the solver in the Cartesian y coordinate.
    * @param referenceFootstepLocation location of the desired reference footstep.
    */
   public void setFootstepAdjustmentConditions(double recursionMultiplier, double xWeight, double yWeight, double safetyFactor, FramePoint2D referenceFootstepLocation)
   {
      footstepRecursionMultiplier = recursionMultiplier;
      footstepAdjustmentSafetyFactor = safetyFactor;

      xWeight = Math.max(minimumFootstepWeight, xWeight);
      yWeight = Math.max(minimumFootstepWeight, yWeight);

      footstepWeight.zero();
      footstepWeight.set(0, 0, xWeight);
      footstepWeight.set(1, 1, yWeight);

      referenceFootstepLocation.changeFrame(worldFrame);
      this.referenceFootstepLocation.set(0, 0, referenceFootstepLocation.getX());
      this.referenceFootstepLocation.set(1, 0, referenceFootstepLocation.getY());

      indexHandler.registerFootstep();
   }

   /**
    * Sets the conditions for the minimization of the angular momentum task. This includes whether or not to utilize angular momentum to help stabilize
    * the ICP dynamics, as well as the weight on its minimization.
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
    * Enables the use of footstep rate in the solver, and also sets the weight on it. This task minimizes the differences between solutions of the
    * footstep location.
    *
    * @param rateWeight weight placed on changes in the footstep location solution.
    */
   public void setFootstepRateWeight(double rateWeight)
   {
      CommonOps.setIdentity(footstepRateWeight);
      CommonOps.scale(rateWeight, footstepRateWeight);

      hasFootstepRateTerm = true;
   }

   /**
    * Resets the footstep rate objectives. This is important to call at the start of every new step, if using footstep rate.
    *
    * @param previousFootstepLocation new location of the previous footstep location to try and minimize against.
    */
   public void resetFootstepRate(FramePoint2D previousFootstepLocation)
   {
      previousFootstepLocation.changeFrame(worldFrame);
      this.previousFootstepLocation.set(0, 0, previousFootstepLocation.getX());
      this.previousFootstepLocation.set(1, 0, previousFootstepLocation.getY());
   }

   /**
    * Resets the feedback rate objective.
    *
    * @param previousFeedbackDeltaSolution new location of the previous feedback location to try and minimize against.
    */
   public void resetFeedbackRate(FramePoint2D previousFeedbackDeltaSolution)
   {
      previousFeedbackDeltaSolution.changeFrame(worldFrame);
      this.previousFeedbackDeltaSolution.set(0, 0, previousFeedbackDeltaSolution.getX());
      this.previousFeedbackDeltaSolution.set(1, 0, previousFeedbackDeltaSolution.getY());
   }




   /**
    * Sets the conditions for the feedback minimization task and the dynamic relaxation minimization task. This task minimizes the difference between
    * the nominal CMP location and the one used to control the ICP dynamics. The dynamic relaxation allows the ICP recursive dynamics to be violated by a
    * small magnitude, which is critical to not overconstraining the problem.
    *
    * @param feedbackWeight weight on the minimization of the feedback action for the solver.
    * @param feedbackGain ICP controller proportional gain.
    * @param dynamicsWeight weight on the minimization of the dynamic relaxation for the solver.
    */
   public void setFeedbackConditions(double feedbackWeight, double feedbackGain, double dynamicsWeight)
   {
      this.setFeedbackConditions(feedbackWeight, feedbackWeight, feedbackGain, feedbackGain, dynamicsWeight);
   }

   /**
    * Sets the conditions for the feedback minimization task and the dynamic relaxation minimization task. This task minimizes the difference between
    * the nominal CMP location and the one used to control the ICP dynamics. The dynamic relaxation allows the ICP recursive dynamics to be violated by a
    * small magnitude, which is critical to not overconstraining the problem.
    *
    * @param feedbackXWeight weight on the minimization of the feedback action for the solver in the Cartesian x coordinate direction.
    * @param feedbackYWeight weight on the minimization of the feedback action for the solver in the Cartesian y coordinate direction.
    * @param feedbackXGain ICP controller proportional gain in the Cartesian x coordinate direction.
    * @param feedbackYGain ICP controller proportional gain in the Cartesian y coordinate direction.
    * @param dynamicsWeight weight on the minimization of the dynamic relaxation for the solver.
    */
   public void setFeedbackConditions(double feedbackXWeight, double feedbackYWeight, double feedbackXGain, double feedbackYGain, double dynamicsWeight)
   {
      feedbackXWeight = Math.max(feedbackXWeight, minimumFeedbackWeight);
      feedbackYWeight = Math.max(feedbackYWeight, minimumFeedbackWeight);

      this.feedbackWeight.zero();
      this.feedbackWeight.set(0, 0, feedbackXWeight);
      this.feedbackWeight.set(1, 1, feedbackYWeight);

      this.feedbackGain.zero();
      this.feedbackGain.set(0, 0, feedbackXGain);
      this.feedbackGain.set(1, 1, feedbackYGain);

      CommonOps.setIdentity(this.dynamicsWeight);
      CommonOps.scale(dynamicsWeight, this.dynamicsWeight);
   }

   /**
    * Enables the use of feedback rate in the solver, and also sets the weight on it. This task minimizes the differences between solutions of the
    * amount of CMP feedback to stabilize the ICP dynamics.
    *
    * @param rateWeight weight placed on changes in the CMP feedback solution.
    */
   public void setFeedbackRateWeight(double rateWeight)
   {
      CommonOps.setIdentity(feedbackRateWeight);
      CommonOps.scale(rateWeight, feedbackRateWeight);

      hasFeedbackRateTerm = true;
   }

   /**
    * Solves a linearly constrained quadratic program that computes the desired CMP feedback action combined with the desired step adjustment to stabilize the
    * ICP dynamics. This problem attempts to minimize the magnitude of CMP feedback while minimizing the amount of step adjustment. This is achieved by noting
    * that the current desired ICP location is a linear transformation of the upcoming step locations and the final desired ICP location.
    *
    * All the tasks must be set every tick before calling this method.
    *
    * @param perfectCMP current desired value of the CMP based on the nominal ICP location.
    * @throws NoConvergenceException whether or not a solution was found. If it is thrown, the previous valid problem solution is used.
    */
   public void compute(FrameVector2DReadOnly currentICPError, FramePoint2D perfectCMP) throws NoConvergenceException
   {
      indexHandler.computeProblemSize();

      reset();
      reshape();

      currentICPError.checkReferenceFrameMatch(worldFrame);
      perfectCMP.changeFrame(worldFrame);

      this.currentICPError.set(0, 0, currentICPError.getX());
      this.currentICPError.set(1, 0, currentICPError.getY());
      this.perfectCMP.set(0, 0, perfectCMP.getX());
      this.perfectCMP.set(1, 0, perfectCMP.getY());

      addFeedbackTask();

      if (indexHandler.useStepAdjustment())
         addStepAdjustmentTask();

      if (indexHandler.useAngularMomentum())
         addAngularMomentumMinimizationTask();

      addDynamicConstraintTask();

      if (copLocationConstraint.getInequalityConstraintSize() > 0)
         addCoPLocationConstraint();

      if (Double.isFinite(cmpSafeDistanceFromEdge) && indexHandler.useAngularMomentum() && cmpLocationConstraint.getInequalityConstraintSize() > 0)
         addCMPLocationConstraint();

      if (indexHandler.useStepAdjustment())
      {
         if (reachabilityConstraint.getInequalityConstraintSize() > 0)
            addReachabilityConstraint();
         if (planarRegionConstraint.getInequalityConstraintSize() > 0)
            addPlanarRegionConstraint();
      }

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
    * Also adds the rate of the footstep adjustment,  if enabled.
    */
   private void addStepAdjustmentTask()
   {
      int stepIndex = 0;
      inputCalculator.computeFootstepTask(stepIndex, footstepTaskInput, footstepWeight, referenceFootstepLocation);

      if (hasFootstepRateTerm)
         inputCalculator.computeFootstepRateTask(stepIndex, footstepTaskInput, footstepRateWeight, previousFootstepLocation);

      inputCalculator.submitFootstepTask(footstepTaskInput, solverInput_H, solverInput_h, solverInputResidualCost);
   }

   /**
    * Adds the minimization of feedback task to the quadratic program's cost objectives.<br>
    * Also adds the rate of the feedback term, if enabled.
    */
   private void addFeedbackTask()
   {
      ICPQPInputCalculator.computeFeedbackTask(feedbackTaskInput, feedbackWeight);

      if (hasFeedbackRateTerm)
         inputCalculator.computeFeedbackRateTask(feedbackTaskInput, feedbackRateWeight, previousFeedbackDeltaSolution);

      inputCalculator.submitFeedbackTask(feedbackTaskInput, solverInput_H, solverInput_h, solverInputResidualCost);
   }

   /**
    * Adds the minimization of angular momentum to the quadratic program's cost objectives.
    */
   private void addAngularMomentumMinimizationTask()
   {
      ICPQPInputCalculator.computeAngularMomentumMinimizationTask(angularMomentumMinimizationTask, angularMomentumMinimizationWeight);
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
      copLocationConstraint.setDeltaInside(copSafeDistanceToEdge);
      copLocationConstraint.formulateConstraint();

      int constraintSize = copLocationConstraint.getInequalityConstraintSize();
      MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getFeedbackCMPIndex(), copLocationConstraint.Aineq, 0, 0, constraintSize, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, copLocationConstraint.bineq, 0, 0, constraintSize, 1, 1.0);

      currentInequalityConstraintIndex += constraintSize;
   }

   /**
    * Adds the convex CMP location constraint that requires the CMP to be within some distance of in the support polygon.
    *
    * <p>
    * Takes the form Ax <= b.
    * </p>
    */
   private void addCMPLocationConstraint()
   {
      cmpLocationConstraint.setPositionOffset(perfectCMP);
      cmpLocationConstraint.setDeltaInside(-cmpSafeDistanceFromEdge);
      cmpLocationConstraint.formulateConstraint();

      int constraintSize = cmpLocationConstraint.getInequalityConstraintSize();
      MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getFeedbackCMPIndex(), cmpLocationConstraint.Aineq, 0, 0, constraintSize, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getAngularMomentumIndex(), cmpLocationConstraint.Aineq, 0, 0, constraintSize, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, cmpLocationConstraint.bineq, 0, 0, constraintSize, 1, 1.0);

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
      reachabilityConstraint.setDeltaInside(0.0);
      reachabilityConstraint.formulateConstraint();

      int constraintSize = reachabilityConstraint.getInequalityConstraintSize();
      MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getFootstepStartIndex(), reachabilityConstraint.Aineq, 0, 0, constraintSize, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, reachabilityConstraint.bineq, 0, 0, constraintSize, 1, 1.0);

      currentInequalityConstraintIndex += constraintSize;
   }

   /**
    * Adds a convex location constraint on the footstep location that requires the footstep to be in a planar region.
    *
    * <p>
    * Takes the form Ax <= b.
    * </p>
    */
   private void addPlanarRegionConstraint()
   {
      // todo formulate the distance inside
      planarRegionConstraint.formulateConstraint();

      int constraintSize = planarRegionConstraint.getInequalityConstraintSize();
      MatrixTools.setMatrixBlock(solverInput_Aineq, currentInequalityConstraintIndex, indexHandler.getFootstepStartIndex(), planarRegionConstraint.Aineq, 0, 0, constraintSize, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, planarRegionConstraint.bineq, 0, 0, constraintSize, 1, 1.0);

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
   private void addDynamicConstraintTask()
   {
      inputCalculator.computeDynamicsTask(dynamicsTaskInput, currentICPError, referenceFootstepLocation, feedbackGain,
            dynamicsWeight, footstepRecursionMultiplier, footstepAdjustmentSafetyFactor);
      inputCalculator.submitDynamicsTask(dynamicsTaskInput, solverInput_H, solverInput_h, solverInputResidualCost);
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
      MatrixTools.setMatrixBlock(footstepLocationSolutionToPack, 0, 0, solution, indexHandler.getFootstepStartIndex(), 0, indexHandler.getNumberOfFootstepVariables(), 1, 1.0);
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
    * Sets the location of the previous footstep location for the footstep rate task.
    *
    * @param footstepLocationSolution location of the footstep solution.
    */
   private void setPreviousFootstepSolution(DenseMatrix64F footstepLocationSolution)
   {
      int stepIndex = 0;
      MatrixTools.setMatrixBlock(previousFootstepLocation, 0, 0, footstepLocationSolution, 2 * stepIndex, 0, 2, 1, 1.0);
   }

   /**
    * Sets the location of the previous CMP feedback for the feedback rate task.
    *
    * @param feedbackDeltaSolution amount of CMP feedback.
    */
   private void setPreviousFeedbackDeltaSolution(DenseMatrix64F feedbackDeltaSolution)
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
      angularMomentumMinimizationCostToGo.zero();

      // feedback cost:
      CommonOps.mult(feedbackTaskInput.quadraticTerm, feedbackDeltaSolution, tmpFeedbackCost);
      CommonOps.multTransA(feedbackDeltaSolution, tmpFeedbackCost, feedbackCostToGo);
      CommonOps.scale(0.5, feedbackCostToGo);

      CommonOps.multAddTransA(-1.0, feedbackTaskInput.linearTerm, feedbackDeltaSolution, feedbackCostToGo);
      CommonOps.addEquals(feedbackCostToGo, feedbackTaskInput.residualCost);

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
   public void getFootstepSolutionLocation(int footstepIndex, FramePoint2D footstepLocationToPack)
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
   public void getCoPFeedbackDifference(FrameVector2D cmpFeedbackDifferenceToPack)
   {
      cmpFeedbackDifferenceToPack.setToZero(worldFrame);
      cmpFeedbackDifferenceToPack.setX(feedbackDeltaSolution.get(0, 0));
      cmpFeedbackDifferenceToPack.setY(feedbackDeltaSolution.get(1, 0));
   }

   /**
    * Gets the difference between the CMP and the CoP. This is equivalent to a scaled version of
    * the angular momentum of the system.
    *
    * @param differenceToPack difference between the two points. Modified.
    */
   public void getCMPDifferenceFromCoP(FrameVector2D differenceToPack)
   {
      differenceToPack.setToZero(worldFrame);
      differenceToPack.setX(angularMomentumSolution.get(0, 0));
      differenceToPack.setY(angularMomentumSolution.get(1, 0));
   }

   /**
    * Gets the total cost to go of the optimization problem.
    * @return cost to go
    */
   public double getCostToGo()
   {
      return costToGo.get(0);
   }

   /**
    * Gets the cost to go of the footstep adjustment task.
    * @return cost to go
    */
   public double getFootstepCostToGo()
   {
      return footstepCostToGo.get(0);
   }

   /**
    * Gets the cost to go of the feedback minimization task.
    * @return cost to go
    */
   public double getFeedbackCostToGo()
   {
      return feedbackCostToGo.get(0);
   }

   /**
    * Gets the cost to go of the angular momentum minimization task.
    * @return cost to go
    */
   public double getAngularMomentumMinimizationCostToGo()
   {
      return angularMomentumMinimizationCostToGo.get(0);
   }

   /**
    * Gets the number of iterations required to solve by the active set solver. Will return 1 if using the Quad Prog solver.
    * @return number of iterations
    */
   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public ConstraintToConvexRegion getCoPLocationConstraint()
   {
      return copLocationConstraint;
   }

   public ConstraintToConvexRegion getCMPLocationConstraint()
   {
      return cmpLocationConstraint;
   }
}
