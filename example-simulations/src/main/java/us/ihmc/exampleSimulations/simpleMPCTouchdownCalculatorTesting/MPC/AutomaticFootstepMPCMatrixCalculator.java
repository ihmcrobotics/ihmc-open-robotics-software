package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.closedSourceControl.fastWalking.parameters.FastWalkingParameters;
import us.ihmc.closedSourceControl.fastWalking.parameters.YoFastWalkingConstantParametersForStep;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.BPWPlanarWalkingRobotEstimates;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This purpose of this class is to calculate the matrices
 * for automatic footstepping with MPC.
 * especially, A<sub>&delta</sub> T,
 * <p>
 * given <br>
 * - T<sub>s</sub> : swing duration<br>
 * - t : current time<br>
 * - w : sqrt(g/CoMz)<br>
 * - N<sub>s</sub> : number of steps look ahead , in this test, we look 2-step ahead.<br>
 * - m : total mass of the robot.<br>
 * - M : Number of sample<br>
 * - &delta T : N_s/M is sample period<br>
 * - A<sub>&delta T</sub> (discreteStateTransitionMatrix) : state transition matrix in discrete time states<br>
 */
public class AutomaticFootstepMPCMatrixCalculator
{
   private static final double rateRegularizationWeight = 0.00001;
   private static final double idealStepWeight = 2.0;

   public static final int STATE_SIZE = 4;
   public static final int CONTROL_SIZE = 2;
   private static final int NUMBER_OF_STEPS = 2;
   private static final int NUMBER_OF_VARIABLES = CONTROL_SIZE * NUMBER_OF_STEPS;
   private static final int SIZE_OF_FUTURE_STATES = STATE_SIZE * NUMBER_OF_STEPS;

   private final DMatrixRMaj currentTickStateVector;
   private final DMatrixRMaj lowerBoundOfDistanceLimit;
   private final DMatrixRMaj upperBoundOfDistanceLimit;
   private final DMatrixRMaj distanceLimitConstraintMatrix;
   private final DMatrixRMaj lowerBoundOfEllipseLimit;
   private final DMatrixRMaj upperBoundOfEllipseLimit;
   private final DMatrixRMaj ellipseLimitConstraintMatrix;

   //New things, clean later
   private final DoubleProvider mass;
   private final DoubleProvider gravity;
   private final ReferenceFrame controlFrame;

   //TODO should change below data type. this is temporally value
   //   private final FixedFramePoint2DBasics pendulumBaseInSoleFrame;
   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D angularMomentumAboutContactPoint = new FrameVector3D();
   private final double nominalStanceWidth;
   //TODO should be changed into Vecot3DReadOnly, angularmomentum is read as the 3DReadOnly
   private final Vector2DReadOnly desiredXYVelocity;
   private final double comZUpFrameX;
   private final double comZUpFrameY;
   private final double massComZUpXOmegaX;
   private final double massComZUpYOmegaY;

   private final double massComZUpX;
   private final double massComZUpY;
   private final double massGravity;

   private final double firstFootAngle;
   private final double secondFootAngle;
   //   private final double timeRemainingInSwing;

   private final DoubleProvider omegaX;
   private final DoubleProvider omegaY;
   private final DoubleProvider swingDuration;
   //   private final FastWalkingFeetManager feetManager;

   private final DMatrixRMaj optimizedSolution;
   private final DMatrixRMaj previousOptimizedSolution;

   private final YoVector2D angularMomentumInMPC;
   private final YoVector2D yoAngularMomentumInContactPoint;

   private final YoMatrix yoCurrentTickStateVector;

   private final FrameVector2D tempVector = new FrameVector2D();
   private final FrameVector2D comVelocity = new FrameVector2D();

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);
   private final YoMatrix mpcSolution;

   private final DMatrixRMaj predictedStateAtEndOfCurrentPhase = new DMatrixRMaj(STATE_SIZE, 1);
   private final YoMatrix yoPredictedStateAtEndOfCurrentPhase;

   private final DMatrixRMaj ARemaining = new DMatrixRMaj(STATE_SIZE, STATE_SIZE);
   private final DMatrixRMaj A = new DMatrixRMaj(STATE_SIZE, STATE_SIZE); // This is the state transition matrix for the full step duration
   private final DMatrixRMaj AJump = new DMatrixRMaj(STATE_SIZE,
                                                     STATE_SIZE); // This is the state transition matrix from one terminal step to the next (A bar in the slides)
   private final DMatrixRMaj BJump = new DMatrixRMaj(STATE_SIZE,
                                                     CONTROL_SIZE); // This is the control transition matrix from one terminal step to the next (B bar in the slides)
   private final DMatrixRMaj AHat = new DMatrixRMaj(STATE_SIZE, STATE_SIZE); // This is A * AJump
   private final DMatrixRMaj BHat = new DMatrixRMaj(STATE_SIZE, CONTROL_SIZE); // This is A * BJump
   private final DMatrixRMaj AHatAHat = new DMatrixRMaj(STATE_SIZE, STATE_SIZE);
   private final DMatrixRMaj AHatBHat = new DMatrixRMaj(STATE_SIZE, CONTROL_SIZE);

   private final DMatrixRMaj stackedStateTransitionMatrix = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, STATE_SIZE);
   private final DMatrixRMaj stackedControlInputMatrix = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, NUMBER_OF_VARIABLES);

   private final DMatrixRMaj desiredTwoStepAheadStatesVector;
   private final YoMatrix desiredTwoStepStates;
   private final DMatrixRMaj desiredStateVector = new DMatrixRMaj(STATE_SIZE, 1);
   private final DMatrixRMaj idealStepVector = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, 1);
   private final DMatrixRMaj costFunctionHessian = new DMatrixRMaj(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);
   private final DMatrixRMaj costFunctionGradient = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);
   private final DMatrixRMaj weightQMatrix;
   private final DMatrixRMaj rateRegularizationWeightMatrix = new DMatrixRMaj(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);
   private final DMatrixRMaj idealStepWeightMatrix = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, SIZE_OF_FUTURE_STATES);
   private final DMatrixRMaj optimalTwoStepStates = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, SIZE_OF_FUTURE_STATES);
   private final YoMatrix yoOptimalTwoStepStates;
   private final FramePoint2D capturePointInMPC = new FramePoint2D();
   private final YoVector2D supportFootPosition;
   private final FramePoint2D supportFootPoint = new FramePoint2D();

   private final YoVector2D pendulumBaseInMPC;
   private final YoFrameVector2D comPositionRelativeToContact;
   private final YoVector2D yoCapturePointInMPC;
   private final YoVector2D differenceOfDesiredMeasuredCoP;

   private final YoMatrix savedInitialStateVector;
   private final YoDouble angularMomentumOffsetLeftSupport;
   private final YoDouble angularMomentumOffsetRightSupport;

   private final YoMatrix aTransitionMatrix;

   //TODO Change this into final. for now, to escape the error, don't justify as final value.

   public AutomaticFootstepMPCMatrixCalculator(RobotSide supportSide,
                                               FrameVector3DReadOnly angularMomentum,
                                               BPWPlanarWalkingRobotEstimates estimates,
                                               Vector2DReadOnly desiredVelocityProvider,
                                               YoFastWalkingConstantParametersForStep fastWalkingParametersForStep,
                                               FastWalkingParameters fastWalkingParameters,
                                               double deltaT,
                                               YoRegistry parentRegistry)
   {
      this(supportSide,
           estimates::getTotalMass,
           deltaT,
           desiredVelocityProvider,
           fastWalkingParametersForStep.getSwingDuration(),
           angularMomentum,
           estimates.getCenterOfMassControlZUPFrame(),
           fastWalkingParametersForStep.getOmegaX(),
           fastWalkingParametersForStep.getOmegaY(),
           estimates::getGravity,
           0.0,
           0.2,
           parentRegistry);
   }

   //TODO swingDuration and doubleSupportFraction, omegaX,, omegaY should be changed in YoDouble
   public AutomaticFootstepMPCMatrixCalculator(RobotSide supportSide,
                                               DoubleProvider mass,
                                               double deltaT,
                                               Vector2DReadOnly desiredXYVelocity,
                                               DoubleProvider swingDuration,
                                               FrameVector3DReadOnly angularMomentum,
                                               ReferenceFrame controlFrame,
                                               DoubleProvider omegaX,
                                               DoubleProvider omegaY,
                                               DoubleProvider gravity,
                                               double desiredTurningVelocity,
                                               double stanceWidth,
                                               YoRegistry parentRegistry)
   {
      this.mass = mass;
      this.gravity = gravity;
      this.controlFrame = controlFrame;

      this.desiredXYVelocity = desiredXYVelocity;
      this.swingDuration = swingDuration;
      //TODO add momentum about stance foot

      FrameVector2D momentum2D = new FrameVector2D();
      momentum2D.set(angularMomentum.getX(), angularMomentum.getY());

      this.omegaX = omegaX;
      this.omegaY = omegaY;

      //TODO check how call the CoM height from the ground at once.
      comZUpFrameX = gravity.getValue() / (omegaX.getValue() * omegaX.getValue());
      comZUpFrameY = gravity.getValue() / (omegaY.getValue() * omegaY.getValue());

      massComZUpXOmegaX = mass.getValue() * comZUpFrameX * omegaX.getValue(); //
      massComZUpYOmegaY = mass.getValue() * comZUpFrameY * omegaY.getValue();//

      massComZUpX = mass.getValue() * comZUpFrameX;
      massComZUpY = mass.getValue() * comZUpFrameY;
      massGravity = mass.getValue() * gravity.getValue();

      // Need updating
      //TODO find better way to get the turning speed
      nominalStanceWidth = stanceWidth;

      firstFootAngle = swingDuration.getValue() * desiredTurningVelocity;
      secondFootAngle = swingDuration.getValue() * desiredTurningVelocity + firstFootAngle;

      optimizedSolution = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);
      previousOptimizedSolution = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);

      currentTickStateVector = new DMatrixRMaj(STATE_SIZE, 1);

      lowerBoundOfDistanceLimit = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);
      upperBoundOfDistanceLimit = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);

      distanceLimitConstraintMatrix = new DMatrixRMaj(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);

      lowerBoundOfEllipseLimit = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);
      upperBoundOfEllipseLimit = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);
      ellipseLimitConstraintMatrix = new DMatrixRMaj(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);

      desiredTwoStepAheadStatesVector = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, 1);

      weightQMatrix = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, SIZE_OF_FUTURE_STATES);

      String prefix = supportSide.getLowerCaseName() + "MPCCalculator";
      YoRegistry registry = new YoRegistry(prefix);

      angularMomentumInMPC = new YoVector2D(prefix + "AngularMomentumInMPC", registry);
      yoAngularMomentumInContactPoint = new YoVector2D(prefix + "AngularMomentumInContactPoint", registry);

      yoCurrentTickStateVector = new YoMatrix(prefix + "CurrentTickStatesVector", 4, 1, registry);

      //      whichFootIsSupportFoot = new YoDouble(prefix + "SupportSideInMPCCalculator", parentRegistry);
      mpcSolution = new YoMatrix(prefix + "MPCSolution", 4, 1, registry);

      yoPredictedStateAtEndOfCurrentPhase = new YoMatrix(prefix + "PredictedStateAtEndOfCurrentPhase", 4, 1, registry);
      desiredTwoStepStates = new YoMatrix(prefix + "DesiredTwoStepStates", 8, 1, registry);
      yoOptimalTwoStepStates = new YoMatrix(prefix + "OptimalTwoStepStates", 8, 1, registry);
      pendulumBaseInMPC = new YoVector2D(prefix + "PendulumBaseInMPC", registry);
      yoCapturePointInMPC = new YoVector2D(prefix + "CapturePointInMPC", registry);
      savedInitialStateVector = new YoMatrix(prefix + "SavedInitialStateVector", 4, 1, registry);
      differenceOfDesiredMeasuredCoP = new YoVector2D(prefix + "DifferenceOfDesiredMeasureCoP", registry);
      supportFootPosition = new YoVector2D(prefix + "SupportFootPosition", registry);
      angularMomentumOffsetLeftSupport = new YoDouble(prefix + "AngularMomentumOffsetLeftSupport", registry);
      angularMomentumOffsetRightSupport = new YoDouble(prefix + "AngularMomentumOffsetRightSupport", registry);

      comPositionRelativeToContact = new YoFrameVector2D(prefix + "CoMPositionRelativeToContact", controlFrame, registry);

      aTransitionMatrix = new YoMatrix(prefix + "ATransitionMatrix", 4, 4, registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   private void computeAngularMomentumInContactPoint(FrameVector3DReadOnly angularMomentumInCoM,
                                                     FrameVector2D comVelocity,
                                                     FrameVector3D angularMomentumInContactPoint)
   {
      double deltaZInX = gravity.getValue() / MathTools.square(omegaX.getValue());
      double deltaZInY = gravity.getValue() / MathTools.square(omegaY.getValue());

      angularMomentumInContactPoint.setX(angularMomentumInCoM.getX() - deltaZInY * comVelocity.getY() * mass.getValue());
      angularMomentumInContactPoint.setY(angularMomentumInCoM.getY() + deltaZInX * comVelocity.getX() * mass.getValue());

      yoAngularMomentumInContactPoint.set(angularMomentumInContactPoint);
   }

   private final DMatrixRMaj steadyStateAX = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj steadyStateAY = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj steadyStateSelectionX = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj steadyStateSelectionY = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj steadyStateBX = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj steadyStateBY = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj steadStateInitialSolutionDecoupled = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj steadStateInitialSolution = new DMatrixRMaj(4, 1);

   private void computeSteadyStateVector(RobotSide supportSide, DMatrixRMaj steadyState)
   {
      steadyStateSelectionX.set(0, 0, -1.0);
      steadyStateSelectionX.set(1, 1, -1.0);

      steadyStateAX.set(0, 0, A.get(0, 0));
      steadyStateAX.set(0, 1, A.get(0, 3));
      steadyStateAX.set(1, 0, A.get(3, 0));
      steadyStateAX.set(1, 1, A.get(3, 3));

      CommonOps_DDRM.addEquals(steadyStateAX, steadyStateSelectionX);

      steadyStateBX.zero();
      steadyStateBX.set(0, 0, desiredXYVelocity.getX() * swingDuration.getValue());

      solver.setA(steadyStateAX);
      solver.solve(steadyStateBX, steadStateInitialSolutionDecoupled);
      steadStateInitialSolution.set(0, 0, steadStateInitialSolutionDecoupled.get(0, 0));
      steadStateInitialSolution.set(3, 0, steadStateInitialSolutionDecoupled.get(1, 0));


      steadyStateSelectionY.set(0, 0, -1.0);
      steadyStateSelectionY.set(1, 1, 1.0);

      // first goal is tha
      steadyStateAY.set(0, 0, A.get(1, 1) - 1.0);// ftur
      steadyStateAY.set(0, 1, A.get(1, 2)); // future of y state
      steadyStateAY.set(1, 0, 1.0);

      steadyStateBY.zero();
      steadyStateBY.set(1, 0, desiredXYVelocity.getY() * swingDuration.getValue() + supportSide.negateIfLeftSide(0.5 * nominalStanceWidth));

      solver.setA(steadyStateAY);
      solver.solve(steadyStateBY, steadStateInitialSolutionDecoupled);
      //f ixme
      steadStateInitialSolution.set(1, 0, steadStateInitialSolutionDecoupled.get(0, 0));
      steadStateInitialSolution.set(2, 0, steadStateInitialSolutionDecoupled.get(1, 0));

      CommonOps_DDRM.mult(A, steadStateInitialSolution, steadyState);
   }


   private void calculateInitialStateVectorRelatedCoM(FrameVector2DReadOnly comPositionRelativeToBase,
                                                      FrameVector3DReadOnly angularMomentumAboutContactPoint,
                                                      double timeToReachGoal,
                                                      DMatrixRMaj initialStatesVectorToPack)
   {
      currentTickStateVector.zero();
      // FIXMe this should be negative
      currentTickStateVector.set(0, 0, comPositionRelativeToBase.getX());
      currentTickStateVector.set(1, 0, comPositionRelativeToBase.getY());
      currentTickStateVector.set(2, 0, angularMomentumAboutContactPoint.getX());
      currentTickStateVector.set(3, 0, angularMomentumAboutContactPoint.getY());

      double heightX = gravity.getValue() / (omegaX.getValue() * omegaX.getValue());
      double heightY = gravity.getValue() / (omegaY.getValue() * omegaY.getValue());
      double mhX = mass.getValue() * heightX; //
      double mhY = mass.getValue() * heightY; //
      double massComZUpXOmegaX = mhX * omegaX.getValue();
      double massComZUpYOmegaY = mhY * omegaY.getValue();

      FootstepMPCMatrixTools.computeStateTransitionMatrix(omegaX.getValue(),
                                                          omegaY.getValue(),
                                                          timeToReachGoal,
                                                          massComZUpXOmegaX,
                                                          massComZUpYOmegaY,
                                                          ARemaining);

      //      System.out.println("angular momentum : " + angularMomentum);
      CommonOps_DDRM.mult(ARemaining, currentTickStateVector, initialStatesVectorToPack);
      yoPredictedStateAtEndOfCurrentPhase.set(initialStatesVectorToPack);

      yoCurrentTickStateVector.set(currentTickStateVector);
   }

   // TODO it
   // Should I split these constraint methods into different source files?

   /**
    * Method for setting the distance constraint.
    * Simple length and width will be constrained in future foot steps.
    *
    * @param lengthMax the maximum length in x-direction
    * @param lengthMin the minimum length in x-direction
    * @param widthMax  the maximum width in y-direction
    * @param widthMin  the minimum width in y-direction
    * @param robotSide tells which foot is support side
    */
   public void computeDistanceConstraint(double lengthMax, double lengthMin, double widthMax, double widthMin, RobotSide robotSide)
   {
      lowerBoundOfDistanceLimit.set(0, 0, lengthMin);
      lowerBoundOfDistanceLimit.set(1, 0, widthMin);
      lowerBoundOfDistanceLimit.set(2, 0, lengthMin);
      lowerBoundOfDistanceLimit.set(3, 0, widthMin);

      upperBoundOfDistanceLimit.set(0, 0, lengthMax);
      upperBoundOfDistanceLimit.set(1, 0, widthMax);
      upperBoundOfDistanceLimit.set(2, 0, lengthMax);
      upperBoundOfDistanceLimit.set(3, 0, widthMax);

      CommonOps_DDRM.setIdentity(distanceLimitConstraintMatrix);
      distanceLimitConstraintMatrix.set(1, 1, robotSide.negateIfLeftSide(1.0));
      distanceLimitConstraintMatrix.set(3, 3, -robotSide.negateIfLeftSide(1.0));
   }

   public DMatrixRMaj getLowerBoundOfDistanceLimit()
   {
      return lowerBoundOfDistanceLimit;
   }

   public DMatrixRMaj getUpperBoundOfDistanceLimit()
   {
      return upperBoundOfDistanceLimit;
   }

   public DMatrixRMaj getDistanceLimitConstraintMatrix()
   {
      return distanceLimitConstraintMatrix;
   }

   public void computeEllipseConstraint(double lengthMax, double widthMax, double widthOffset, RobotSide robotSide)
   {
      lowerBoundOfEllipseLimit.set(0, 0, -lengthMax);
      lowerBoundOfEllipseLimit.set(1, 0, -Math.pow(widthMax, 2.0) / lengthMax + robotSide.negateIfLeftSide(1.0) * widthOffset);
      lowerBoundOfEllipseLimit.set(2, 0, -lengthMax);
      lowerBoundOfEllipseLimit.set(1, 0, -Math.pow(widthMax, 2.0) / lengthMax - robotSide.negateIfLeftSide(1.0) * widthOffset);

      upperBoundOfEllipseLimit.set(0, 0, lengthMax);
      upperBoundOfEllipseLimit.set(1, 0, Math.pow(widthMax, 2.0) / lengthMax + robotSide.negateIfLeftSide(1.0) * widthOffset);
      upperBoundOfEllipseLimit.set(2, 0, lengthMax);
      upperBoundOfEllipseLimit.set(1, 0, Math.pow(widthMax, 2.0) / lengthMax - robotSide.negateIfLeftSide(1.0) * widthOffset);

      CommonOps_DDRM.setIdentity(ellipseLimitConstraintMatrix);
      ellipseLimitConstraintMatrix.set(0, 0, Math.sqrt(1.0 + Math.pow(widthMax, 2.0) / Math.pow(lengthMax, 2.0)));
      ellipseLimitConstraintMatrix.set(2, 2, Math.sqrt(1.0 + Math.pow(widthMax, 2.0) / Math.pow(lengthMax, 2.0)));
   }

   public DMatrixRMaj getLowerBoundOfEllipseLimit()
   {
      return lowerBoundOfEllipseLimit;
   }

   public DMatrixRMaj getUpperBoundOfEllipseLimit()
   {
      return upperBoundOfEllipseLimit;
   }

   public DMatrixRMaj getEllipseLimitConstraintMatrix()
   {
      return ellipseLimitConstraintMatrix;
   }

   // Examine the output results by inputting optimized value to the object function.

   public boolean firstTickAfterLeavingDoubleSupport = false;

   private final FramePoint2D pendulumBase = new FramePoint2D();
   private final FramePoint3D tempPoint = new FramePoint3D();

   // to check the feasibility of one-step ahead foot step generation.
   public void computeTwoStepMPCTouchdown(BPWPlanarWalkingRobotEstimates estimates,
                                          double timeToReachGoal,
                                          FramePoint2DReadOnly pendulumBase,
                                          RobotSide supportSide,
                                          YoDouble capturePointTimeConstantX,
                                          YoDouble capturePointTimeConstantY,
                                          YoFrameVector2D predictedALIPVelocityAtTouchdown,
                                          YoFramePoint2D desiredMPCTouchdownPosition,
                                          boolean leavingDoubleSupport)
   {
      // One foot is starting to swing for the robot

      tempVector.setIncludingFrame(estimates.getCenterOfMassVelocity());
      tempVector.changeFrameAndProjectToXYPlane(controlFrame);

      comVelocity.setIncludingFrame(estimates.getCenterOfMassVelocity());
      comVelocity.changeFrameAndProjectToXYPlane(controlFrame);


      angularMomentum.setIncludingFrame(estimates.getCentroidalAngularMomentum());
      angularMomentum.changeFrame(controlFrame);
      angularMomentumInMPC.set(angularMomentum);
      //      capturePointInMPC.set(estimates.getCapturePoint());
      capturePointInMPC.setIncludingFrame(estimates.getCapturePoint());
      capturePointInMPC.changeFrameAndProjectToXYPlane(controlFrame);

      yoCapturePointInMPC.set(capturePointInMPC);



      supportFootPoint.setIncludingFrame(estimates.getFootPosition(supportSide));
      supportFootPoint.changeFrameAndProjectToXYPlane(estimates.getCenterOfMassControlZUPFrame());

      computeAngularMomentumInContactPoint(angularMomentum, comVelocity, angularMomentumAboutContactPoint);

      if (leavingDoubleSupport)
      {
         // We assume that we haven't ticked at all when in swing, so we go into this method the first tick of swing support
         if (!firstTickAfterLeavingDoubleSupport)
         {
            // This is set to true so the next time we go in this method in the update look, this won't be the first tick in swing
            //            firstTickAfterLeavingDoubleSupport = true;
            //            savedInitialStateVector.set(oneStepPredictedInitialVector);
            differenceOfDesiredMeasuredCoP.set(pendulumBase);
            differenceOfDesiredMeasuredCoP.sub(capturePointInMPC);

            supportFootPosition.set(supportFootPoint);
            LogTools.info("Leaving double support");
         }
      }
      else // When we are entering double support, we go into this else, and we set the first tick to false becase we went into double support
      {
         LogTools.info("Entering double support");
         firstTickAfterLeavingDoubleSupport = false;
      }

      if (pendulumBase != null && Double.isFinite(timeToReachGoal))
      {
         double tanhHalfYSwingT = Math.tanh(omegaY.getValue() * swingDuration.getValue() * 0.5);

         this.pendulumBase.setIncludingFrame(pendulumBase);
         this.pendulumBase.changeFrame(controlFrame);
         tempPoint.setIncludingFrame(estimates.getCenterOfMass());
         tempPoint.changeFrame(controlFrame);
         comPositionRelativeToContact.set(tempPoint);
         comPositionRelativeToContact.sub(this.pendulumBase);

         pendulumBaseInMPC.set(pendulumBase);

         //output is "oneStepMPCInitialVector"
         calculateInitialStateVectorRelatedCoM(comPositionRelativeToContact, angularMomentumAboutContactPoint, timeToReachGoal, predictedStateAtEndOfCurrentPhase);

         aTransitionMatrix.set(ARemaining);

         computeStateTransitionMatrices(omegaX.getValue(), omegaY.getValue(), swingDuration.getValue());

         if (!firstTickAfterLeavingDoubleSupport)
         {
            firstTickAfterLeavingDoubleSupport = true;
            savedInitialStateVector.set(predictedStateAtEndOfCurrentPhase);
         }

         computeAngularMomentumOffset(savedInitialStateVector);

         //         computeIdealStepPosition(swingDuration.getValue(), nominalStanceWidth, supportSide);
         computeDesiredTwoStepAheadStates(omegaX.getValue(),
                                          omegaY.getValue(),
                                          swingDuration.getValue(),
                                          tanhHalfYSwingT,
                                          nominalStanceWidth,
                                          massComZUpXOmegaX,
                                          massComZUpYOmegaY,
                                          angularMomentumOffsetLeftSupport.getDoubleValue(),
                                          //                                          0.0,
                                          // 3300
                                          angularMomentumOffsetRightSupport.getDoubleValue(),
                                          //                                          0.0,
                                          // 3500
                                          supportSide,
                                          predictedStateAtEndOfCurrentPhase);

         double lowWeight = 5.0;
         double highWeight = 20.0;
         CommonOps_DDRM.setIdentity(weightQMatrix);
         // Set a to converge in X in one step, and Y in two
         weightQMatrix.set(0, 0, highWeight);
         weightQMatrix.set(1, 1, lowWeight);
         weightQMatrix.set(2, 2, lowWeight / mass.getValue());
         weightQMatrix.set(3, 3, highWeight / mass.getValue());
         weightQMatrix.set(4, 4, lowWeight);
         weightQMatrix.set(5, 5, highWeight);
         weightQMatrix.set(6, 6, highWeight / mass.getValue());
         weightQMatrix.set(7, 7, lowWeight / mass.getValue());

         // TODO extract this value
         MatrixTools.setDiagonal(rateRegularizationWeightMatrix, rateRegularizationWeight);

         idealStepWeightMatrix.zero();
         idealStepWeightMatrix.set(0, 0, idealStepWeight);
         idealStepWeightMatrix.set(1, 1, idealStepWeight);
         idealStepWeightMatrix.set(4, 4, idealStepWeight);
         idealStepWeightMatrix.set(5, 5, idealStepWeight);

         costFunctionHessian.zero();
         costFunctionGradient.zero();

         addTwoStepAheadTrackingCost(weightQMatrix);
         //                  addIdealStepTrackingCost(idealStepWeightMatrix);
         addRateRegularizationTask(rateRegularizationWeightMatrix, previousOptimizedSolution);

         CommonOps_DDRM.scale(-1.0, costFunctionGradient);
         solver.setA(costFunctionHessian);
         solver.solve(costFunctionGradient, optimizedSolution);

         previousOptimizedSolution.set(optimizedSolution);
         mpcSolution.set(optimizedSolution);

         CommonOps_DDRM.mult(stackedStateTransitionMatrix, predictedStateAtEndOfCurrentPhase, optimalTwoStepStates);
         CommonOps_DDRM.multAdd(stackedControlInputMatrix, optimizedSolution, optimalTwoStepStates);
         yoOptimalTwoStepStates.set(optimalTwoStepStates);

         //         desiredMPCTouchdownPosition.set(optimizedSolution.get(0, 0) + pendulumBase.getX(), optimizedSolution.get(1, 0) + pendulumBase.getY());
         desiredMPCTouchdownPosition.set(optimizedSolution);
      }
      else
      {
         predictedALIPVelocityAtTouchdown.set(tempVector);
         predictedALIPVelocityAtTouchdown.addX(1.0 / massComZUpX * angularMomentum.getY());
         predictedALIPVelocityAtTouchdown.addY(-1.0 / massComZUpY * angularMomentum.getX());

         desiredMPCTouchdownPosition.set(capturePointTimeConstantX.getDoubleValue() * predictedALIPVelocityAtTouchdown.getX(),
                                         capturePointTimeConstantY.getDoubleValue() * predictedALIPVelocityAtTouchdown.getY());
      }
   }

   private void computeAngularMomentumOffset(YoMatrix predictedInitialVector)
   {
      if (predictedInitialVector.get(2, 0) < 0) //left support
      {
         if (predictedInitialVector.get(2, 0) < -250.0)
         {
            angularMomentumOffsetLeftSupport.set(2100);
            angularMomentumOffsetRightSupport.set(4000.0);
         }
         if (predictedInitialVector.get(2, 0) > -250.0 && predictedInitialVector.get(2, 0) < -150)
         {
            angularMomentumOffsetLeftSupport.set(2800.0);
            angularMomentumOffsetRightSupport.set(3500.0);
         }
         if (predictedInitialVector.get(2, 0) > -150.0 && predictedInitialVector.get(2, 0) < -100)
         {
            angularMomentumOffsetLeftSupport.set(4000);
            angularMomentumOffsetRightSupport.set(3700.0);
         }
      }
      else // right support
      {
         if (predictedInitialVector.get(2, 0) >= 100 && predictedInitialVector.get(2, 0) < 150)
         {
            angularMomentumOffsetLeftSupport.set(3100.0);
            angularMomentumOffsetRightSupport.set(3500.0);
         }
         if (predictedInitialVector.get(2, 0) >= 140)// && predictedInitialVector.get(2,0) < 150)
         {
            angularMomentumOffsetLeftSupport.set(2000);
            angularMomentumOffsetRightSupport.set(2000);
         }
      }
   }

   private void computeStateTransitionMatrices(double omegaX, double omegaY, double swingDuration)
   {
      FootstepMPCMatrixTools.computeStateTransitionMatrix(omegaX, omegaY, swingDuration, massComZUpXOmegaX, massComZUpYOmegaY, A);
      FootstepMPCMatrixTools.computeJumpMatrices(AJump, BJump);
      CommonOps_DDRM.mult(A, AJump, AHat);
      CommonOps_DDRM.mult(A, BJump, BHat);

      CommonOps_DDRM.mult(AHat, AHat, AHatAHat);
      CommonOps_DDRM.mult(AHat, BHat, AHatBHat);

      stackedStateTransitionMatrix.zero();
      MatrixTools.setMatrixBlock(stackedStateTransitionMatrix, 0, 0, AHat, 0, 0, STATE_SIZE, STATE_SIZE, 1.0);
      MatrixTools.setMatrixBlock(stackedStateTransitionMatrix, STATE_SIZE, 0, AHatAHat, 0, 0, STATE_SIZE, STATE_SIZE, 1.0);

      stackedControlInputMatrix.zero();
      MatrixTools.setMatrixBlock(stackedControlInputMatrix, 0, 0, BHat, 0, 0, STATE_SIZE, CONTROL_SIZE, 1.0);
      MatrixTools.setMatrixBlock(stackedControlInputMatrix, STATE_SIZE, 0, AHatBHat, 0, 0, STATE_SIZE, CONTROL_SIZE, 1.0);
      MatrixTools.setMatrixBlock(stackedControlInputMatrix, STATE_SIZE, CONTROL_SIZE, BHat, 0, 0, STATE_SIZE, CONTROL_SIZE, 1.0);
   }

   private void computeIdealStepPosition(double stepDuration, double stepWidth, RobotSide supportSide)
   {
      double xDelta = desiredXYVelocity.getX() * stepDuration / 2.0;
      double yDelta = desiredXYVelocity.getY() * stepDuration / 2.0;
      double halfWidth = stepWidth / 2.0;

      // position at the end of the next step
      idealStepVector.set(0, 0, xDelta);
      idealStepVector.set(1, 0, yDelta + supportSide.negateIfRightSide(halfWidth));
      // position at the end of the step after that
      idealStepVector.set(4, 0, xDelta);
      idealStepVector.set(5, 0, yDelta + supportSide.negateIfLeftSide(halfWidth));
   }

   private void computeDesiredTwoStepAheadStates(double omegaX,
                                                 double omegaY,
                                                 double stepDuration,
                                                 double tanhHalfYT,
                                                 double stepWidth,
                                                 double massComZUpXOmegaX,
                                                 double massComZUpYOmegaY,
                                                 double angularMomentumXLeftSupportOffset,
                                                 double angularMomentumXRightSupportOffset,
                                                 RobotSide supportSide,
                                                 DMatrixRMaj initialStateVector)
   {
      double coshXT = Math.cosh(omegaX * stepDuration);
      double sinhXT = Math.sinh(omegaX * stepDuration);
      double coshYT = Math.cosh(omegaY * stepDuration);
      double sinhYT = Math.sinh(omegaY * stepDuration);
      double omegaXTimeDuration = omegaX * stepDuration;
      double omegaYTimeDuration = omegaY * stepDuration;

      double angularMomentumOffsetFistStep = 0;
      double angularMomentumOffsetSecondStep = 0;

      if (supportSide.getOppositeSide() == RobotSide.LEFT) // right foot support
      {
         angularMomentumOffsetFistStep = angularMomentumXRightSupportOffset;
         angularMomentumOffsetSecondStep = angularMomentumXLeftSupportOffset;
      }
      if (supportSide.getOppositeSide() == RobotSide.RIGHT) // left foot support
      {
         angularMomentumOffsetFistStep = angularMomentumXLeftSupportOffset;
         angularMomentumOffsetSecondStep = angularMomentumXRightSupportOffset;
      }

      // TODO This should be done in a more organized manner
      desiredTwoStepAheadStatesVector.zero();
      //      desiredTwoStepAheadStatesVector.set(0, 0, (1.0 - coshXT) / (massComZUpXOmegaX * sinhXT) * initialStateVector.get(3, 0)); // x position
      //      // FIXME this is wrong, it should have the step width in there.
      desiredTwoStepAheadStatesVector.set(1,
                                          0,
                                          supportSide.negateIfLeftSide(1.0) * (1 + coshYT) * initialStateVector.get(2, 0) / (massComZUpYOmegaY
                                                                                                                             * sinhYT)); // y position

      desiredTwoStepAheadStatesVector.set(2,
                                          0,
                                          -supportSide.negateIfLeftSide(1.0) * massComZUpYOmegaY * stepWidth * tanhHalfYT + angularMomentumOffsetFistStep
                                                                                                                            * differenceOfDesiredMeasuredCoP.getY());//*(-oneStepPredictedInitialVector.get(2,0) + currentTickStateVector.get(2,0))); // Lx
      desiredTwoStepAheadStatesVector.set(3, 0, initialStateVector.get(3, 0)); // Ly

      desiredTwoStepAheadStatesVector.set(4, 0, 2 * (1 - coshXT) / (massComZUpXOmegaX * sinhXT) * initialStateVector.get(3, 0)); // x position
      desiredTwoStepAheadStatesVector.set(5,
                                          0,
                                          desiredTwoStepAheadStatesVector.get(1, 0)
                                          - supportSide.negateIfLeftSide(1.0) * (1 + coshYT) * desiredTwoStepAheadStatesVector.get(2, 0) / (massComZUpYOmegaY
                                                                                                                                            * sinhYT)); // y position
      desiredTwoStepAheadStatesVector.set(6,
                                          // Lx
                                          0,
                                          supportSide.negateIfLeftSide(1.0) * massComZUpYOmegaY * stepWidth * tanhHalfYT
                                          + angularMomentumOffsetSecondStep * differenceOfDesiredMeasuredCoP.getY());
      desiredTwoStepAheadStatesVector.set(7, 0, initialStateVector.get(3, 0)); // Ly

      // TODO override with teh steady state values
      computeSteadyStateVector(supportSide.getOppositeSide(), desiredStateVector);
      CommonOps_DDRM.insert(desiredStateVector, desiredTwoStepAheadStatesVector, 0, 0);
      computeSteadyStateVector(supportSide, desiredStateVector);
      CommonOps_DDRM.insert(desiredStateVector, desiredTwoStepAheadStatesVector, STATE_SIZE, 0);

      //  use different manner of desired(reference) states.
      //      desiredTwoStepAheadStatesVector.set(0, 0, 1 / massComZUpXOmegaX * Math.tanh(0.5 * omegaXTimeDuration) * initialStateVector.get(3, 0));
      //      desiredTwoStepAheadStatesVector.set(1, 0, -0.5 * stepWidth * supportSide.negateIfRightSide(1.0));
      //      desiredTwoStepAheadStatesVector.set(2,
      //                                          0,
      //                                          0.5 * supportSide.negateIfRightSide(1.0) * massComZUpYOmegaY * stepDuration * Math.tanh(0.5 * omegaYTimeDuration) - supportSide.negateIfLeftSide(150.0));
      //      desiredTwoStepAheadStatesVector.set(3,0,initialStateVector.get(3, 0)) ;
      //
      //      desiredTwoStepAheadStatesVector.set(4,0, 2*1 / massComZUpXOmegaX * Math.tanh(0.5 * omegaXTimeDuration) * initialStateVector.get(3, 0));
      //      desiredTwoStepAheadStatesVector.set(5, 0, 0.5 * stepWidth * supportSide.negateIfRightSide(1.0));
      //      desiredTwoStepAheadStatesVector.set(6,
      //                                          0,
      //                                          -0.5 * supportSide.negateIfRightSide(1.0) * massComZUpYOmegaY * stepDuration * Math.tanh(0.5 * omegaYTimeDuration) - supportSide.negateIfRightSide(100.0));
      //      desiredTwoStepAheadStatesVector.set(7,0,initialStateVector.get(3, 0)) ;

      //      desiredTwoStepAheadStatesVector.set(0, 0, 0.0); // assumes zero x velocity for the first step position
      //      desiredTwoStepAheadStatesVector.set(4, 0, 0.0); // assumes zero x velocity for the second step position
      //
      //      desiredTwoStepAheadStatesVector.set(1, 0, supportSide.negateIfLeftSide(stepWidth / 2.0)); // assumes zero velocity for the first step position
      //      desiredTwoStepAheadStatesVector.set(5, 0, supportSide.negateIfRightSide(stepWidth / 2.0)); // assumes zero velocity for the first step position
      //
      //      desiredTwoStepAheadStatesVector.set(3, 0, 0.0); // assumes zero x velocity
      //      desiredTwoStepAheadStatesVector.set(7, 0, 0.0); // assumes zero x velocity

      desiredTwoStepStates.set(desiredTwoStepAheadStatesVector);
   }

   private final DMatrixRMaj objective = new DMatrixRMaj(8, 1);
   private final DMatrixRMaj jacobian = new DMatrixRMaj(8, 4);

   private void addTwoStepAheadTrackingCost(DMatrixRMaj Q)
   {
      // This cost is trying to achieve the desired tracking. That is, minimize (x - x_des). Since x = A x_0 + B u, this can be formulated as
      // min (A x_0 + B u - x_des)^T Q (A x_0 + B u - x_des). This is then equivalent to saying min (J u - b)^T Q (J u - b), where J = B and b = x_des - A x_0
      jacobian.set(stackedControlInputMatrix);
      objective.set(desiredTwoStepAheadStatesVector);
      CommonOps_DDRM.multAdd(-1.0, stackedStateTransitionMatrix, predictedStateAtEndOfCurrentPhase, objective);

      addObjectiveToCostFunction(jacobian, objective, Q);
   }

   private void addIdealStepTrackingCost(DMatrixRMaj weight)
   {
      // This cost is trying to achieve the desired tracking. That is, minimize (x - x_des). Since x = A x_0 + B u, this can be formulated as
      // min (A x_0 + B u - x_des)^T Q (A x_0 + B u - x_des). This is then equivalent to saying min (J u - b)^T Q (J u - b), where J = B and b = x_des - A x_0
      jacobian.set(stackedControlInputMatrix);
      objective.set(idealStepVector);
      CommonOps_DDRM.multAdd(-1.0, stackedStateTransitionMatrix, predictedStateAtEndOfCurrentPhase, objective);

      addObjectiveToCostFunction(jacobian, objective, weight);
   }

   private void addRateRegularizationTask(double weight, DMatrixRMaj previousOptimizedSolution)
   {
      MatrixTools.addDiagonal(costFunctionHessian, weight);
      CommonOps_DDRM.addEquals(costFunctionGradient, -weight, previousOptimizedSolution);
   }

   private void addRateRegularizationTask(DMatrixRMaj R, DMatrixRMaj previousOptimizedSolution)
   {
      CommonOps_DDRM.addEquals(costFunctionHessian, R);
      CommonOps_DDRM.multAdd(-1.0, R, previousOptimizedSolution, costFunctionGradient);
   }

   private final DMatrixRMaj weightTimesJacobian = new DMatrixRMaj(8, 8);

   /**
    * Adds a generic task to the cost function that is formulated as (J x - b)^T Q (J x - b)
    *
    * @param taskJacobian  J in the above equation
    * @param taskObjective b in the above equation
    * @param weight        Q in the above equation
    */
   private void addObjectiveToCostFunction(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj weight)
   {
      int inputSize = weight.getNumRows();
      weightTimesJacobian.reshape(inputSize, NUMBER_OF_VARIABLES);
      CommonOps_DDRM.mult(weight, taskJacobian, weightTimesJacobian);

      //F IXME why is there no negative sign?
      CommonOps_DDRM.multAddTransA(taskJacobian, weightTimesJacobian, costFunctionHessian);
      CommonOps_DDRM.multAddTransA(-1.0, taskObjective, weightTimesJacobian, costFunctionGradient);
   }

   /**
    * Adds a generic task to the cost function that is formulated as (J x - b)^T Q (J x - b)
    *
    * @param taskJacobian  J in the above equation
    * @param taskObjective b in the above equation
    * @param weight        Q in the above equation
    */
   private void addObjectiveToCostFunction(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double weight)
   {
      CommonOps_DDRM.multAddTransA(weight, taskJacobian, taskJacobian, costFunctionHessian);
      CommonOps_DDRM.multAddTransA(-weight, taskObjective, taskJacobian, costFunctionGradient);
   }
}


