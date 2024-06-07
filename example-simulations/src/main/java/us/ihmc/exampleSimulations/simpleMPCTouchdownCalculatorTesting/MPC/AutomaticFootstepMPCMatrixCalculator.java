package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.BPWPlanarWalkingRobotEstimates;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
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
   private static final double similarityOfStepsWeight = 2.0;
   private static final double firstStepSteadyStateConvergenceWeight = 10.0;
   private static final double finalSteadyStateConvergenceWeight = 25.0;
   private static final double desiredStepWeight = 0.0;

   public static final int STATE_SIZE = 4;
   public static final int CONTROL_SIZE = 2;
   private static final int NUMBER_OF_STEPS = 2;
   public static final int NUMBER_OF_VARIABLES = CONTROL_SIZE * NUMBER_OF_STEPS;
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

   private final FootstepMPCMatrixTools mpcMatrixTools = new FootstepMPCMatrixTools();

   //TODO should change below data type. this is temporally value
   //   private final FixedFramePoint2DBasics pendulumBaseInSoleFrame;
   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D angularMomentumAboutContactPoint = new FrameVector3D();
   private final DoubleProvider nominalStanceWidth;
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

   private final DMatrixRMaj stackedStateTransitionMatrix = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, STATE_SIZE);
   private final DMatrixRMaj stackedControlInputMatrix = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, NUMBER_OF_VARIABLES);

   private final DMatrixRMaj desiredTwoStepAheadStatesVector;
   private final DMatrixRMaj steadyStateVector;
   private final YoMatrix desiredTwoStepStates;
   private final DMatrixRMaj desiredStateVector = new DMatrixRMaj(STATE_SIZE, 1);
   private final DMatrixRMaj idealStepVector = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, 1);
   private final DMatrixRMaj costFunctionHessian = new DMatrixRMaj(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);
   private final DMatrixRMaj costFunctionGradient = new DMatrixRMaj(NUMBER_OF_VARIABLES, 1);
   private final DMatrixRMaj weightQMatrix;
   private final DMatrixRMaj weightDesiredStep;
   private final DMatrixRMaj rateRegularizationWeightMatrix = new DMatrixRMaj(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);
   private final DMatrixRMaj similiarityOfStepsWeightMatrix = new DMatrixRMaj(CONTROL_SIZE, CONTROL_SIZE);
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

   private final PoseReferenceFrame endOfCurrentStateFrame;
   private final PoseReferenceFrame firstStepFrame;
   private final PoseReferenceFrame endOfFirstStepFrame;
   private final PoseReferenceFrame secondStepFrame;
   private final PoseReferenceFrame endOfSecondStepFrame;

   private final YoFramePoint2D solutionStepLocation0;
   private final YoFramePoint2D solutionStepLocation1;

   private final YoFramePoint2D solutionStepLocation0InWorld;
   private final YoFramePoint2D solutionStepLocation1InWorld;

   private final YoFramePoint2D endOfCurrentStep;
   private final YoFramePoint2D endOfFirstStep;
   private final YoFramePoint2D endOfSecondStep;

   private final YoMatrix aTransitionMatrix;

   //TODO Change this into final. for now, to escape the error, don't justify as final value.
   public AutomaticFootstepMPCMatrixCalculator(RobotSide supportSide,
                                               FrameVector3DReadOnly angularMomentum,
                                               BPWPlanarWalkingRobotEstimates estimates,
                                               Vector2DReadOnly desiredVelocityProvider,
                                               BPWPlanarWalkerParameters parameters,
                                               YoRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry)
   {
      this(supportSide,
           estimates::getTotalMass,
           desiredVelocityProvider,
           parameters.getSwingDuration(),
           angularMomentum,
           estimates.getCenterOfMassControlZUPFrame(),
           parameters.getOmegaX(),
           parameters.getOmegaY(),
           estimates::getGravity,
           () -> 0.0,
           parameters::getStanceWidth,
           parentRegistry,
           graphicsListRegistry);
   }

   //TODO swingDuration and doubleSupportFraction, omegaX,, omegaY should be changed in YoDouble
   public AutomaticFootstepMPCMatrixCalculator(RobotSide supportSide,
                                               DoubleProvider mass,
                                               Vector2DReadOnly desiredXYVelocity,
                                               DoubleProvider swingDuration,
                                               FrameVector3DReadOnly angularMomentum,
                                               ReferenceFrame controlFrame,
                                               DoubleProvider omegaX,
                                               DoubleProvider omegaY,
                                               DoubleProvider gravity,
                                               DoubleProvider desiredTurningVelocity,
                                               DoubleProvider stanceWidth,
                                               YoRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry)
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

      firstFootAngle = swingDuration.getValue() * desiredTurningVelocity.getValue();
      secondFootAngle = swingDuration.getValue() * desiredTurningVelocity.getValue() + firstFootAngle;

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
      steadyStateVector = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, 1);

      weightQMatrix = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, SIZE_OF_FUTURE_STATES);
      weightDesiredStep = new DMatrixRMaj(SIZE_OF_FUTURE_STATES, SIZE_OF_FUTURE_STATES);

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

      solutionStepLocation0 = new YoFramePoint2D(prefix + "SolutionStepLocation0", controlFrame, registry);
      solutionStepLocation1 = new YoFramePoint2D(prefix + "SolutionStepLocation1", controlFrame, registry);

      aTransitionMatrix = new YoMatrix(prefix + "ATransitionMatrix", 4, 4, registry);

      endOfCurrentStateFrame = new PoseReferenceFrame(prefix + "EndOfCurrentStateFrame", controlFrame);
      firstStepFrame = new PoseReferenceFrame(prefix + "FirstStepFrame", endOfCurrentStateFrame);
      endOfFirstStepFrame = new PoseReferenceFrame(prefix + "EndOfFirstStepFrame", firstStepFrame);
      secondStepFrame = new PoseReferenceFrame(prefix + "SecondStepFrame", endOfFirstStepFrame);
      endOfSecondStepFrame = new PoseReferenceFrame(prefix + "EndOfSecondStepFrame", secondStepFrame);

      solutionStepLocation0InWorld = new YoFramePoint2D(prefix + "SolutionStepLocation0InWorld", ReferenceFrame.getWorldFrame(), registry);
      solutionStepLocation1InWorld = new YoFramePoint2D(prefix + "SolutionStepLocation1InWorld", ReferenceFrame.getWorldFrame(), registry);

      endOfCurrentStep = new YoFramePoint2D(prefix + "EndOfCurrentStep", ReferenceFrame.getWorldFrame(), registry);
      endOfFirstStep = new YoFramePoint2D(prefix + "EndOfFirstStep", ReferenceFrame.getWorldFrame(), registry);
      endOfSecondStep = new YoFramePoint2D(prefix + "EndOfSecondStep", ReferenceFrame.getWorldFrame(), registry);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
         if (graphicsListRegistry != null)
         {
            YoGraphicPosition firstStepGraphic = new YoGraphicPosition(prefix + "FirstStepViz", solutionStepLocation0InWorld, 0.01, YoAppearance.Red(), YoGraphicPosition.GraphicType.SOLID_BALL);
            YoGraphicPosition secondStepGraphic = new YoGraphicPosition(prefix + "SecondStepViz", solutionStepLocation1InWorld, 0.005, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL);

            YoGraphicPosition endOfCurrentStepViz = new YoGraphicPosition(prefix + "endOfCurrentStepViz", endOfCurrentStep, 0.01, YoAppearance.Blue(), YoGraphicPosition.GraphicType.SOLID_BALL);
            YoGraphicPosition endOfFirstStepViz = new YoGraphicPosition(prefix + "endOfFirstStepViz", endOfFirstStep, 0.0075, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
            YoGraphicPosition endOfSecondStepViz = new YoGraphicPosition(prefix + "endOfSecondStepViz", endOfSecondStep, 0.005, YoAppearance.Blue(), YoGraphicPosition.GraphicType.CROSS);

            String name = prefix + "MPC";
            graphicsListRegistry.registerYoGraphic(name, firstStepGraphic);
            graphicsListRegistry.registerYoGraphic(name, secondStepGraphic);
            graphicsListRegistry.registerYoGraphic(name, endOfCurrentStepViz);
            graphicsListRegistry.registerYoGraphic(name, endOfFirstStepViz);
            graphicsListRegistry.registerYoGraphic(name, endOfSecondStepViz);

            graphicsListRegistry.registerArtifact(name, firstStepGraphic.createArtifact());
            graphicsListRegistry.registerArtifact(name, secondStepGraphic.createArtifact());
            graphicsListRegistry.registerArtifact(name, endOfCurrentStepViz.createArtifact());
            graphicsListRegistry.registerArtifact(name, endOfFirstStepViz.createArtifact());
            graphicsListRegistry.registerArtifact(name, endOfSecondStepViz.createArtifact());
         }
      }
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

         mpcMatrixTools.computeEndOfStepStateTransitionMatrices(omegaX.getValue(), omegaY.getValue(), gravity.getValue(), mass.getValue(), swingDuration.getValue(), stackedStateTransitionMatrix, stackedControlInputMatrix);

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
                                          nominalStanceWidth.getValue(),
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
         computeDesiredSteadyStates(omegaX.getValue(), omegaY.getValue(), supportSide);

         // Set a to converge in X in one step, and Y in two
         weightQMatrix.zero();
         weightQMatrix.set(0, 0, firstStepSteadyStateConvergenceWeight);
         weightQMatrix.set(1, 1, firstStepSteadyStateConvergenceWeight);
         weightQMatrix.set(2, 2, firstStepSteadyStateConvergenceWeight / mass.getValue());
         weightQMatrix.set(3, 3, firstStepSteadyStateConvergenceWeight / mass.getValue());
         weightQMatrix.set(4, 4, finalSteadyStateConvergenceWeight);
         weightQMatrix.set(5, 5, finalSteadyStateConvergenceWeight);
         weightQMatrix.set(6, 6, finalSteadyStateConvergenceWeight / mass.getValue());
         weightQMatrix.set(7, 7, finalSteadyStateConvergenceWeight / mass.getValue());

         MatrixTools.setDiagonal(weightDesiredStep, desiredStepWeight);
         // TODO extract this value
         MatrixTools.setDiagonal(rateRegularizationWeightMatrix, rateRegularizationWeight);

         similiarityOfStepsWeightMatrix.zero();
         similiarityOfStepsWeightMatrix.set(0, 0, similarityOfStepsWeight);
         similiarityOfStepsWeightMatrix.set(1, 1, similarityOfStepsWeight);

         costFunctionHessian.zero();
         costFunctionGradient.zero();

         addDesiredStepTrackingTask(weightDesiredStep);
         addSteadyStateTrackingTask(weightQMatrix);
         //                  addIdealStepTrackingCost(idealStepWeightMatrix);
         addSimilarityOfStepsTask(similiarityOfStepsWeightMatrix);
         addRateRegularizationTask(rateRegularizationWeightMatrix, previousOptimizedSolution);


         solveForOptimalFootsteps(desiredMPCTouchdownPosition);

         reconstructSolution();
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

   private void solveForOptimalFootsteps(FixedFramePoint2DBasics desiredMPCTouchdownPosition)
   {
      CommonOps_DDRM.scale(-1.0, costFunctionGradient);
      solver.setA(costFunctionHessian);
      solver.solve(costFunctionGradient, optimizedSolution);

      previousOptimizedSolution.set(optimizedSolution);
      mpcSolution.set(optimizedSolution);

      desiredMPCTouchdownPosition.set(optimizedSolution);
   }

   private final FramePose3D poseOfEndOfCurrentStep = new FramePose3D();
   private final FramePose3D poseOfFirstStep = new FramePose3D();
   private final FramePose3D poseOfEndOfFirstStep = new FramePose3D();
   private final FramePose3D poseOfSecondStep = new FramePose3D();
   private final FramePose3D poseOfEndOfSecondStep = new FramePose3D();

   private void reconstructSolution()
   {
      solutionStepLocation0.set(optimizedSolution);
      solutionStepLocation1.set(2, optimizedSolution);

      CommonOps_DDRM.mult(stackedStateTransitionMatrix, predictedStateAtEndOfCurrentPhase, optimalTwoStepStates);
      CommonOps_DDRM.multAdd(stackedControlInputMatrix, optimizedSolution, optimalTwoStepStates);
      yoOptimalTwoStepStates.set(optimalTwoStepStates);

      // TODO set some heights
      poseOfEndOfCurrentStep.setToZero(controlFrame);
      poseOfEndOfCurrentStep.getPosition().set(predictedStateAtEndOfCurrentPhase.get(0), predictedStateAtEndOfCurrentPhase.get(1), 0.0);
      poseOfEndOfCurrentStep.getPosition().sub(currentTickStateVector.get(0), currentTickStateVector.get(1), 0.0);
      endOfCurrentStateFrame.setPoseAndUpdate(poseOfEndOfCurrentStep);

      poseOfFirstStep.setToZero(endOfCurrentStateFrame);
      poseOfFirstStep.getPosition().set(solutionStepLocation0.getX(), solutionStepLocation0.getY(), 0.0);
      firstStepFrame.setPoseAndUpdate(poseOfFirstStep);

      poseOfEndOfFirstStep.setToZero(firstStepFrame);
      poseOfEndOfFirstStep.getPosition().set(optimalTwoStepStates.get(0), optimalTwoStepStates.get(1), 0.0);
      endOfFirstStepFrame.setPoseAndUpdate(poseOfEndOfFirstStep);

      poseOfSecondStep.setToZero(endOfFirstStepFrame);
      poseOfSecondStep.getPosition().set(solutionStepLocation1.getX(), solutionStepLocation1.getY(), 0.0);
      secondStepFrame.setPoseAndUpdate(poseOfSecondStep);

      poseOfEndOfSecondStep.setToZero(secondStepFrame);
      poseOfEndOfSecondStep.getPosition().set(optimalTwoStepStates.get(2), optimalTwoStepStates.get(3), 0.0);
      endOfSecondStepFrame.setPoseAndUpdate(poseOfEndOfSecondStep);

      solutionStepLocation0InWorld.setFromReferenceFrame(firstStepFrame);
      solutionStepLocation1InWorld.setFromReferenceFrame(secondStepFrame);
      endOfCurrentStep.setFromReferenceFrame(endOfCurrentStateFrame);
      endOfFirstStep.setFromReferenceFrame(endOfFirstStepFrame);
      endOfSecondStep.setFromReferenceFrame(endOfSecondStepFrame);
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

   private void computeDesiredSteadyStates(double omegaX,double omegaY, RobotSide supportSide)
   {
      mpcMatrixTools.computeSteadyEndState(supportSide, omegaX, omegaY, gravity.getValue(), mass.getValue(), swingDuration.getValue(), nominalStanceWidth.getValue(), desiredXYVelocity, steadyStateVector);
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


      desiredTwoStepStates.set(desiredTwoStepAheadStatesVector);
   }

   private final DMatrixRMaj objective = new DMatrixRMaj(8, 1);
   private final DMatrixRMaj jacobian = new DMatrixRMaj(8, 4);

   private void addDesiredStepTrackingTask(DMatrixRMaj Q)
   {
      // This cost is trying to achieve the desired tracking. That is, minimize (x - x_des). Since x = A x_0 + B u, this can be formulated as
      // min (A x_0 + B u - x_des)^T Q (A x_0 + B u - x_des). This is then equivalent to saying min (J u - b)^T Q (J u - b), where J = B and b = x_des - A x_0
      jacobian.set(stackedControlInputMatrix);
      objective.set(desiredTwoStepAheadStatesVector);
      CommonOps_DDRM.multAdd(-1.0, stackedStateTransitionMatrix, predictedStateAtEndOfCurrentPhase, objective);

      addObjectiveToCostFunction(jacobian, objective, Q);
   }

   private void addSteadyStateTrackingTask(DMatrixRMaj Q)
   {
      // This cost is trying to achieve the desired tracking. That is, minimize (x - x_des). Since x = A x_0 + B u, this can be formulated as
      // min (A x_0 + B u - x_des)^T Q (A x_0 + B u - x_des). This is then equivalent to saying min (J u - b)^T Q (J u - b), where J = B and b = x_des - A x_0
      jacobian.set(stackedControlInputMatrix);
      objective.set(steadyStateVector);
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

   private void addSimilarityOfStepsTask(DMatrixRMaj weight)
   {
      jacobian.reshape(2, 4);
      objective.reshape(2, 1);

      // x adjustments should be the same
      jacobian.zero();
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 2, -1.0);

      jacobian.set(1, 1, 1.0);
      jacobian.set(1, 3, -1.0);

      objective.zero();
      objective.set(0, 0, 0.0);
      objective.set(1, 0, nominalStanceWidth.getValue());

      addObjectiveToCostFunction(jacobian, objective, weight);
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

