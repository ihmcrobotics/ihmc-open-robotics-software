package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DGrowArray;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.data.IGrowArray;
import org.ejml.interfaces.linsol.LinearSolverSparse;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.CommonOps_DSCC;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

/**
 * <p>
 *    This is the main class of the trajectory-based CoM Trajectory Planner.
 * </p>
 * <p>
 *    This class assumes that the final phase is always the "stopping" phase, where the CoM is supposed to come to rest.
 *    This means that the final VRP is the terminal DCM location
 *  </p>
 *  <p>
 *     The CoM has the following definitions:
 *     <li>      x(t) = c<sub>0</sub> e<sup>&omega; t</sup> + c<sub>1</sub> e<sup>-&omega; t</sup> + c<sub>2</sub> t<sup>3</sup> + c<sub>3</sub> t<sup>2</sup> +
 *     c<sub>4</sub> t + c<sub>5</sub></li>
 *     <li> d/dt x(t) = &omega; c<sub>0</sub> e<sup>&omega; t</sup> - &omega; c<sub>1</sub> e<sup>-&omega; t</sup> + 3 c<sub>2</sub> t<sup>2</sup> +
 *     2 c<sub>3</sub> t+ c<sub>4</sub>
 *     <li> d<sup>2</sup> / dt<sup>2</sup> x(t) = &omega;<sup>2</sup> c<sub>0</sub> e<sup>&omega; t</sup> + &omega;<sup>2</sup> c<sub>1</sub> e<sup>-&omega; t</sup>
 *     + 6 c<sub>2</sub> t + 2 c<sub>3</sub>  </li>
 *  </p>
 *
 *
 *    <p> From this, it follows that the VRP has the trajectory
 *    <li> v(t) =  c<sub>2</sub> t<sup>3</sup> + c<sub>3</sub> t<sup>2</sup> + (c<sub>4</sub> - 6/&omega;<sup>2</sup> c<sub>2</sub>) t - 2/&omega; c<sub>3</sub> + c<sub>5</sub></li>
 *    </p>
 */
public class OptimizedCoMTrajectoryPlanner implements CoMTrajectoryProvider
{
   private static boolean verbose = false;
   private static final int maxCapacity = 10;
   private static final boolean includeVelocityObjective = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double CONSTRAINT_WEIGHT = 1e7;
   private static final double HIGH_WEIGHT = 1e3;
   public static final double MEDIUM_WEIGHT = 1e1;
   private static final double LOW_WEIGHT = 1e-1;
   private static final double VERY_LOW_WEIGHT = 1e-5;
   private static final double CONSTRAINT_WEIGHT_SQRT = Math.sqrt(CONSTRAINT_WEIGHT);
   private static final double HIGH_WEIGHT_SQRT = Math.sqrt(HIGH_WEIGHT);
   private static final double MEDIUM_WEIGHT_SQRT = Math.sqrt(MEDIUM_WEIGHT);
   private static final double LOW_WEIGHT_SQRT = Math.sqrt(LOW_WEIGHT);
   private static final double VERY_LOW_WEIGHT_SQRT = Math.sqrt(VERY_LOW_WEIGHT);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixSparseCSC coefficientJacobian = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC lowerTriangularHessian = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC hessian = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC xGradient = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC yGradient = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC zGradient = new DMatrixSparseCSC(0, 0);

   private final DMatrixSparseCSC tempSparse = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC xEquivalents = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC yEquivalents = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC zEquivalents = new DMatrixSparseCSC(0, 1);

   private final DMatrixSparseCSC xConstants = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC yConstants = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC zConstants = new DMatrixSparseCSC(0, 1);

   private final DMatrixSparseCSC vrpWaypointJacobian = new DMatrixSparseCSC(0, 1);

   private final DMatrixSparseCSC vrpXWaypoints = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC vrpYWaypoints = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC vrpZWaypoints = new DMatrixSparseCSC(0, 1);

   // FIXME fill reducing?
   private final LinearSolverSparse<DMatrixSparseCSC, DMatrixRMaj> sparseSolver = LinearSolverFactory_DSCC.lu(FillReducing.NONE);

   final DMatrixSparseCSC xCoefficientVector = new DMatrixSparseCSC(0, 1);
   final DMatrixSparseCSC yCoefficientVector = new DMatrixSparseCSC(0, 1);
   final DMatrixSparseCSC zCoefficientVector = new DMatrixSparseCSC(0, 1);

   private final List<CoMTrajectoryPlanningCostPolicy> costPolicies = new ArrayList<>();

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private final CoMTrajectoryPlannerIndexHandler indexHandler = new CoMTrajectoryPlannerIndexHandler();

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredVRPVelocity = new FrameVector3D(worldFrame);
   private final FixedFramePoint3DBasics desiredECMPPosition = new FramePoint3D(worldFrame);

   private final RecyclingArrayList<FramePoint3D> startVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> endVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);

   private final YoBoolean maintainInitialCoMVelocityContinuity = new YoBoolean("maintainInitialCoMVelocityContinuity", registry);
   private final YoFramePoint3D finalDCMPosition = new YoFramePoint3D("goalDCMPosition", worldFrame, registry);

   private final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition", worldFrame, registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);

   private final YoFramePoint3D yoFirstCoefficient = new YoFramePoint3D("comFirstCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoSecondCoefficient = new YoFramePoint3D("comSecondCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoThirdCoefficient = new YoFramePoint3D("comThirdCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoFourthCoefficient = new YoFramePoint3D("comFourthCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoFifthCoefficient = new YoFramePoint3D("comFifthCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoSixthCoefficient = new YoFramePoint3D("comSixthCoefficient", worldFrame, registry);

   private final RecyclingArrayList<FramePoint3D> dcmCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final RecyclingArrayList<Trajectory3D> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   private final RecyclingArrayList<LineSegment3D> vrpSegments = new RecyclingArrayList<>(LineSegment3D::new);
   private final List<Trajectory3D> vrpTrajectories = new ArrayList<>();

   private int numberOfConstraints = 0;

   private CornerPointViewer viewer = null;
   private BagOfBalls comTrajectoryViewer = null;

   public OptimizedCoMTrajectoryPlanner(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      this.omega = omega;

      parentRegistry.addChild(registry);
   }

   public OptimizedCoMTrajectoryPlanner(double gravityZ, YoDouble omega, YoRegistry parentRegistry)
   {
      this.omega = omega;
      this.gravityZ = Math.abs(gravityZ);

      omega.addListener(v -> comHeight.set(gravityZ / MathTools.square(omega.getValue())));
      omega.notifyListeners();

      parentRegistry.addChild(registry);
   }

   public void setMaintainInitialCoMVelocityContinuity(boolean maintainInitialCoMVelocityContinuity)
   {
      this.maintainInitialCoMVelocityContinuity.set(maintainInitialCoMVelocityContinuity);
   }

   public void setCornerPointViewer(CornerPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void setupCoMTrajectoryViewer(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      comTrajectoryViewer = new BagOfBalls(50, 0.01, YoAppearance.Black(), registry, yoGraphicsListRegistry);
   }

   public void addCostPolicy(CoMTrajectoryPlanningCostPolicy costPolicy)
   {
      this.costPolicies.add(costPolicy);
   }

   /** {@inheritDoc} */
   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.comHeight.set(nominalCoMHeight);
   }

   /** {@inheritDoc} */
   @Override
   public double getNominalCoMHeight()
   {
      return comHeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public void solveForTrajectory(List<? extends ContactStateProvider> contactSequence)
   {
      if (!ContactStateProviderTools.checkContactSequenceIsValid(contactSequence))
         throw new IllegalArgumentException("The contact sequence is not valid.");

      indexHandler.update(contactSequence);

      resetMatrices();

      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(), gravityZ, omega.getValue(), currentCoMVelocity, contactSequence, startVRPPositions,
                                                    endVRPPositions, true);

      solveForCoefficientConstraintMatrix(contactSequence);

      // update coefficient holders
      int firstCoefficientIndex = 0;
      int secondCoefficientIndex = 1;
      int thirdCoefficientIndex = 2;
      int fourthCoefficientIndex = 3;
      int fifthCoefficientIndex = 4;
      int sixthCoefficientIndex = 5;

      yoFirstCoefficient.setX(xCoefficientVector.get(firstCoefficientIndex, 0));
      yoFirstCoefficient.setY(yCoefficientVector.get(firstCoefficientIndex, 0));
      yoFirstCoefficient.setZ(zCoefficientVector.get(firstCoefficientIndex, 0));

      yoSecondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex, 0));
      yoSecondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex, 0));
      yoSecondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex, 0));

      yoThirdCoefficient.setX(xCoefficientVector.get(thirdCoefficientIndex, 0));
      yoThirdCoefficient.setY(yCoefficientVector.get(thirdCoefficientIndex, 0));
      yoThirdCoefficient.setZ(zCoefficientVector.get(thirdCoefficientIndex, 0));

      yoFourthCoefficient.setX(xCoefficientVector.get(fourthCoefficientIndex, 0));
      yoFourthCoefficient.setY(yCoefficientVector.get(fourthCoefficientIndex, 0));
      yoFourthCoefficient.setZ(zCoefficientVector.get(fourthCoefficientIndex, 0));

      yoFifthCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex, 0));
      yoFifthCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex, 0));
      yoFifthCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex, 0));

      yoSixthCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex, 0));
      yoSixthCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex, 0));
      yoSixthCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex, 0));

      updateCornerPoints(contactSequence);

      if (viewer != null)
      {
         viewer.updateDCMCornerPoints(dcmCornerPoints);
         viewer.updateCoMCornerPoints(comCornerPoints);
         viewer.updateVRPWaypoints(vrpSegments);
      }
      if (comTrajectoryViewer != null)
      {
         updateCoMTrajectoryViewer();
      }
   }

   private final IGrowArray gw = new IGrowArray();
   private final DGrowArray gx = new DGrowArray();

   private void solveForCoefficientConstraintMatrix(List<? extends ContactStateProvider> contactSequence)
   {
      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      numberOfConstraints = 0;

      // set initial constraint
      addCoMPositionObjective(HIGH_WEIGHT_SQRT, currentCoMPosition);
      if (includeVelocityObjective && maintainInitialCoMVelocityContinuity.getBooleanValue())
         addCoMVelocityObjective(MEDIUM_WEIGHT_SQRT, currentCoMVelocity);
      addDynamicsInitialObjective(HIGH_WEIGHT_SQRT, MEDIUM_WEIGHT_SQRT, contactSequence, 0);

      // add transition continuity constraints
      int transition = 0;
      {
         addCoMPositionContinuityObjective(CONSTRAINT_WEIGHT_SQRT, contactSequence, 0, 1);
         addCoMVelocityContinuityObjective(CONSTRAINT_WEIGHT_SQRT, contactSequence, 0, 1);
         addDynamicsFinalObjective(LOW_WEIGHT_SQRT, VERY_LOW_WEIGHT_SQRT, contactSequence, 0);
         addDynamicsInitialObjective(LOW_WEIGHT_SQRT, VERY_LOW_WEIGHT_SQRT, contactSequence, 1);
      }
      transition++;
      for (; transition < numberOfTransitions; transition++)
      {
         int previousSequence = transition;
         int nextSequence = transition + 1;
         addCoMPositionContinuityObjective(CONSTRAINT_WEIGHT_SQRT, contactSequence, previousSequence, nextSequence);
         addCoMVelocityContinuityObjective(CONSTRAINT_WEIGHT_SQRT, contactSequence, previousSequence, nextSequence);
         addDynamicsFinalObjective(MEDIUM_WEIGHT_SQRT, LOW_WEIGHT_SQRT, contactSequence, previousSequence);
         addDynamicsInitialObjective(MEDIUM_WEIGHT_SQRT, LOW_WEIGHT_SQRT, contactSequence, nextSequence);
      }

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequence.get(numberOfPhases - 1);
      finalDCMPosition.set(endVRPPositions.getLast());
      double finalDuration = Math.min(lastContactPhase.getTimeInterval().getDuration(), CoMTrajectoryPlannerTools.sufficientlyLongTime);
      addDCMPositionObjective(HIGH_WEIGHT_SQRT, numberOfPhases - 1, finalDuration, finalDCMPosition);
      addDynamicsFinalObjective(HIGH_WEIGHT_SQRT, MEDIUM_WEIGHT_SQRT, contactSequence, numberOfPhases - 1);


      // TODO make an add equals method. Also don't pass in null, as that apparently makes garbage.
      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpXWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, xConstants, xEquivalents, gw, gx);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpYWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, yConstants, yEquivalents, gw, gx);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpZWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, zConstants, zEquivalents, gw, gx);

      // We have J x - b, we need to formulate that into the QP
      CommonOps_DSCC.innerProductLower(coefficientJacobian, lowerTriangularHessian, gw, gx);
      CommonOps_DSCC.symmLowerToFull(lowerTriangularHessian, hessian, gw );

      CommonOps_DSCC.multTransA(coefficientJacobian, xEquivalents, xGradient, gw, gx);
      CommonOps_DSCC.multTransA(coefficientJacobian, yEquivalents, yGradient, gw, gx);
      CommonOps_DSCC.multTransA(coefficientJacobian, zEquivalents, zGradient, gw, gx);

      CommonOps_DSCC.scale(2.0, xGradient, xGradient);
      CommonOps_DSCC.scale(2.0, yGradient, yGradient);
      CommonOps_DSCC.scale(2.0, zGradient, zGradient);

      for (int segment = 0; segment < numberOfPhases; segment++)
      {
         double duration = contactSequence.get(segment).getTimeInterval().getDuration();
         CoMTrajectoryPlannerTools.addMinimizeCoMAccelerationObjective(LOW_WEIGHT, 0.0, duration, omega.getValue(), segment, hessian);
         CoMTrajectoryPlannerTools.addMinimizeCoMJerkObjective(VERY_LOW_WEIGHT, 0.0, duration, omega.getValue(), segment, hessian);
      }

      for (int i = 0; i < costPolicies.size(); i++)
         costPolicies.get(i).assessPolicy(this, contactSequence, hessian, xGradient, yGradient, zGradient);

      CommonOps_DSCC.scale(0.5, xGradient, xGradient);
      CommonOps_DSCC.scale(0.5, yGradient, yGradient);
      CommonOps_DSCC.scale(0.5, zGradient, zGradient);

      sparseSolver.setA(hessian);
      sparseSolver.solveSparse(xGradient, xCoefficientVector);
      sparseSolver.solveSparse(yGradient, yCoefficientVector);
      sparseSolver.solveSparse(zGradient, zCoefficientVector);
   }

   private final FramePoint3D comPositionToThrowAway = new FramePoint3D();
   private final FramePoint3D dcmPositionToThrowAway = new FramePoint3D();

   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D vrpVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FrameVector3D vrpStartVelocity = new FrameVector3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FrameVector3D vrpEndVelocity = new FrameVector3D();
   private final FramePoint3D ecmpPositionToThrowAway = new FramePoint3D();

   private void updateCornerPoints(List<? extends ContactStateProvider> contactSequence)
   {
      vrpTrajectoryPool.clear();
      vrpTrajectories.clear();

      comCornerPoints.clear();
      dcmCornerPoints.clear();
      vrpSegments.clear();

      boolean verboseBefore = verbose;
      verbose = false;
      for (int segmentId = 0; segmentId < Math.min(contactSequence.size(), maxCapacity + 1); segmentId++)
      {
         double duration = contactSequence.get(segmentId).getTimeInterval().getDuration();

         duration = Math.min(duration, CoMTrajectoryPlannerTools.sufficientlyLongTime);
         compute(segmentId, 0.0, comCornerPoints.add(), comVelocityToThrowAway, comAccelerationToThrowAway, dcmCornerPoints.add(),
                 dcmVelocityToThrowAway, vrpStartPosition, vrpStartVelocity, ecmpPositionToThrowAway);
         compute(segmentId, duration, comPositionToThrowAway, comVelocityToThrowAway, comAccelerationToThrowAway, dcmPositionToThrowAway,
                 dcmVelocityToThrowAway, vrpEndPosition, vrpEndVelocity, ecmpPositionToThrowAway);

         Trajectory3D trajectory3D = vrpTrajectoryPool.add();
         trajectory3D.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
//         trajectory3D.setLinear(0.0, duration, vrpStartPosition, vrpEndPosition);
         vrpTrajectories.add(trajectory3D);

         vrpSegments.add().set(vrpStartPosition, vrpEndPosition);
      }

      verbose = verboseBefore;
   }

   private void updateCoMTrajectoryViewer()
   {
      comTrajectoryViewer.reset();

      boolean verboseBefore = verbose;
      verbose = false;
      for (int i = 0; i < comTrajectoryViewer.getNumberOfBalls(); i++)
      {
         double time = 0.05 * i;
         int segmentId = getSegmentNumber(time);
         double timeInSegment = getTimeInSegment(segmentId, time);

         compute(segmentId, timeInSegment, comPositionToThrowAway, comVelocityToThrowAway, comAccelerationToThrowAway, dcmPositionToThrowAway,
                 dcmVelocityToThrowAway, vrpStartPosition, ecmpPositionToThrowAway);

         comTrajectoryViewer.setBall(comPositionToThrowAway);
      }

      verbose = verboseBefore;
   }

   /** {@inheritDoc} */
   @Override
   public void compute(int segmentId, double timeInPhase)
   {
      compute(segmentId, timeInPhase, desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration, desiredDCMPosition, desiredDCMVelocity,
              desiredVRPPosition, desiredECMPPosition);

      if (verbose)
      {
         LogTools.info("At time " + timeInPhase + ", Desired DCM = " + desiredDCMPosition + ", Desired CoM = " + desiredCoMPosition);
      }
   }

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();
   private final FramePoint3D fifthCoefficient = new FramePoint3D();
   private final FramePoint3D sixthCoefficient = new FramePoint3D();

   @Override
   public void compute(int segmentId, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack, FixedFramePoint3DBasics ecmpPositionToPack)
   {
      compute(segmentId, timeInPhase, comPositionToPack, comVelocityToPack, comAccelerationToPack, dcmPositionToPack, dcmVelocityToPack,
              vrpPositionToPack, vrpVelocityToThrowAway, ecmpPositionToPack);
   }

   public void compute(int segmentId, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack, FixedFrameVector3DBasics vrpVelocityToPack, FixedFramePoint3DBasics ecmpPositionToPack)
   {
      if (segmentId < 0)
         throw new IllegalArgumentException("time is invalid.");

      int startIndex = indexHandler.getContactSequenceStartIndex(segmentId);
      firstCoefficient.setX(xCoefficientVector.get(startIndex, 0));
      firstCoefficient.setY(yCoefficientVector.get(startIndex, 0));
      firstCoefficient.setZ(zCoefficientVector.get(startIndex, 0));

      int secondCoefficientIndex = startIndex + 1;
      secondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex, 0));

      int thirdCoefficientIndex = startIndex + 2;
      thirdCoefficient.setX(xCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setY(yCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setZ(zCoefficientVector.get(thirdCoefficientIndex, 0));

      int fourthCoefficientIndex = startIndex + 3;
      fourthCoefficient.setX(xCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setY(yCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setZ(zCoefficientVector.get(fourthCoefficientIndex, 0));

      int fifthCoefficientIndex = startIndex + 4;
      fifthCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex, 0));

      int sixthCoefficientIndex = startIndex + 5;
      sixthCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex, 0));

      double omega = this.omega.getValue();

      CoMTrajectoryPlannerTools.constructDesiredCoMPosition(comPositionToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                  sixthCoefficient, timeInPhase, omega);
      CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(comVelocityToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                  sixthCoefficient, timeInPhase, omega);
      CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(comAccelerationToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                      sixthCoefficient, timeInPhase, omega);

      CoMTrajectoryPlannerTools.constructDesiredVRPVelocity(vrpVelocityToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                                            sixthCoefficient, timeInPhase, omega);

      CapturePointTools.computeCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPositionToPack);
      CapturePointTools.computeCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      CapturePointTools.computeCentroidalMomentumPivot(dcmPositionToPack, dcmVelocityToPack, omega, vrpPositionToPack);

      ecmpPositionToPack.set(vrpPositionToPack);
      ecmpPositionToPack.subZ(comHeight.getDoubleValue());
   }

   /** {@inheritDoc} */
   @Override
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.currentCoMPosition.setMatchingFrame(centerOfMassPosition);
      this.currentCoMVelocity.setMatchingFrame(centerOfMassVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   public FrameVector3DReadOnly getDesiredVRPVelocity()
   {
      return desiredVRPVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
   }

   /**
    * Resets and resizes the internal matrices.
    */
   private void resetMatrices()
   {
      int size = indexHandler.getTotalNumberOfCoefficients();
      int numberOfVRPWaypoints = indexHandler.getNumberOfVRPWaypoints();

      int extraObjectives = includeVelocityObjective && maintainInitialCoMVelocityContinuity.getBooleanValue() ? 1 : 0;
      coefficientJacobian.reshape(size + extraObjectives, size);
      hessian.reshape(size, size);
      lowerTriangularHessian.reshape(size, size);
      xGradient.reshape(size, 1);
      yGradient.reshape(size, 1);
      zGradient.reshape(size, 1);
      tempSparse.reshape(size, 1);
      xEquivalents.reshape(size + extraObjectives, 1);
      yEquivalents.reshape(size + extraObjectives, 1);
      zEquivalents.reshape(size + extraObjectives, 1);
      xConstants.reshape(size + extraObjectives, 1);
      yConstants.reshape(size + extraObjectives, 1);
      zConstants.reshape(size + extraObjectives, 1);
      vrpWaypointJacobian.reshape(size + extraObjectives, numberOfVRPWaypoints); // only position
      vrpXWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpYWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpZWaypoints.reshape(numberOfVRPWaypoints, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      coefficientJacobian.zero();
      lowerTriangularHessian.zero();
      hessian.zero();
      xEquivalents.zero();
      yEquivalents.zero();
      zEquivalents.zero();
      xConstants.zero();
      yConstants.zero();
      zConstants.zero();
      vrpWaypointJacobian.zero();
      vrpXWaypoints.zero();
      vrpYWaypoints.zero();
      vrpZWaypoints.zero();
   }

   /**
    * <p> Sets the continuity constraint on the initial CoM position. This DOES result in a initial discontinuity on the desired DCM location,
    * coming from a discontinuity on the desired CoM Velocity. </p>
    * <p> This constraint should be used for the initial position of the center of mass to properly initialize the trajectory. </p>
    * <p> Recall that the equation for the center of mass is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p>
    *    This constraint defines
    * </p>
    * <p>
    *    x<sub>0</sub>(0) = x<sub>d</sub>,
    * </p>
    * <p>
    *    substituting in the coefficients into the constraint matrix.
    * </p>
    * @param centerOfMassLocationForConstraint x<sub>d</sub> in the above equations
    */
   private void addCoMPositionObjective(double weight, FramePoint3DReadOnly centerOfMassLocationForConstraint)
   {
      CoMTrajectoryPlannerTools.addCoMPositionObjective(weight, centerOfMassLocationForConstraint, omega.getValue(), 0.0, 0, numberOfConstraints,
                                                        coefficientJacobian, xConstants, yConstants, zConstants);
      numberOfConstraints++;
   }


   private void addCoMVelocityObjective(double weight, FrameVector3DReadOnly desiredCoMVelocity)
   {
      CoMTrajectoryPlannerTools.addCoMVelocityObjective(weight, desiredCoMVelocity, omega.getValue(), 0.0, 0, numberOfConstraints,
                                                        coefficientJacobian, xConstants, yConstants, zConstants);
      numberOfConstraints++;
   }

   /**
    * <p> Sets a constraint on the desired DCM position. This constraint is useful for constraining the terminal location of the DCM trajectory. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> and the center of mass velocity is defined by </p>
    * <p>
    *    d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    *    &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    *     2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>
    * </p>
    * <p>
    *    This constraint is then combining these two, saying
    * </p>
    * <p> x<sub>i</sub>(t<sub>i</sub>) + 1 / &omega; d/dt x<sub>i</sub>(t<sub>i</sub>) = &xi;<sub>d</sub>,</p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId i in the above equations
    * @param time t<sub>i</sub> in the above equations
    * @param desiredDCMPosition desired DCM location. &xi;<sub>d</sub> in the above equations.
    */
   private void addDCMPositionObjective(double weight, int sequenceId, double time, FramePoint3DReadOnly desiredDCMPosition)
   {
      CoMTrajectoryPlannerTools.addDCMPositionObjective(weight, sequenceId, numberOfConstraints, time, omega.getValue(), desiredDCMPosition, coefficientJacobian,
                                                         xConstants, yConstants, zConstants);
      numberOfConstraints++;
   }

   /**
    * <p> Set a continuity constraint on the CoM position at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> x<sub>i-1</sub>(T<sub>i-1</sub>) = x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param contactSequence current contact sequence.
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void addCoMPositionContinuityObjective(double weight,
                                                  List<? extends ContactStateProvider> contactSequence,
                                                  int previousSequence,
                                                  int nextSequence)
   {
      double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
      CoMTrajectoryPlannerTools.addCoMPositionContinuityObjective(weight, previousSequence, nextSequence, numberOfConstraints, omega.getValue(), previousDuration,
                                                                   coefficientJacobian);

      numberOfConstraints++;
   }

   /**
    * <p> Set a continuity constraint on the CoM velocity at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    *    &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    *     2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> d / dt x<sub>i-1</sub>(T<sub>i-1</sub>) = d / dt x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param contactSequence current contact sequence.
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void addCoMVelocityContinuityObjective(double weight,
                                                  List<? extends ContactStateProvider> contactSequence,
                                                  int previousSequence,
                                                  int nextSequence)
   {
      double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
      CoMTrajectoryPlannerTools.addCoMVelocityContinuityObjective(weight, previousSequence, nextSequence, numberOfConstraints, omega.getValue(),
                                                                  previousDuration, coefficientJacobian);
      numberOfConstraints++;
   }

   private final FrameVector3D desiredVelocity = new FrameVector3D();

   /**
    * Used to enforce the dynamics at the beginning of the trajectory segment {@param sequenceId}.
    *
    * @param contactSequence current contact sequence.
    * @param sequenceId desired trajectory segment.
    */
   private void addDynamicsInitialObjective(double positionWeight, double velocityWeight, List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      if (contactState.isLoadBearing())
      {
         double duration = contactStateProvider.getTimeInterval().getDuration();
         desiredVelocity.sub(endVRPPositions.get(sequenceId), startVRPPositions.get(sequenceId));
         desiredVelocity.scale(1.0 / duration);
         addVRPPositionObjective(positionWeight, sequenceId, indexHandler.getVRPWaypointStartPositionIndex(sequenceId), 0.0, startVRPPositions.get(sequenceId));
         addVRPVelocityObjective(velocityWeight, sequenceId, indexHandler.getVRPWaypointStartVelocityIndex(sequenceId), 0.0, desiredVelocity);
      }
      else
      {
         addCoMAccelerationIsGravityObjective(CONSTRAINT_WEIGHT_SQRT, sequenceId, 0.0);
         addZeroCoMJerkObjective(CONSTRAINT_WEIGHT_SQRT, sequenceId, 0.0);
      }
   }

   /**
    * Used to enforce the dynamics at the end of the trajectory segment {@param sequenceId}.
    *
    * @param contactSequence current contact sequence.
    * @param sequenceId desired trajectory segment.
    */
   private void addDynamicsFinalObjective(double positionWeight, double velocityWeight, List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      double duration = contactStateProvider.getTimeInterval().getDuration();
      if (contactState.isLoadBearing())
      {
         desiredVelocity.sub(endVRPPositions.get(sequenceId), startVRPPositions.get(sequenceId));
         desiredVelocity.scale(1.0 / duration);
         addVRPPositionObjective(positionWeight, sequenceId, indexHandler.getVRPWaypointFinalPositionIndex(sequenceId), duration, endVRPPositions.get(sequenceId));
         addVRPVelocityObjective(velocityWeight, sequenceId, indexHandler.getVRPWaypointFinalVelocityIndex(sequenceId), duration, desiredVelocity);
      }
      else
      {
         addCoMAccelerationIsGravityObjective(CONSTRAINT_WEIGHT_SQRT, sequenceId, duration);
         addZeroCoMJerkObjective(CONSTRAINT_WEIGHT_SQRT, sequenceId, duration);
      }
   }

   /**
    * <p> Adds a constraint for the desired VRP position.</p>
    * <p> Recall that the VRP is defined as </p>
    * <p> v<sub>i</sub>(t<sub>i</sub>) =  c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * (c<sub>4,i</sub> - 6/&omega;<sup>2</sup> c<sub>2,i</sub>) t<sub>i</sub> - 2/&omega; c<sub>3,i</sub> + c<sub>5,i</sub></p>.
    * <p> This constraint then says </p>
    * <p> v<sub>i</sub>(t<sub>i</sub>) = J v<sub>d</sub> </p>
    * <p> where J is a Jacobian that maps from a vector of desired VRP waypoints to the constraint form, and </p>
    * <p> v<sub>d,j</sub> = v<sub>r</sub> </p>
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointPositionIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPPosition reference VRP position, v<sub>r</sub> in the above equations.
    */
   private void addVRPPositionObjective(double weight,
                                        int sequenceId,
                                        int vrpWaypointPositionIndex,
                                        double time,
                                        FramePoint3DReadOnly desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addVRPPositionObjective(weight, sequenceId, numberOfConstraints, vrpWaypointPositionIndex, time, omega.getValue(),
                                                        desiredVRPPosition, coefficientJacobian, vrpXWaypoints, vrpYWaypoints, vrpZWaypoints, vrpWaypointJacobian);
      numberOfConstraints++;
   }

   /**
    * <p> Adds a constraint for the desired VRP velocity.</p>
    * <p> Recall that the VRP velocity is defined as </p>
    * <p> d/dt v<sub>i</sub>(t<sub>i</sub>) =  3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> + 2 c<sub>3,i</sub> t<sub>i</sub> +
    * (c<sub>4,i</sub> - 6/&omega;<sup>2</sup> c<sub>2,i</sub>).
    * <p> This constraint then says </p>
    * <p> d/dt v<sub>i</sub>(t<sub>i</sub>) = J v<sub>d</sub> </p>
    * <p> where J is a Jacobian that maps from a vector of desired VRP waypoints to the constraint form, and </p>
    * <p> v<sub>d,j</sub> = d/dt v<sub>r</sub> </p>
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointVelocityIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPVelocity reference VRP veloctiy, d/dt v<sub>r</sub> in the above equations.
    */
   private void addVRPVelocityObjective(double weight,
                                        int sequenceId,
                                        int vrpWaypointVelocityIndex,
                                        double time,
                                        FrameVector3DReadOnly desiredVRPVelocity)
   {
      CoMTrajectoryPlannerTools.addVRPVelocityObjective(weight, sequenceId, numberOfConstraints, vrpWaypointVelocityIndex, omega.getValue(), time,
                                                        desiredVRPVelocity, coefficientJacobian, vrpXWaypoints, vrpYWaypoints, vrpZWaypoints, vrpWaypointJacobian);
      numberOfConstraints++;
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have an acceleration equal to gravity at time t.</p>
    * <p> Recall that the CoM acceleration is defined as </p>
    * d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>2</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> +
    * &omega;<sup>2</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub> t<sub>i</sub> + 2 c<sub>3,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = -g, </p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   private void addCoMAccelerationIsGravityObjective(double weight, int sequenceId, double time)
   {
      CoMTrajectoryPlannerTools.addCoMAccelerationIsGravityObjective(weight, sequenceId, numberOfConstraints, omega.getValue(), time, gravityZ, coefficientJacobian,
                                                                  zConstants);
      numberOfConstraints++;
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have a jerk equal to 0.0 at time t.</p>
    * <p> Recall that the CoM jerk is defined as </p>
    * d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>3</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega;<sup>3</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = 0.0, </p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   private void addZeroCoMJerkObjective(double weight, int sequenceId, double time)
   {
      CoMTrajectoryPlannerTools.addZeroCoMJerkObjective(weight, time, omega.getValue(), sequenceId, numberOfConstraints, coefficientJacobian);
      numberOfConstraints++;
   }

   @Override
   public List<Trajectory3D> getVRPTrajectories()
   {
      return vrpTrajectories;
   }
}
