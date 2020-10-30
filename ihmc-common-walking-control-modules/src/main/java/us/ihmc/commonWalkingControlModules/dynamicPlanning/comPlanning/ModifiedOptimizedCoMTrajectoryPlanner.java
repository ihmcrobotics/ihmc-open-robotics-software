package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
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
public class ModifiedOptimizedCoMTrajectoryPlanner implements CoMTrajectoryProvider
{
   private static boolean verbose = false;
   private static final int maxCapacity = 10;
   private static final boolean includeVelocityObjective = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final double MEDIUM_WEIGHT = 1e1;
   public static final double LOW_WEIGHT = 1e-2;

   private double jerkMinimizationWeight = 0.0;//1e-8;
   private double accelerationMinimizationWeight = 0.0;//1e-6;


   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixSparseCSC hessian = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC xGradient = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC yGradient = new DMatrixSparseCSC(0, 0);
   private final DMatrixSparseCSC zGradient = new DMatrixSparseCSC(0, 0);

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

   private CornerPointViewer viewer = null;
   private BagOfBalls comTrajectoryViewer = null;

   public ModifiedOptimizedCoMTrajectoryPlanner(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      this.omega = omega;

      parentRegistry.addChild(registry);
   }

   public ModifiedOptimizedCoMTrajectoryPlanner(double gravityZ, YoDouble omega, YoRegistry parentRegistry)
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

   public void setAccelerationMinimizationWeight(double accelerationMinimizationWeight)
   {
      this.accelerationMinimizationWeight = accelerationMinimizationWeight;
   }

   public void setJerkMinimizationWeight(double jerkMinimizationWeight)
   {
      this.jerkMinimizationWeight = jerkMinimizationWeight;
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

   private void solveForCoefficientConstraintMatrix(List<? extends ContactStateProvider> contactSequence)
   {
      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      // set initial constraint
      CoMTrajectoryPlannerTools.addCoMPositionObjective(MEDIUM_WEIGHT, currentCoMPosition, omega.getValue(), 0.0, 0,
                                                        hessian, xGradient, yGradient, zGradient);
      if (includeVelocityObjective && maintainInitialCoMVelocityContinuity.getBooleanValue())
      {
         CoMTrajectoryPlannerTools.addCoMVelocityObjective(MEDIUM_WEIGHT, currentCoMVelocity, omega.getValue(), 0.0, 0,
                                                           hessian, xGradient, yGradient, zGradient);
      }
      addDynamicsInitialObjective(MEDIUM_WEIGHT, MEDIUM_WEIGHT, contactSequence, 0);

      // add transition continuity constraints
      int transition = 0;
      if (numberOfTransitions > 0)
      {
         double previousDuration = contactSequence.get(0).getTimeInterval().getDuration();
         CoMTrajectoryPlannerTools.addCoMPositionContinuityObjective(MEDIUM_WEIGHT, 0, 1, omega.getValue(), previousDuration, hessian);
         CoMTrajectoryPlannerTools.addCoMVelocityContinuityObjective(MEDIUM_WEIGHT, 0, 1, omega.getValue(), previousDuration, hessian);
         addDynamicsFinalObjective(MEDIUM_WEIGHT, MEDIUM_WEIGHT, contactSequence, 0);
         addDynamicsInitialObjective(MEDIUM_WEIGHT, MEDIUM_WEIGHT, contactSequence, 1);
      }
      transition++;
      for (; transition < numberOfTransitions; transition++)
      {
         int previousSequence = transition;
         int nextSequence = transition + 1;
         double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
         CoMTrajectoryPlannerTools.addCoMPositionContinuityObjective(MEDIUM_WEIGHT, previousSequence, nextSequence, omega.getValue(), previousDuration, hessian);
         CoMTrajectoryPlannerTools.addCoMVelocityContinuityObjective(MEDIUM_WEIGHT, previousSequence, nextSequence, omega.getValue(), previousDuration, hessian);
         addDynamicsFinalObjective(MEDIUM_WEIGHT, MEDIUM_WEIGHT, contactSequence, previousSequence);
         addDynamicsInitialObjective(MEDIUM_WEIGHT, MEDIUM_WEIGHT, contactSequence, nextSequence);
      }

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequence.get(numberOfPhases - 1);
      finalDCMPosition.set(endVRPPositions.getLast());
      double finalDuration = Math.min(lastContactPhase.getTimeInterval().getDuration(), CoMTrajectoryPlannerTools.sufficientlyLongTime);
      CoMTrajectoryPlannerTools.addDCMPositionObjective(MEDIUM_WEIGHT, finalDCMPosition, omega.getValue(), finalDuration, numberOfPhases - 1, hessian, xGradient, yGradient, zGradient);
      addDynamicsFinalObjective(MEDIUM_WEIGHT, MEDIUM_WEIGHT, contactSequence, numberOfPhases - 1);


      for (int segment = 0; segment < numberOfPhases; segment++)
      {
         ContactStateProvider contactStateProvider = contactSequence.get(segment);
         if (contactStateProvider.getContactState().isLoadBearing())
         {
            double duration = contactStateProvider.getTimeInterval().getDuration();
            CoMTrajectoryPlannerTools.addMinimizeCoMAccelerationObjective(accelerationMinimizationWeight, 0.0, duration, omega.getValue(), segment, hessian);
            CoMTrajectoryPlannerTools.addMinimizeCoMJerkObjective(jerkMinimizationWeight, 0.0, duration, omega.getValue(), segment, hessian);
         }
      }
//
      for (int i = 0; i < costPolicies.size(); i++)
         costPolicies.get(i).assessPolicy(this, contactSequence, hessian, xGradient, yGradient, zGradient);

      CommonOps_DSCC.scale(-0.5, xGradient, xGradient);
      CommonOps_DSCC.scale(-0.5, yGradient, yGradient);
      CommonOps_DSCC.scale(-0.5, zGradient, zGradient);

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

      hessian.reshape(size, size);
      xGradient.reshape(size, 1);
      yGradient.reshape(size, 1);
      zGradient.reshape(size, 1);
      vrpXWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpYWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpZWaypoints.reshape(numberOfVRPWaypoints, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      hessian.zero();
      vrpXWaypoints.zero();
      vrpYWaypoints.zero();
      vrpZWaypoints.zero();
   }


   private final FrameVector3D desiredVelocity = new FrameVector3D();

   private void addDynamicsInitialObjective(double positionWeight, double velocityWeight, List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      if (contactState.isLoadBearing())
      {
         addContactDynamicsObjective(positionWeight, velocityWeight, 0.0, sequenceId, startVRPPositions.get(sequenceId));
      }
      else
      {
         addFlightDynamicsObjective(0.0, sequenceId);
      }
   }

   private void addDynamicsFinalObjective(double positionWeight, double velocityWeight, List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      double duration = contactStateProvider.getTimeInterval().getDuration();

      if (contactState.isLoadBearing())
      {
         addContactDynamicsObjective(positionWeight, velocityWeight, duration, sequenceId, endVRPPositions.get(sequenceId));
      }
      else
      {
         addFlightDynamicsObjective(duration, sequenceId);
      }
   }

   private void addContactDynamicsObjective(double positionWeight, double velocityWeight, double time, int sequenceId, FramePoint3D desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addVRPPositionObjective(positionWeight, desiredVRPPosition, omega.getValue(), time, sequenceId,
                                                        hessian, xGradient, yGradient, zGradient);
      CoMTrajectoryPlannerTools.addVRPVelocityObjective(velocityWeight, desiredVelocity, omega.getValue(), time, sequenceId,
                                                        hessian, xGradient, yGradient, zGradient);
   }

   private void addFlightDynamicsObjective(double time, int sequenceId)
   {
      CoMTrajectoryPlannerTools.addCoMAccelerationIsGravityObjective(MEDIUM_WEIGHT, sequenceId, omega.getValue(), time, -gravityZ, hessian, zGradient);
      CoMTrajectoryPlannerTools.addCoMJerkObjective(MEDIUM_WEIGHT, null, omega.getValue(), time, sequenceId, hessian, null, null, null);
   }

   @Override
   public List<Trajectory3D> getVRPTrajectories()
   {
      return vrpTrajectories;
   }
}
