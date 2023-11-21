package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import com.esotericsoftware.minlog.Log;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.interfaces.linsol.LinearSolverSparse;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.CommonOps_DSCC;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;
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
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

/**
 * <p>
 * This is the main class of the trajectory-based CoM Trajectory Planner.
 * </p>
 * <p>
 * This class assumes that the final phase is always the "stopping" phase, where the CoM is supposed to come to rest.
 * This means that the final VRP is the terminal DCM location
 * </p>
 * <p>
 * The CoM has the following definitions:
 * <li>      x(t) = c<sub>0</sub> e<sup>&omega; t</sup> + c<sub>1</sub> e<sup>-&omega; t</sup> + c<sub>2</sub> t<sup>3</sup> + c<sub>3</sub> t<sup>2</sup> +
 * c<sub>4</sub> t + c<sub>5</sub></li>
 * <li> d/dt x(t) = &omega; c<sub>0</sub> e<sup>&omega; t</sup> - &omega; c<sub>1</sub> e<sup>-&omega; t</sup> + 3 c<sub>2</sub> t<sup>2</sup> +
 * 2 c<sub>3</sub> t+ c<sub>4</sub>
 * <li> d<sup>2</sup> / dt<sup>2</sup> x(t) = &omega;<sup>2</sup> c<sub>0</sub> e<sup>&omega; t</sup> + &omega;<sup>2</sup> c<sub>1</sub> e<sup>-&omega;
 * t</sup>
 * + 6 c<sub>2</sub> t + 2 c<sub>3</sub>  </li>
 * </p>
 *
 *
 * <p> From this, it follows that the VRP has the trajectory
 * <li> v(t) =  c<sub>2</sub> t<sup>3</sup> + c<sub>3</sub> t<sup>2</sup> + (c<sub>4</sub> - 6/&omega;<sup>2</sup> c<sub>2</sub>) t - 2/&omega; c<sub>3</sub> +
 * c<sub>5</sub></li>
 * </p>
 */
public class CoMTrajectoryPlanner implements CoMTrajectoryProvider
{
   private static boolean verbose = false;
   private static final int maxCapacity = 10;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixSparseCSC coefficientMultipliersSparse = new DMatrixSparseCSC(0, 0);

   private final DMatrixRMaj xEquivalents = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj yEquivalents = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj zEquivalents = new DMatrixRMaj(0, 1);

   private final DMatrixRMaj xConstants = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj yConstants = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj zConstants = new DMatrixRMaj(0, 1);

   private final DMatrixSparseCSC vrpWaypointJacobian = new DMatrixSparseCSC(0, 1);

   private final DMatrixRMaj vrpXWaypoints = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj vrpYWaypoints = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj vrpZWaypoints = new DMatrixRMaj(0, 1);

   private final LinearSolverSparse<DMatrixSparseCSC, DMatrixRMaj> sparseSolver = LinearSolverFactory_DSCC.lu(FillReducing.NONE);

   final DMatrixRMaj xCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj yCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj zCoefficientVector = new DMatrixRMaj(0, 1);

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private final CoMTrajectoryPlannerIndexHandler indexHandler = new CoMTrajectoryPlannerIndexHandler();
   private final LinearCoMTrajectoryHandler trajectoryHandler = new LinearCoMTrajectoryHandler(registry);

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
   private final RecyclingArrayList<FramePoint3D> modifiedStartVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> modifiedEndVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FrameVector3D> modifiedStartVRPVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final RecyclingArrayList<FrameVector3D> modifiedEndVRPVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final RecyclingArrayList<FrameVector3D> externalForceStart = new RecyclingArrayList<>(FrameVector3D::new);
   private final RecyclingArrayList<FrameVector3D> externalForceEnd = new RecyclingArrayList<>(FrameVector3D::new);


   private final YoFramePoint3D finalDCMPosition = new YoFramePoint3D("goalDCMPosition", worldFrame, registry);

   private final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition", worldFrame, registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);

   private final RecyclingArrayList<FramePoint3D> dcmCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final RecyclingArrayList<LineSegment3D> vrpSegments = new RecyclingArrayList<>(LineSegment3D::new);

   private int numberOfConstraints = 0;
   private final YoBoolean maintainInitialCoMVelocityContinuity = new YoBoolean("maintainInitialComVelocityContinuity", registry);

   private CornerPointViewer viewer = null;
   private BagOfBalls comTrajectoryViewer = null;

   private CoMContinuityCalculator comContinuityCalculator = null;

   public CoMTrajectoryPlanner(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      this.omega = omega;
      maintainInitialCoMVelocityContinuity.set(false);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public CoMTrajectoryPlanner(double gravityZ, YoDouble omega, YoRegistry parentRegistry)
   {
      this.omega = omega;
      this.gravityZ = Math.abs(gravityZ);

      omega.addListener(v -> comHeight.set(this.gravityZ / MathTools.square(omega.getValue())));
      omega.notifyListeners();

      maintainInitialCoMVelocityContinuity.set(false);

      if (parentRegistry != null)
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

   public void setComContinuityCalculator(CoMContinuityCalculator comContinuityCalculator)
   {
      this.comContinuityCalculator = comContinuityCalculator;
   }

   public void reset()
   {
      trajectoryHandler.clearTrajectory();
   }

   public void initializeTrajectory(FramePoint3DReadOnly endPosition, double stepDuration)
   {
      stepDuration = Math.min(sufficientlyLongTime, stepDuration);
      trajectoryHandler.setLinear(currentCoMPosition, endPosition, omega.getValue(), stepDuration);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.comHeight.set(nominalCoMHeight);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getNominalCoMHeight()
   {
      return comHeight.getDoubleValue();
   }

   public double getOmega()
   {
      return omega.getValue();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void solveForTrajectory(List<? extends ContactStateProvider> contactSequence)
   {
      if (!ContactStateProviderTools.checkContactSequenceIsValid(contactSequence))
         throw new IllegalArgumentException("The contact sequence is not valid.");

      indexHandler.update(contactSequence);

      resetMatrices();

      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                                    gravityZ,
                                                    omega.getValue(),
                                                    currentCoMVelocity,
                                                    contactSequence,
                                                    startVRPPositions,
                                                    endVRPPositions,
                                                    true);
      CoMTrajectoryPlannerTools.offsetVRPWaypointsWithExternalForces(omega.getValue(),
                                                                     startVRPPositions,
                                                                     endVRPPositions,
                                                                     contactSequence,
                                                                     modifiedStartVRPPositions,
                                                                     modifiedEndVRPPositions,
                                                                     modifiedStartVRPVelocities,
                                                                     modifiedEndVRPVelocities);

      externalForceStart.clear();
      externalForceEnd.clear();
      for (int i = 0; i < contactSequence.size(); i++)
      {
         externalForceStart.add().set(contactSequence.get(i).getExternalContactAccelerationStart());
         externalForceEnd.add().set(contactSequence.get(i).getExternalContactAccelerationEnd());
      }

      solveForCoefficientConstraintMatrix(contactSequence);
      trajectoryHandler.setCoefficientsFromSolution(omega.getValue(), contactSequence, xCoefficientVector, yCoefficientVector, zCoefficientVector);

      if (false && maintainInitialCoMVelocityContinuity.getBooleanValue() && comContinuityCalculator != null && contactSequence.size() > 1)
      {
         int segmentId = comContinuityCalculator.getDepthForCalculation() - 1;

         if (maintainInitialCoMVelocityContinuity.getBooleanValue() && contactSequence.size() > segmentId)
         {
            double time = contactSequence.get(segmentId).getTimeInterval().getDuration();
            compute(segmentId,
                    time,
                    comPositionToThrowAway,
                    comVelocityToThrowAway,
                    comAccelerationToThrowAway,
                    dcmPositionToThrowAway,
                    dcmVelocityToThrowAway,
                    vrpStartPosition,
                    ecmpPositionToThrowAway);

            comContinuityCalculator.setInitialCoMPosition(currentCoMPosition);
            comContinuityCalculator.setInitialCoMVelocity(currentCoMVelocity);
            comContinuityCalculator.setFinalICPToAchieve(dcmPositionToThrowAway);

            if (comContinuityCalculator.solve(contactSequence))
            {
               comContinuityCalculator.getXCoefficientOverrides(xCoefficientVector);
               comContinuityCalculator.getYCoefficientOverrides(yCoefficientVector);
               comContinuityCalculator.getZCoefficientOverrides(zCoefficientVector);
            }
         }
      }

      trajectoryHandler.setCoefficientsFromSolution(omega.getValue(), contactSequence, xCoefficientVector, yCoefficientVector, zCoefficientVector);

      if (viewer != null)
      {
         updateCornerPoints(contactSequence);

         viewer.updateDCMCornerPoints(dcmCornerPoints);
         viewer.updateCoMCornerPoints(comCornerPoints);
         viewer.updateVRPWaypoints(vrpSegments);
      }
      if (comTrajectoryViewer != null)
      {
         updateCoMTrajectoryViewer();
      }
   }

   private boolean hasExternalForceObjective()
   {
      for (int i = 0; i < externalForceEnd.size(); i++)
      {
         if (externalForceEnd.get(i).containsNaN())
            return false;
         if (externalForceStart.get(i).containsNaN())
            return false;
      }

      return true;
   }

   private void solveForCoefficientConstraintMatrix(List<? extends ContactStateProvider> contactSequence)
   {
      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      numberOfConstraints = 0;

      boolean hasExternalForce = hasExternalForceObjective();
      // set initial constraint
//      if (!hasExternalForce)
         setCoMPositionConstraint(0, 0.0, currentCoMPosition);
      setDynamicsInitialConstraint(contactSequence, 0);

      // add transition continuity constraints
      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int previousSequence = transition;
         int nextSequence = transition + 1;
         setCoMPositionContinuity(contactSequence, previousSequence, nextSequence);
         setCoMVelocityContinuity(contactSequence, previousSequence, nextSequence);
         setDynamicsFinalConstraint(contactSequence, previousSequence);
         setDynamicsInitialConstraint(contactSequence, nextSequence);
      }

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequence.get(numberOfPhases - 1);
      finalDCMPosition.set(modifiedEndVRPPositions.getLast());
      double finalDuration = lastContactPhase.getTimeInterval().getDuration();
      if (hasExternalForce)
         setCoMPositionConstraint(numberOfPhases - 1, finalDuration, finalDCMPosition);
      else
         setDCMPositionConstraint(numberOfPhases - 1, finalDuration, finalDCMPosition);

      setDynamicsFinalConstraint(contactSequence, numberOfPhases - 1);

      sparseSolver.setA(coefficientMultipliersSparse);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpXWaypoints, xEquivalents);
      CommonOps_DDRM.addEquals(xEquivalents, xConstants);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpYWaypoints, yEquivalents);
      CommonOps_DDRM.addEquals(yEquivalents, yConstants);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpZWaypoints, zEquivalents);
      CommonOps_DDRM.addEquals(zEquivalents, zConstants);

      sparseSolver.solve(xEquivalents, xCoefficientVector);
      sparseSolver.solve(yEquivalents, yCoefficientVector);
      sparseSolver.solve(zEquivalents, zCoefficientVector);
   }

   private final FramePoint3D comPositionToThrowAway = new FramePoint3D();
   private final FramePoint3D dcmPositionToThrowAway = new FramePoint3D();

   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FramePoint3D ecmpPositionToThrowAway = new FramePoint3D();

   private void updateCornerPoints(List<? extends ContactStateProvider> contactSequence)
   {
      comCornerPoints.clear();
      dcmCornerPoints.clear();
      vrpSegments.clear();

      boolean verboseBefore = verbose;
      verbose = false;
      for (int segmentId = 0; segmentId < Math.min(contactSequence.size(), maxCapacity + 1); segmentId++)
      {
         double duration = contactSequence.get(segmentId).getTimeInterval().getDuration();

         compute(segmentId,
                 0.0,
                 comCornerPoints.add(),
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmCornerPoints.add(),
                 dcmVelocityToThrowAway,
                 vrpStartPosition,
                 ecmpPositionToThrowAway);
         compute(segmentId,
                 duration,
                 comPositionToThrowAway,
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmPositionToThrowAway,
                 dcmVelocityToThrowAway,
                 vrpEndPosition,
                 ecmpPositionToThrowAway);

         vrpSegments.add().set(vrpStartPosition, vrpEndPosition);
      }

      verbose = verboseBefore;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void compute(int segmentId, double timeInPhase)
   {
      compute(segmentId,
              timeInPhase,
              desiredCoMPosition,
              desiredCoMVelocity,
              desiredCoMAcceleration,
              desiredDCMPosition,
              desiredDCMVelocity,
              desiredVRPPosition,
              desiredECMPPosition);

      if (verbose)
      {
         LogTools.info("At time " + timeInPhase + ", Desired DCM = " + desiredDCMPosition + ", Desired CoM = " + desiredCoMPosition);
      }
   }

   private final FrameVector3D desiredExternalAcceleration = new FrameVector3D();
   private final FrameVector3D desiredExternalAccelerationRate = new FrameVector3D();

   @Override
   public void compute(int segmentId,
                       double timeInPhase,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFramePoint3DBasics ecmpPositionToPack)
   {
      if (segmentId < 0)
         throw new IllegalArgumentException("time is invalid.");

      trajectoryHandler.compute(segmentId,
                                timeInPhase,
                                comPositionToPack,
                                comVelocityToPack,
                                comAccelerationToPack,
                                dcmPositionToPack,
                                dcmVelocityToPack,
                                vrpPositionToPack,
                                desiredVRPVelocity);

      // TODO subtract out the external force
      double denominator = -1.0 / (omega.getValue() * omega.getValue());
      double duration = Math.min(trajectoryHandler.getVrpTrajectories().get(segmentId).getDuration(), sufficientlyLongTime);
      double alpha = timeInPhase / duration;
      desiredExternalAcceleration.interpolate(externalForceStart.get(segmentId), externalForceEnd.get(segmentId), alpha);
      desiredExternalAccelerationRate.sub(externalForceEnd.get(segmentId), externalForceStart.get(segmentId));
      desiredExternalAccelerationRate.scale(1.0 / duration);

      if (!desiredExternalAcceleration.containsNaN())
         vrpPositionToPack.scaleAdd(denominator, desiredExternalAcceleration, vrpPositionToPack); // FIXME should this be negative?
      // TODO do VRP velocity

      ecmpPositionToPack.set(vrpPositionToPack);
      ecmpPositionToPack.subZ(comHeight.getDoubleValue());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.currentCoMPosition.setMatchingFrame(centerOfMassPosition);
      this.currentCoMVelocity.setMatchingFrame(centerOfMassVelocity);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   public FrameVector3DReadOnly getDesiredVRPVelocity()
   {
      return desiredVRPVelocity;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
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

         compute(segmentId,
                 timeInSegment,
                 comPositionToThrowAway,
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmPositionToThrowAway,
                 dcmVelocityToThrowAway,
                 vrpStartPosition,
                 ecmpPositionToThrowAway);

         comTrajectoryViewer.setBall(comPositionToThrowAway);
      }

      verbose = verboseBefore;
   }

   /**
    * Resets and resizes the internal matrices.
    */
   private void resetMatrices()
   {
      int size = indexHandler.getTotalNumberOfCoefficients();
      int numberOfVRPWaypoints = indexHandler.getNumberOfVRPWaypoints();

      coefficientMultipliersSparse.reshape(size, size);
      xEquivalents.reshape(size, 1);
      yEquivalents.reshape(size, 1);
      zEquivalents.reshape(size, 1);
      xConstants.reshape(size, 1);
      yConstants.reshape(size, 1);
      zConstants.reshape(size, 1);
      vrpWaypointJacobian.reshape(size, numberOfVRPWaypoints); // only position
      vrpXWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpYWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpZWaypoints.reshape(numberOfVRPWaypoints, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      coefficientMultipliersSparse.zero();
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
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p>
    * This constraint defines
    * </p>
    * <p>
    * x<sub>0</sub>(0) = x<sub>d</sub>,
    * </p>
    * <p>
    * substituting in the coefficients into the constraint matrix.
    * </p>
    *
    * @param centerOfMassLocationForConstraint x<sub>d</sub> in the above equations
    */
   private void setCoMPositionConstraint(int sequenceId, double time, FramePoint3DReadOnly centerOfMassLocationForConstraint)
   {
      CoMTrajectoryPlannerTools.addCoMPositionConstraint(centerOfMassLocationForConstraint,
                                                         omega.getValue(),
                                                         time,
                                                         sequenceId,
                                                         numberOfConstraints,
                                                         coefficientMultipliersSparse,
                                                         xConstants,
                                                         yConstants,
                                                         zConstants);
      numberOfConstraints++;
   }

   /**
    * <p> Sets a constraint on the desired DCM position. This constraint is useful for constraining the terminal location of the DCM trajectory. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> and the center of mass velocity is defined by </p>
    * <p>
    * d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    * 2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>
    * </p>
    * <p>
    * This constraint is then combining these two, saying
    * </p>
    * <p> x<sub>i</sub>(t<sub>i</sub>) + 1 / &omega; d/dt x<sub>i</sub>(t<sub>i</sub>) = &xi;<sub>d</sub>,</p>
    * <p> substituting in the appropriate coefficients. </p>
    *
    * @param sequenceId i in the above equations
    * @param time t<sub>i</sub> in the above equations
    * @param desiredDCMPosition desired DCM location. &xi;<sub>d</sub> in the above equations.
    */
   private void setDCMPositionConstraint(int sequenceId, double time, FramePoint3DReadOnly desiredDCMPosition)
   {
      CoMTrajectoryPlannerTools.addDCMPositionConstraint(sequenceId,
                                                         numberOfConstraints,
                                                         time,
                                                         omega.getValue(),
                                                         desiredDCMPosition,
                                                         coefficientMultipliersSparse,
                                                         xConstants,
                                                         yConstants,
                                                         zConstants);
      numberOfConstraints++;
   }

   /**
    * <p> Set a continuity constraint on the CoM position at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> x<sub>i-1</sub>(T<sub>i-1</sub>) = x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param contactSequence current contact sequence.
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setCoMPositionContinuity(List<? extends ContactStateProvider> contactSequence, int previousSequence, int nextSequence)
   {
      double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
      CoMTrajectoryPlannerTools.addCoMPositionContinuityConstraint(previousSequence,
                                                                   nextSequence,
                                                                   numberOfConstraints,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
      numberOfConstraints++;
   }

   /**
    * <p> Set a continuity constraint on the CoM velocity at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    * 2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> d / dt x<sub>i-1</sub>(T<sub>i-1</sub>) = d / dt x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param contactSequence current contact sequence.
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setCoMVelocityContinuity(List<? extends ContactStateProvider> contactSequence, int previousSequence, int nextSequence)
   {
      double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
      CoMTrajectoryPlannerTools.addCoMVelocityContinuityConstraint(previousSequence,
                                                                   nextSequence,
                                                                   numberOfConstraints,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
      numberOfConstraints++;
   }

   /**
    * Used to enforce the dynamics at the beginning of the trajectory segment {@param sequenceId}.
    *
    * @param contactSequence current contact sequence.
    * @param sequenceId desired trajectory segment.
    */
   private void setDynamicsInitialConstraint(List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      if (contactState.isLoadBearing())
      {
         constrainVRPPosition(sequenceId, indexHandler.getVRPWaypointStartPositionIndex(sequenceId), 0.0, modifiedStartVRPPositions.get(sequenceId));
         constrainVRPVelocity(sequenceId, indexHandler.getVRPWaypointStartVelocityIndex(sequenceId), 0.0, modifiedStartVRPVelocities.get(sequenceId));
      }
      else
      {
         constrainCoMAccelerationToGravity(sequenceId, 0.0);
         constrainCoMJerkToZero(sequenceId, 0.0);
      }
   }

   /**
    * Used to enforce the dynamics at the end of the trajectory segment {@param sequenceId}.
    *
    * @param contactSequence current contact sequence.
    * @param sequenceId desired trajectory segment.
    */
   private void setDynamicsFinalConstraint(List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      double duration = contactStateProvider.getTimeInterval().getDuration();
      if (contactState.isLoadBearing())
      {
         constrainVRPPosition(sequenceId, indexHandler.getVRPWaypointFinalPositionIndex(sequenceId), duration, modifiedEndVRPPositions.get(sequenceId));
         constrainVRPVelocity(sequenceId, indexHandler.getVRPWaypointFinalVelocityIndex(sequenceId), duration, modifiedEndVRPVelocities.get(sequenceId));
      }
      else
      {
         constrainCoMAccelerationToGravity(sequenceId, duration);
         constrainCoMJerkToZero(sequenceId, duration);
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
    *
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointPositionIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPPosition reference VRP position, v<sub>r</sub> in the above equations.
    */
   private void constrainVRPPosition(int sequenceId, int vrpWaypointPositionIndex, double time, FramePoint3DReadOnly desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addVRPPositionConstraint(sequenceId,
                                                         numberOfConstraints,
                                                         vrpWaypointPositionIndex,
                                                         time,
                                                         omega.getValue(),
                                                         desiredVRPPosition,
                                                         coefficientMultipliersSparse,
                                                         vrpXWaypoints,
                                                         vrpYWaypoints,
                                                         vrpZWaypoints,
                                                         vrpWaypointJacobian);
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
    *
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointVelocityIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPVelocity reference VRP veloctiy, d/dt v<sub>r</sub> in the above equations.
    */
   private void constrainVRPVelocity(int sequenceId, int vrpWaypointVelocityIndex, double time, FrameVector3DReadOnly desiredVRPVelocity)
   {
      CoMTrajectoryPlannerTools.addVRPVelocityConstraint(sequenceId,
                                                         numberOfConstraints,
                                                         vrpWaypointVelocityIndex,
                                                         omega.getValue(),
                                                         time,
                                                         desiredVRPVelocity,
                                                         coefficientMultipliersSparse,
                                                         vrpXWaypoints,
                                                         vrpYWaypoints,
                                                         vrpZWaypoints,
                                                         vrpWaypointJacobian);
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
    *
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   private void constrainCoMAccelerationToGravity(int sequenceId, double time)
   {
      CoMTrajectoryPlannerTools.constrainCoMAccelerationToGravity(sequenceId,
                                                                  numberOfConstraints,
                                                                  omega.getValue(),
                                                                  time,
                                                                  gravityZ,
                                                                  coefficientMultipliersSparse,
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
    *
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   private void constrainCoMJerkToZero(int sequenceId, double time)
   {
      CoMTrajectoryPlannerTools.constrainCoMJerkToZero(time, omega.getValue(), sequenceId, numberOfConstraints, coefficientMultipliersSparse);
      numberOfConstraints++;
   }

   public boolean hasTrajectories()
   {
      return trajectoryHandler.hasTrajectory();
   }

   public void removeCompletedSegments(double timeToCrop)
   {
      trajectoryHandler.removeCompletedSegments(timeToCrop);
   }

   @Override
   public List<Polynomial3DReadOnly> getVRPTrajectories()
   {
      if (!hasTrajectories())
         throw new RuntimeException("VRP Trajectories are not calculated");

      // FIXME these are wrong
      return trajectoryHandler.getVrpTrajectories();
   }

   @Override
   public MultipleCoMSegmentTrajectoryGenerator getCoMTrajectory()
   {
      if (!hasTrajectories())
         throw new RuntimeException("CoM Trajectories are not calculated");

      return trajectoryHandler.getComTrajectory();
   }
}
