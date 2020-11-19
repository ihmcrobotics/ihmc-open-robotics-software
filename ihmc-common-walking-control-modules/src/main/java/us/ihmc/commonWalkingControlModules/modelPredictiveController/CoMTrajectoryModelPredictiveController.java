package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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
public class CoMTrajectoryModelPredictiveController
{
   private static boolean verbose = false;
   private static final boolean includeVelocityObjective = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int numberOfBasisVectorsPerContactPoint = 4;
   private static final int maxCapacity = 10;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private static final double mu = 0.8;
   private static final double rhoWeight = 1e-5;
   private static final double rhoRateWeight = 1e-8;
   public static final double MEDIUM_WEIGHT = 1e1;

   private final MPCIndexHandler indexHandler;

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

   private final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition", worldFrame, registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);

   private final RecyclingArrayList<FramePoint3D> dcmCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final RecyclingArrayList<Trajectory3D> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   private final RecyclingArrayList<LineSegment3D> vrpSegments = new RecyclingArrayList<>(LineSegment3D::new);
   private final List<Trajectory3D> vrpTrajectories = new ArrayList<>();

   private final RecyclingArrayList<RecyclingArrayList<ContactStateMagnitudeToForceMatrixHelper>> rhoJacobianHelperPool;
   private final RecyclingArrayList<RecyclingArrayList<CoefficientJacobianMatrixHelper>> coefficientJacobianHelperPool;

   private final RecyclingArrayList<MPCValueObjective> comValueObjectivePool = new RecyclingArrayList<>(MPCValueObjective::new);
   private final RecyclingArrayList<CoMContinuityObjective> comContinuityObjectivePool = new RecyclingArrayList<>(CoMContinuityObjective::new);
   private final MPCCommandList mpcCommands = new MPCCommandList();

   private CornerPointViewer viewer = null;
   private BagOfBalls comTrajectoryViewer = null;

   public CoMTrajectoryModelPredictiveController(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);
      this.omega = omega;

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      indexHandler = new MPCIndexHandler(numberOfBasisVectorsPerContactPoint);

      FrictionConeRotationCalculator coneRotationCalculator = new ZeroConeRotationCalculator();
      Supplier<ContactStateMagnitudeToForceMatrixHelper> rhoJacobianProvider = () -> new ContactStateMagnitudeToForceMatrixHelper(6,
                                                                                                                                  numberOfBasisVectorsPerContactPoint,
                                                                                                                                  coneRotationCalculator);
      Supplier<CoefficientJacobianMatrixHelper> coefficientJacobianProvider = () -> new CoefficientJacobianMatrixHelper(6, numberOfBasisVectorsPerContactPoint);
      rhoJacobianHelperPool = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(rhoJacobianProvider));
      coefficientJacobianHelperPool = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(coefficientJacobianProvider));

      parentRegistry.addChild(registry);
   }

   public void setCornerPointViewer(CornerPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void setupCoMTrajectoryViewer(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      comTrajectoryViewer = new BagOfBalls(50, 0.01, YoAppearance.Black(), registry, yoGraphicsListRegistry);
   }

   /**
    * {@inheritDoc}
    */
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.comHeight.set(nominalCoMHeight);
   }

   /**
    * {@inheritDoc}
    */
   public double getNominalCoMHeight()
   {
      return comHeight.getDoubleValue();
   }

   /**
    * {@inheritDoc}
    */
   public void solveForTrajectory(List<ContactPlaneProvider> contactSequence)
   {
      if (!ContactStateProviderTools.checkContactSequenceIsValid(contactSequence))
         throw new IllegalArgumentException("The contact sequence is not valid.");

      indexHandler.initialize(contactSequence);

      resetMatrices();

      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                                    gravityZ,
                                                    omega.getValue(),
                                                    currentCoMVelocity,
                                                    contactSequence,
                                                    startVRPPositions,
                                                    endVRPPositions,
                                                    true);
      computeMatrixHelpers(contactSequence);
      computeObjectives(contactSequence);

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

   private void computeMatrixHelpers(List<ContactPlaneProvider> contactSequence)
   {
      rhoJacobianHelperPool.clear();
      coefficientJacobianHelperPool.clear();

      for (int sequenceId = 0; sequenceId < contactSequence.size(); sequenceId++)
      {
         ContactPlaneProvider contact = contactSequence.get(sequenceId);

         RecyclingArrayList<ContactStateMagnitudeToForceMatrixHelper> rhoJacobians = rhoJacobianHelperPool.add();
         RecyclingArrayList<CoefficientJacobianMatrixHelper> coefficientJacobians = coefficientJacobianHelperPool.add();
         rhoJacobians.clear();
         coefficientJacobians.clear();

         for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         {
            ContactStateMagnitudeToForceMatrixHelper rhoJacobian = rhoJacobians.add();
            CoefficientJacobianMatrixHelper coefficientJacobian = coefficientJacobians.add();

            rhoJacobian.computeMatrices(contact.getContactsInBodyFrame(contactId), contact.getContactPose(contactId), rhoWeight, rhoRateWeight, mu);
            coefficientJacobian.reshape(contact.getNumberOfContactPointsInPlane(contactId));
         }
      }
   }

   private void computeObjectives(List<ContactPlaneProvider> contactSequence)
   {
      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      comValueObjectivePool.clear();
      comContinuityObjectivePool.clear();
      mpcCommands.clear();

      mpcCommands.addCommand(computeInitialCoMPositionObjective(comValueObjectivePool.add()));
      if (includeVelocityObjective)
         mpcCommands.addCommand(computeInitialCoMVelocityObjective(comValueObjectivePool.add()));
      if (contactSequence.get(0).getContactState().isLoadBearing())
      {
         double duration = contactSequence.get(0).getTimeInterval().getDuration();
         mpcCommands.addCommand(computeVRPSegmentObjective(comValueObjectivePool.add(), comValueObjectivePool.add(), startVRPPositions.get(0), 0, duration, 0.0));
      }

      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int nextSequence = transition + 1;

         double firstSegmentDuration = contactSequence.get(transition).getTimeInterval().getDuration();

         mpcCommands.addCommand(computeContinuityObjective(comContinuityObjectivePool.add(), transition, firstSegmentDuration, 0));
         mpcCommands.addCommand(computeContinuityObjective(comContinuityObjectivePool.add(), transition, firstSegmentDuration, 1));

         if (contactSequence.get(transition).getContactState().isLoadBearing())
         {
            mpcCommands.addCommand(computeVRPSegmentObjective(comValueObjectivePool.add(), comValueObjectivePool.add(), endVRPPositions.get(transition), transition, firstSegmentDuration, firstSegmentDuration));
         }
         if (contactSequence.get(nextSequence).getContactState().isLoadBearing())
         {
            double duration = contactSequence.get(nextSequence).getTimeInterval().getDuration();
            mpcCommands.addCommand(computeVRPSegmentObjective(comValueObjectivePool.add(), comValueObjectivePool.add(), startVRPPositions.get(nextSequence), nextSequence, duration, 0.0));
         }
      }
   }

   private MPCCommand<?> computeInitialCoMPositionObjective(MPCValueObjective objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setValueType(MPCValueType.COM);
      objectiveToPack.setDerivativeOrder(0);
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(0.0);
      objectiveToPack.setObjective(currentCoMPosition);
      for (int i = 0; i < rhoJacobianHelperPool.get(0).size(); i++)
      {
         objectiveToPack.addRhoToForceMatrixHelper(rhoJacobianHelperPool.get(0).get(i));
         objectiveToPack.addJacobianMatrixHelper(coefficientJacobianHelperPool.get(0).get(i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeInitialCoMVelocityObjective(MPCValueObjective objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setValueType(MPCValueType.COM);
      objectiveToPack.setDerivativeOrder(1);
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(0.0);
      objectiveToPack.setObjective(currentCoMVelocity);
      for (int i = 0; i < rhoJacobianHelperPool.get(0).size(); i++)
      {
         objectiveToPack.addRhoToForceMatrixHelper(rhoJacobianHelperPool.get(0).get(i));
         objectiveToPack.addJacobianMatrixHelper(coefficientJacobianHelperPool.get(0).get(i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeContinuityObjective(CoMContinuityObjective continuityObjectiveToPack,
                                                    int firstSegmentNumber,
                                                    double firstSegmentDuration,
                                                    int derivativeOrder)
   {
      continuityObjectiveToPack.clear();
      continuityObjectiveToPack.setDerivativeOrder(0);
      continuityObjectiveToPack.setOmega(omega.getValue());
      continuityObjectiveToPack.setFirstSegmentNumber(firstSegmentNumber);
      continuityObjectiveToPack.setFirstSegmentDuration(firstSegmentDuration);
      continuityObjectiveToPack.setDerivativeOrder(derivativeOrder);

      for (int i = 0; i < rhoJacobianHelperPool.get(firstSegmentNumber).size(); i++)
      {
         continuityObjectiveToPack.addFirstSegmentRhoToForceMatrixHelper(rhoJacobianHelperPool.get(firstSegmentNumber).get(i));
         continuityObjectiveToPack.addFirstSegmentJacobianMatrixHelper(coefficientJacobianHelperPool.get(firstSegmentNumber).get(i));
      }

      for (int i = 0; i < rhoJacobianHelperPool.get(firstSegmentNumber + 1).size(); i++)
      {
         continuityObjectiveToPack.addSecondSegmentRhoToForceMatrixHelper(rhoJacobianHelperPool.get(firstSegmentNumber + 1).get(i));
         continuityObjectiveToPack.addSecondSegmentJacobianMatrixHelper(coefficientJacobianHelperPool.get(firstSegmentNumber + 1).get(i));
      }

      return continuityObjectiveToPack;
   }

   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final MPCCommandList segmentObjectiveList = new MPCCommandList();

   private MPCCommand<?> computeVRPSegmentObjective(MPCValueObjective positionObjectiveToPack,
                                                    MPCValueObjective velocityObjectiveToPack,
                                                    FramePoint3DReadOnly desiredVRPPosition,
                                                    int segmentNumber,
                                                    double segmentDuration,
                                                    double constraintTime)
   {
      desiredVelocity.sub(endVRPPositions.get(segmentNumber), startVRPPositions.get(segmentNumber));
      desiredVelocity.scale(1.0 / segmentDuration);

      segmentObjectiveList.clear();
      segmentObjectiveList.addCommand(computeVRPObjective(positionObjectiveToPack, segmentNumber, 0, constraintTime, desiredVRPPosition));
      segmentObjectiveList.addCommand(computeVRPObjective(velocityObjectiveToPack, segmentNumber, 1, constraintTime, desiredVelocity));

      return segmentObjectiveList;
   }

   private MPCCommand<?> computeVRPObjective(MPCValueObjective objectiveToPack,
                                    int segmentNumber,
                                    int derivativeOrder,
                                    double constraintTime,
                                    FrameTuple3DReadOnly objective)
   {
      objectiveToPack.clear();
      objectiveToPack.setValueType(MPCValueType.VRP);
      objectiveToPack.setDerivativeOrder(derivativeOrder);
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeOfObjective(constraintTime);
      objectiveToPack.setObjective(objective);
      for (int i = 0; i < rhoJacobianHelperPool.get(segmentNumber).size(); i++)
      {
         objectiveToPack.addRhoToForceMatrixHelper(rhoJacobianHelperPool.get(segmentNumber).get(i));
         objectiveToPack.addJacobianMatrixHelper(coefficientJacobianHelperPool.get(segmentNumber).get(i));
      }

      return objectiveToPack;
   }



   /**
    * Resets and resizes the internal matrices.
    */
   private void resetMatrices()
   {

   }

   private void computeContacts()
   {

   }

   public int getSegmentNumber(double time)
   {
      double startTime = 0.0;
      for (int i = 0; i < getVRPTrajectories().size(); i++)
      {
         if (getVRPTrajectories().get(i).timeIntervalContains(time - startTime))
            return i;

         startTime += getVRPTrajectories().get(i).getDuration();
      }

      return -1;
   }

   public double getTimeInSegment(int segmentNumber, double time)
   {
      for (int i = 0; i < segmentNumber; i++)
         time -= getVRPTrajectories().get(i).getDuration();

      return time;
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
         compute(segmentId,
                 0.0,
                 comCornerPoints.add(),
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmCornerPoints.add(),
                 dcmVelocityToThrowAway,
                 vrpStartPosition,
                 vrpStartVelocity,
                 ecmpPositionToThrowAway);
         compute(segmentId,
                 duration,
                 comPositionToThrowAway,
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmPositionToThrowAway,
                 dcmVelocityToThrowAway,
                 vrpEndPosition,
                 vrpEndVelocity,
                 ecmpPositionToThrowAway);

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
    * {@inheritDoc}
    */
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
      compute(segmentId,
              timeInPhase,
              comPositionToPack,
              comVelocityToPack,
              comAccelerationToPack,
              dcmPositionToPack,
              dcmVelocityToPack,
              vrpPositionToPack,
              vrpVelocityToThrowAway,
              ecmpPositionToPack);
   }

   public void compute(int segmentId,
                       double timeInPhase,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFrameVector3DBasics vrpVelocityToPack,
                       FixedFramePoint3DBasics ecmpPositionToPack)
   {
      if (segmentId < 0)
         throw new IllegalArgumentException("time is invalid.");
   }

   /**
    * {@inheritDoc}
    */
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.currentCoMPosition.setMatchingFrame(centerOfMassPosition);
      this.currentCoMVelocity.setMatchingFrame(centerOfMassVelocity);
   }

   /**
    * {@inheritDoc}
    */
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   /**
    * {@inheritDoc}
    */
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   /**
    * {@inheritDoc}
    */
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   /**
    * {@inheritDoc}
    */
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   /**
    * {@inheritDoc}
    */
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   /**
    * {@inheritDoc}
    */
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
   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
   }

   public List<Trajectory3D> getVRPTrajectories()
   {
      return vrpTrajectories;
   }
}
