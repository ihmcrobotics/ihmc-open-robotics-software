package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
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
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLongTime;

public class CoMTrajectoryModelPredictiveController
{
   private static boolean verbose = false;
   private static final boolean includeVelocityObjective = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int numberOfBasisVectorsPerContactPoint = 4;
   private static final int maxCapacity = 10;
   private static final double minRhoValue = 0.0;//05;
   private final double maxContactForce;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final CoMMPCSolutionInspection solutionInspection;

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private static final double mu = 0.8;
   public static final double HIGH_WEIGHT = 1e2;
   public static final double MEDIUM_WEIGHT = 1e2;
   public static final double LOW_WEIGHT = 1e1;

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
   private final YoFramePoint3D currentVRPPosition = new YoFramePoint3D("currentVRPPosition", worldFrame, registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);
   private final YoDouble currentTimeInState = new YoDouble("currentTimeInState", registry);
   private final YoDouble durationOfIgnoredSegments = new YoDouble("durationOfIgnoredSegments", registry);
   private final YoFramePoint3D finalDCMObjective = new YoFramePoint3D("finalDCMObjective", worldFrame, registry);

   private final YoDouble initialCoMPositionCostToGo = new YoDouble("initialCoMPositionCostToGo", registry);
   private final YoDouble initialCoMVelocityCostToGo = new YoDouble("initialCoMVelocityCostToGo", registry);

   private final YoDouble maximumPlanningHorizon = new YoDouble("maximumPlanningHorizon", registry);
   private final YoInteger maximumPlanningSegments = new YoInteger("maximumPlanningSegments", registry);
   private final YoInteger activeSegment = new YoInteger("activeSegment", registry);
   private final List<ContactPlaneProvider> planningSequence = new ArrayList<>();

   private final RecyclingArrayList<FramePoint3D> dcmCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final RecyclingArrayList<Trajectory3D> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   private final RecyclingArrayList<LineSegment3D> vrpSegments = new RecyclingArrayList<>(LineSegment3D::new);
   private final List<Trajectory3D> vrpTrajectories = new ArrayList<>();

   private final RecyclingArrayList<RecyclingArrayList<ContactPlaneHelper>> contactPlaneHelperPool;

   private final CoMTrajectoryPlanner initializationCalculator;

   private final CommandProvider commandProvider = new CommandProvider();
   final MPCCommandList mpcCommands = new MPCCommandList();

   final DMatrixRMaj xCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj yCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj zCoefficientVector = new DMatrixRMaj(0, 1);

   final CoMMPCQPSolver qpSolver;
   private CornerPointViewer viewer = null;
   private BagOfBalls comTrajectoryViewer = null;

   private final DoubleConsumer initialComPositionConsumer = initialCoMPositionCostToGo::set;
   private final DoubleConsumer initialComVelocityConsumer = initialCoMVelocityCostToGo::set;

   public CoMTrajectoryModelPredictiveController(double gravityZ, double nominalCoMHeight, double dt, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);
      this.omega = omega;

      initializationCalculator = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);

      this.maxContactForce = 2.0 * Math.abs(gravityZ);
      this.maximumPlanningHorizon.set(2.0);
      this.maximumPlanningSegments.set(2);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      indexHandler = new MPCIndexHandler(numberOfBasisVectorsPerContactPoint);

      FrictionConeRotationCalculator coneRotationCalculator = new ZeroConeRotationCalculator();
      Supplier<ContactPlaneHelper> contactPlaneHelperProvider = () -> new ContactPlaneHelper(6, numberOfBasisVectorsPerContactPoint, coneRotationCalculator);
      contactPlaneHelperPool = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(contactPlaneHelperProvider));

      qpSolver = new CoMMPCQPSolver(indexHandler, dt, gravityZ, registry);
      solutionInspection = new CoMMPCSolutionInspection(indexHandler, gravityZ);

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

   public void setContactPlaneViewers(Supplier<ContactPlaneForceViewer> viewerSupplier)
   {
      contactPlaneHelperPool.clear();
      for (int i = 0; i < 2; i++)
      {
         RecyclingArrayList<ContactPlaneHelper> helpers = contactPlaneHelperPool.add();
         helpers.clear();
         for (int j = 0; j < 6; j++)
         {
            helpers.add().setContactPointForceViewer(viewerSupplier.get());
         }
         helpers.clear();
      }
      contactPlaneHelperPool.clear();
   }

   /**
    * {@inheritDoc}
    */
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.comHeight.set(nominalCoMHeight);
      initializationCalculator.setNominalCoMHeight(nominalCoMHeight);
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

      initializationCalculator.solveForTrajectory(contactSequence);
      double horizonLength = computePlanningHorizon(contactSequence);
      initializationCalculator.compute(horizonLength + durationOfIgnoredSegments.getDoubleValue());
      finalDCMObjective.set(initializationCalculator.getDesiredDCMPosition());

      indexHandler.initialize(planningSequence);

      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                                    gravityZ,
                                                    omega.getValue(),
                                                    currentCoMVelocity,
                                                    planningSequence,
                                                    startVRPPositions,
                                                    endVRPPositions,
                                                    true);

      commandProvider.reset();
      mpcCommands.clear();

      computeMatrixHelpers(planningSequence);
      computeObjectives(planningSequence);
      solveQP(planningSequence.size());

      updateCornerPoints(planningSequence);

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

   private double computePlanningHorizon(List<ContactPlaneProvider> fullContactSequence)
   {
      planningSequence.clear();
      double horizonDuration = 0.0;

      int activeSegment = -1;
      durationOfIgnoredSegments.set(0.0);
      for (int i = 0; i < fullContactSequence.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = fullContactSequence.get(i).getTimeInterval();
         if (timeInterval.intervalContains(currentTimeInState.getDoubleValue()))
         {
            activeSegment = i;
            break;
         }
         durationOfIgnoredSegments.add(timeInterval.getDuration());
      }

      if (this.activeSegment.getIntegerValue() != activeSegment)
      {
         qpSolver.resetRateRegularization();
         qpSolver.notifyResetActiveSet();
      }

      this.activeSegment.set(activeSegment);

      for (int i = activeSegment; i < fullContactSequence.size(); i++)
      {
         ContactPlaneProvider contact = fullContactSequence.get(i);

         planningSequence.add(contact);
         horizonDuration += contact.getTimeInterval().getDuration();

         if (contact.getContactState().isLoadBearing() && (horizonDuration >= maximumPlanningHorizon.getDoubleValue() || planningSequence.size() > maximumPlanningSegments.getValue() - 1))
            break;
      }

      return horizonDuration;
   }

   private void computeMatrixHelpers(List<ContactPlaneProvider> contactSequence)
   {
      contactPlaneHelperPool.clear();

      for (int sequenceId = 0; sequenceId < contactSequence.size(); sequenceId++)
      {
         ContactPlaneProvider contact = contactSequence.get(sequenceId);
         double duration = contact.getTimeInterval().getDuration();

         RecyclingArrayList<ContactPlaneHelper> contactPlaneHelpers = contactPlaneHelperPool.add();
         contactPlaneHelpers.clear();

         double objectiveForce = gravityZ / contact.getNumberOfContactPlanes();
         for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         {
            ContactPlaneHelper contactPlaneHelper = contactPlaneHelpers.add();
            contactPlaneHelper.setMaxNormalForce(maxContactForce);
            contactPlaneHelper.computeBasisVectors(contact.getContactsInBodyFrame(contactId), contact.getContactPose(contactId), mu);
            contactPlaneHelper.computeAccelerationIntegrationMatrix(duration, omega.getValue(), objectiveForce);
         }
      }
   }

   private void computeObjectives(List<ContactPlaneProvider> contactSequence)
   {
      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      mpcCommands.addCommand(computeInitialCoMPositionObjective(commandProvider.getNextCoMPositionCommand()));
      if (includeVelocityObjective)
         mpcCommands.addCommand(computeInitialCoMVelocityObjective(commandProvider.getNextCoMVelocityCommand()));
      if (contactSequence.get(0).getContactState().isLoadBearing())
      {
         double duration = contactSequence.get(0).getTimeInterval().getDuration();
         mpcCommands.addCommand(computeVRPSegmentObjective(LOW_WEIGHT,
                                                           commandProvider.getNextVRPPositionCommand(),
                                                           commandProvider.getNextVRPVelocityCommand(),
                                                           startVRPPositions.get(0),
                                                           0,
                                                           duration,
                                                           0.0));
         mpcCommands.addCommand(computeMinRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), 0, 0.0));
//         mpcCommands.addCommand(computeMaxRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), 0, currentTimeInState.getDoubleValue()));
      }

      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int nextSequence = transition + 1;

         double firstSegmentDuration = contactSequence.get(transition).getTimeInterval().getDuration();

         mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextComPositionContinuityCommand(), transition, firstSegmentDuration));
         mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextComVelocityContinuityCommand(), transition, firstSegmentDuration));

         if (contactSequence.get(transition).getContactState().isLoadBearing())
         {
            mpcCommands.addCommand(computeVRPSegmentObjective(commandProvider.getNextVRPPositionCommand(),
                                                              commandProvider.getNextVRPVelocityCommand(),
                                                              endVRPPositions.get(transition),
                                                              transition,
                                                              firstSegmentDuration,
                                                              firstSegmentDuration));
            mpcCommands.addCommand(computeMinRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), transition, firstSegmentDuration));
//            mpcCommands.addCommand(computeMaxRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), transition, firstSegmentDuration));
         }
         if (contactSequence.get(nextSequence).getContactState().isLoadBearing())
         {
            double duration = Math.min(contactSequence.get(nextSequence).getTimeInterval().getDuration(), sufficientlyLongTime);
            mpcCommands.addCommand(computeVRPSegmentObjective(commandProvider.getNextVRPPositionCommand(),
                                                              commandProvider.getNextVRPVelocityCommand(),
                                                              startVRPPositions.get(nextSequence),
                                                              nextSequence,
                                                              duration,
                                                              0.0));
            mpcCommands.addCommand(computeMinRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), nextSequence, 0.0));
//            mpcCommands.addCommand(computeMaxRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), nextSequence, 0.0));
         }
      }

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequence.get(numberOfPhases - 1);
      double finalDuration = Math.min(lastContactPhase.getTimeInterval().getDuration(), sufficientlyLongTime);
      mpcCommands.addCommand(computeDCMPositionObjective(commandProvider.getNextDCMPositionCommand(),
                                                         finalDCMObjective,
                                                         numberOfPhases - 1,
                                                         finalDuration));
      mpcCommands.addCommand(computeVRPSegmentObjective(commandProvider.getNextVRPPositionCommand(),
                                                        commandProvider.getNextVRPVelocityCommand(),
                                                        startVRPPositions.get(numberOfPhases - 1),
                                                        numberOfPhases - 1,
                                                        finalDuration,
                                                        finalDuration));
      mpcCommands.addCommand(computeMinRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), numberOfPhases - 1, finalDuration));
//      mpcCommands.addCommand(computeMaxRhoObjective(commandProvider.getNextRhoValueObjectiveCommand(), numberOfPhases - 1, finalDuration));
   }

   private MPCCommand<?> computeInitialCoMPositionObjective(CoMPositionCommand objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(HIGH_WEIGHT);
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(currentTimeInState.getDoubleValue() - durationOfIgnoredSegments.getDoubleValue());
      objectiveToPack.setObjective(currentCoMPosition);
      objectiveToPack.setCostToGoConsumer(initialComPositionConsumer);
//      objectiveToPack.setConstraintType(ConstraintType.EQUALITY);
      for (int i = 0; i < contactPlaneHelperPool.get(0).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(0).get(i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeInitialCoMVelocityObjective(CoMVelocityCommand objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(HIGH_WEIGHT);
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(currentTimeInState.getDoubleValue() - durationOfIgnoredSegments.getDoubleValue());
      objectiveToPack.setObjective(currentCoMVelocity);
      objectiveToPack.setCostToGoConsumer(initialComVelocityConsumer);
//      objectiveToPack.setConstraintType(ConstraintType.EQUALITY);
      for (int i = 0; i < contactPlaneHelperPool.get(0).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(0).get(i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeContinuityObjective(CoMContinuityCommand continuityObjectiveToPack, int firstSegmentNumber, double firstSegmentDuration)
   {
      continuityObjectiveToPack.clear();
      continuityObjectiveToPack.setOmega(omega.getValue());
      continuityObjectiveToPack.setFirstSegmentNumber(firstSegmentNumber);
      continuityObjectiveToPack.setFirstSegmentDuration(firstSegmentDuration);
      continuityObjectiveToPack.setConstraintType(ConstraintType.EQUALITY);

      for (int i = 0; i < contactPlaneHelperPool.get(firstSegmentNumber).size(); i++)
      {
         continuityObjectiveToPack.addFirstSegmentContactPlaneHelper(contactPlaneHelperPool.get(firstSegmentNumber).get(i));
      }

      for (int i = 0; i < contactPlaneHelperPool.get(firstSegmentNumber + 1).size(); i++)
      {
         continuityObjectiveToPack.addSecondSegmentContactPlaneHelper(contactPlaneHelperPool.get(firstSegmentNumber + 1).get(i));
      }

      return continuityObjectiveToPack;
   }

   private MPCCommand<?> computeMinRhoObjective(RhoValueObjectiveCommand valueObjective, int segmentNumber, double constraintTime)
   {
      valueObjective.clear();
      valueObjective.setOmega(omega.getValue());
      valueObjective.setTimeOfObjective(constraintTime);
      valueObjective.setSegmentNumber(segmentNumber);
      valueObjective.setConstraintType(ConstraintType.GEQ_INEQUALITY);
      valueObjective.setScalarObjective(minRhoValue);
      valueObjective.setUseScalarObjective(true);
      for (int i = 0; i < contactPlaneHelperPool.get(segmentNumber).size(); i++)
      {
         valueObjective.addContactPlaneHelper(contactPlaneHelperPool.get(segmentNumber).get(i));
      }

      return valueObjective;
   }

   private MPCCommand<?> computeMaxRhoObjective(RhoValueObjectiveCommand valueObjective, int segmentNumber, double constraintTime)
   {
      valueObjective.clear();
      valueObjective.setOmega(omega.getValue());
      valueObjective.setTimeOfObjective(constraintTime);
      valueObjective.setSegmentNumber(segmentNumber);
      valueObjective.setConstraintType(ConstraintType.LEQ_INEQUALITY);
      valueObjective.setUseScalarObjective(false);

      int objectiveSize = 0;
      for (int i = 0; i < contactPlaneHelperPool.get(segmentNumber).size(); i++)
      {
         ContactPlaneHelper contactPlaneHelper = contactPlaneHelperPool.get(segmentNumber).get(i);
         objectiveSize += contactPlaneHelper.getRhoSize();
         valueObjective.addContactPlaneHelper(contactPlaneHelper);
      }

      int rowStart = 0;
      DMatrixRMaj objectiveVector = valueObjective.getObjectiveVector();
      objectiveVector.reshape(objectiveSize, 1);
      for (int i = 0; i < contactPlaneHelperPool.get(segmentNumber).size(); i++)
      {
         ContactPlaneHelper contactPlaneHelper = contactPlaneHelperPool.get(segmentNumber).get(i);
         MatrixTools.setMatrixBlock(objectiveVector, rowStart, 0, contactPlaneHelper.getRhoMaxMatrix(), 0, 0, contactPlaneHelper.getRhoSize(), 1, 1.0);

         rowStart += contactPlaneHelper.getRhoSize();
      }

      return valueObjective;
   }

   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final MPCCommandList segmentObjectiveList = new MPCCommandList();

   private MPCCommand<?> computeVRPSegmentObjective(VRPPositionCommand positionObjectiveToPack,
                                                    VRPVelocityCommand velocityObjectiveToPack,
                                                    FramePoint3DReadOnly desiredVRPPosition,
                                                    int segmentNumber,
                                                    double segmentDuration,
                                                    double constraintTime)
   {
      return computeVRPSegmentObjective(MEDIUM_WEIGHT, positionObjectiveToPack, velocityObjectiveToPack, desiredVRPPosition, segmentNumber, segmentDuration, constraintTime);
   }

   private MPCCommand<?> computeVRPSegmentObjective(double weight,
                                                    VRPPositionCommand positionObjectiveToPack,
                                                    VRPVelocityCommand velocityObjectiveToPack,
                                                    FramePoint3DReadOnly desiredVRPPosition,
                                                    int segmentNumber,
                                                    double segmentDuration,
                                                    double constraintTime)
   {
      desiredVelocity.sub(endVRPPositions.get(segmentNumber), startVRPPositions.get(segmentNumber));
      desiredVelocity.scale(1.0 / segmentDuration);

      segmentObjectiveList.clear();
      segmentObjectiveList.addCommand(computeVRPObjective(weight, positionObjectiveToPack, segmentNumber, constraintTime, desiredVRPPosition));
      segmentObjectiveList.addCommand(computeVRPObjective(weight, velocityObjectiveToPack, segmentNumber, constraintTime, desiredVelocity));

      return segmentObjectiveList;
   }

   private MPCCommand<?> computeVRPObjective(MPCValueCommand objectiveToPack, int segmentNumber, double constraintTime, FrameTuple3DReadOnly objective)
   {
      return computeVRPObjective(MEDIUM_WEIGHT, objectiveToPack, segmentNumber, constraintTime, objective);
   }

   private MPCCommand<?> computeVRPObjective(double weight, MPCValueCommand objectiveToPack, int segmentNumber, double constraintTime, FrameTuple3DReadOnly objective)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(weight);
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeOfObjective(constraintTime);
      objectiveToPack.setObjective(objective);
      for (int i = 0; i < contactPlaneHelperPool.get(segmentNumber).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(segmentNumber).get(i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeDCMPositionObjective(DCMPositionCommand objectiveToPack,
                                                     FramePoint3DReadOnly desiredPosition,
                                                     int segmentNumber,
                                                     double timeOfObjective)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeOfObjective(timeOfObjective);
      objectiveToPack.setObjective(desiredPosition);
      objectiveToPack.setConstraintType(ConstraintType.EQUALITY);
      for (int i = 0; i < contactPlaneHelperPool.get(segmentNumber).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(segmentNumber).get(i));
      }

      return objectiveToPack;
   }

   private void solveQP(int numberOfPhases)
   {
      xCoefficientVector.reshape(6 * numberOfPhases, 1);
      yCoefficientVector.reshape(6 * numberOfPhases, 1);
      zCoefficientVector.reshape(6 * numberOfPhases, 1);
      xCoefficientVector.zero();
      yCoefficientVector.zero();
      zCoefficientVector.zero();

      qpSolver.initialize();
      qpSolver.submitMPCCommandList(mpcCommands);
      if (!qpSolver.solve())
      {
         LogTools.info("Failed to find solution");
         return;
      }

      DMatrixRMaj solutionCoefficients = qpSolver.getSolution();

      solutionInspection.inspectSolution(mpcCommands, solutionCoefficients);

      // FIXME do something here
      for (int sequence = 0; sequence < contactPlaneHelperPool.size(); sequence++)
      {
         int coeffStartIdx = indexHandler.getRhoCoefficientStartIndex(sequence);

         for (int contact = 0; contact < contactPlaneHelperPool.get(sequence).size(); contact++)
         {
            ContactPlaneHelper contactPlaneHelper = contactPlaneHelperPool.get(sequence).get(contact);
            contactPlaneHelper.computeContactForceCoefficientMatrix(solutionCoefficients, coeffStartIdx);
            coeffStartIdx += contactPlaneHelper.getCoefficientSize();
         }
      }

      for (int i = 0; i < numberOfPhases; i++)
      {
         int vectorStart = 6 * i;

         xCoefficientVector.set(vectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0), 0));
         yCoefficientVector.set(vectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1), 0));
         zCoefficientVector.set(vectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2), 0));

         xCoefficientVector.set(vectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0) + 1, 0));
         yCoefficientVector.set(vectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1) + 1, 0));
         zCoefficientVector.set(vectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2) + 1, 0));

         for (int contactIdx = 0; contactIdx < contactPlaneHelperPool.get(i).size(); contactIdx++)
         {
            ContactPlaneHelper contactPlaneHelper = contactPlaneHelperPool.get(i).get(contactIdx);
            DMatrixRMaj contactCoefficientMatrix = contactPlaneHelper.getContactWrenchCoefficientMatrix();

            xCoefficientVector.add(vectorStart, 0, contactCoefficientMatrix.get(0, 0));
            yCoefficientVector.add(vectorStart, 0, contactCoefficientMatrix.get(1, 0));
            zCoefficientVector.add(vectorStart, 0, contactCoefficientMatrix.get(2, 0));

            xCoefficientVector.add(vectorStart + 1, 0, contactCoefficientMatrix.get(0, 1));
            yCoefficientVector.add(vectorStart + 1, 0, contactCoefficientMatrix.get(1, 1));
            zCoefficientVector.add(vectorStart + 1, 0, contactCoefficientMatrix.get(2, 1));

            xCoefficientVector.add(vectorStart + 2, 0, contactCoefficientMatrix.get(0, 2));
            yCoefficientVector.add(vectorStart + 2, 0, contactCoefficientMatrix.get(1, 2));
            zCoefficientVector.add(vectorStart + 2, 0, contactCoefficientMatrix.get(2, 2));

            xCoefficientVector.add(vectorStart + 3, 0, contactCoefficientMatrix.get(0, 3));
            yCoefficientVector.add(vectorStart + 3, 0, contactCoefficientMatrix.get(1, 3));
            zCoefficientVector.add(vectorStart + 3, 0, contactCoefficientMatrix.get(2, 3));
         }

         zCoefficientVector.add(vectorStart + 3, 0, -0.5 * gravityZ);
      }
   }

   public void compute(double timeInPhase)
   {
      timeInPhase -= durationOfIgnoredSegments.getDoubleValue();
      int segmentNumber = getSegmentNumber(timeInPhase);
      double timeInSegment = getTimeInSegment(segmentNumber, timeInPhase);
      compute(segmentNumber, timeInSegment);
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

         duration = Math.min(duration, sufficientlyLongTime);
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

         if (segmentId < 0)
            break;

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

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();
   private final FramePoint3D fifthCoefficient = new FramePoint3D();
   private final FramePoint3D sixthCoefficient = new FramePoint3D();

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

      int startIndex = 6 * segmentId;
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

      CoMMPCTools.constructDesiredCoMPosition(comPositionToPack,
                                                            firstCoefficient,
                                                            secondCoefficient,
                                                            thirdCoefficient,
                                                            fourthCoefficient,
                                                            fifthCoefficient,
                                                            sixthCoefficient,
                                                            timeInPhase,
                                                            omega);
      CoMMPCTools.constructDesiredCoMVelocity(comVelocityToPack,
                                                            firstCoefficient,
                                                            secondCoefficient,
                                                            thirdCoefficient,
                                                            fourthCoefficient,
                                                            fifthCoefficient,
                                                            sixthCoefficient,
                                                            timeInPhase,
                                                            omega);
      CoMMPCTools.constructDesiredCoMAcceleration(comAccelerationToPack,
                                                                firstCoefficient,
                                                                secondCoefficient,
                                                                thirdCoefficient,
                                                                fourthCoefficient,
                                                                fifthCoefficient,
                                                                sixthCoefficient,
                                                                timeInPhase,
                                                                omega);

      CoMMPCTools.constructDesiredVRPVelocity(vrpVelocityToPack,
                                                            firstCoefficient,
                                                            secondCoefficient,
                                                            thirdCoefficient,
                                                            fourthCoefficient,
                                                            fifthCoefficient,
                                                            sixthCoefficient,
                                                            timeInPhase,
                                                            omega);

      CapturePointTools.computeCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPositionToPack);
      CapturePointTools.computeCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      CapturePointTools.computeCentroidalMomentumPivot(dcmPositionToPack, dcmVelocityToPack, omega, vrpPositionToPack);

      ecmpPositionToPack.set(vrpPositionToPack);
      ecmpPositionToPack.subZ(comHeight.getDoubleValue());

      segmentId = Math.min(segmentId, contactPlaneHelperPool.size() - 1);
      for (int i = 0; i < contactPlaneHelperPool.get(segmentId).size(); i++)
      {
         contactPlaneHelperPool.get(segmentId).get(i).computeContactForce(omega, timeInPhase);
      }
   }

   /**
    * {@inheritDoc}
    */
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      initializationCalculator.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void setCurrentCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition,
                                           FrameVector3DReadOnly centerOfMassVelocity,
                                           FramePoint3DReadOnly currentVRPPosition,
                                           double timeInState)
   {
      this.currentCoMPosition.setMatchingFrame(centerOfMassPosition);
      this.currentCoMVelocity.setMatchingFrame(centerOfMassVelocity);
      this.currentVRPPosition.setMatchingFrame(currentVRPPosition);
      this.currentTimeInState.set(timeInState);
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
