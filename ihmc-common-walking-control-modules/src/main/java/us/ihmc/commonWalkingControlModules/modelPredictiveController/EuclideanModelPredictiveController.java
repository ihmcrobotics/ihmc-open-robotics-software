package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProviderTools;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowCalculator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPlaneForceViewer;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.LinearMPCTrajectoryViewer;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.MPCCornerPointViewer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLongTime;

public abstract class EuclideanModelPredictiveController
{
   private static final boolean includeVelocityObjective = true;
   private static final boolean includeRhoMinInequality = true;
   private static final boolean includeRhoMaxInequality = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected static final int numberOfBasisVectorsPerContactPoint = 4;
   private static final double minRhoValue = 0.0;//05;
   private final double maxContactForce;

   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   protected final DoubleProvider omega;
   protected final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private static final double mu = 0.8;

   public static final double initialComWeight = 5e3;
   public static final double vrpTrackingWeight = 1e2;

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   protected final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredVRPVelocity = new FrameVector3D(worldFrame);
   private final FixedFramePoint3DBasics desiredECMPPosition = new FramePoint3D(worldFrame);

   private final RecyclingArrayList<FramePoint3D> startVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> endVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);

   protected final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition", worldFrame, registry);
   protected final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);

   protected final YoDouble currentTimeInState = new YoDouble("currentTimeInState", registry);
   private final YoFramePoint3D comPositionAtEndOfWindow = new YoFramePoint3D("comPositionAtEndOfWindow", worldFrame, registry);
   private final YoFrameVector3D comVelocityAtEndOfWindow = new YoFrameVector3D("comVelocityAtEndOfWindow", worldFrame, registry);
   private final YoFramePoint3D dcmAtEndOfWindow = new YoFramePoint3D("dcmAtEndOfWindow", worldFrame, registry);
   private final YoFramePoint3D vrpAtEndOfWindow = new YoFramePoint3D("vrpAtEndOfWindow", worldFrame, registry);

   private final YoDouble initialCoMPositionCostToGo = new YoDouble("initialCoMPositionCostToGo", registry);
   private final YoDouble initialCoMVelocityCostToGo = new YoDouble("initialCoMVelocityCostToGo", registry);
   private final YoDouble vrpTrackingCostToGo0 = new YoDouble("vrpTrackingCostToGo0", registry);
   private final YoDouble vrpTrackingCostToGo1 = new YoDouble("vrpTrackingCostToGo1", registry);
   private final YoDouble vrpTrackingCostToGo2 = new YoDouble("vrpTrackingCostToGo2", registry);

   public final RecyclingArrayList<RecyclingArrayList<MPCContactPlane>> contactPlaneHelperPool;

   protected final PreviewWindowCalculator previewWindowCalculator;
   final LinearMPCTrajectoryHandler linearTrajectoryHandler;

   protected final CommandProvider commandProvider = new CommandProvider();
   final MPCCommandList mpcCommands = new MPCCommandList();

   private final ExecutionTimer mpcTotalTime = new ExecutionTimer("mpcTotalTime", registry);
   private final ExecutionTimer mpcAssemblyTime = new ExecutionTimer("mpcAssemblyTime", registry);
   private final ExecutionTimer mpcQPTime = new ExecutionTimer("mpcQPTime", registry);
   private final ExecutionTimer mpcExtractionTime = new ExecutionTimer("mpcExtractionTime", registry);

   private MPCCornerPointViewer cornerPointViewer = null;
   private LinearMPCTrajectoryViewer trajectoryViewer = null;

   private final DoubleConsumer initialComPositionConsumer = initialCoMPositionCostToGo::set;
   private final DoubleConsumer initialComVelocityConsumer = initialCoMVelocityCostToGo::set;
   private final DoubleConsumer vrpTrackingConsumer0 = vrpTrackingCostToGo0::set;
   private final DoubleConsumer vrpTrackingConsumer1 = vrpTrackingCostToGo1::set;
   private final DoubleConsumer vrpTrackingConsumer2 = vrpTrackingCostToGo2::set;

   public EuclideanModelPredictiveController(LinearMPCIndexHandler indexHandler,
                                             double gravityZ,
                                             double nominalCoMHeight,
                                             YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);
      this.omega = omega;

      previewWindowCalculator = new PreviewWindowCalculator(registry);
      linearTrajectoryHandler = new LinearMPCTrajectoryHandler(indexHandler, gravityZ, nominalCoMHeight, registry);

      this.maxContactForce = 2.0 * Math.abs(gravityZ);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      FrictionConeRotationCalculator coneRotationCalculator = new ZeroConeRotationCalculator();
      Supplier<MPCContactPlane> contactPlaneHelperProvider = () -> new MPCContactPlane(6, numberOfBasisVectorsPerContactPoint, coneRotationCalculator);
      contactPlaneHelperPool = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(contactPlaneHelperProvider));

      parentRegistry.addChild(registry);
   }

   public void setCornerPointViewer(MPCCornerPointViewer viewer)
   {
      cornerPointViewer = viewer;
   }

   public void setupCoMTrajectoryViewer(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      trajectoryViewer = new LinearMPCTrajectoryViewer(registry, yoGraphicsListRegistry);
   }

   public void setContactPlaneViewers(Supplier<ContactPlaneForceViewer> viewerSupplier)
   {
      contactPlaneHelperPool.clear();
      for (int i = 0; i < 2; i++)
      {
         RecyclingArrayList<MPCContactPlane> helpers = contactPlaneHelperPool.add();
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
      linearTrajectoryHandler.setNominalCoMHeight(nominalCoMHeight);
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

      mpcTotalTime.startMeasurement();
      mpcAssemblyTime.startMeasurement();
      previewWindowCalculator.compute(contactSequence, currentTimeInState.getDoubleValue());
      List<ContactPlaneProvider> planningWindow = previewWindowCalculator.getPlanningWindow();

      initializeIndexHandler();

      solveForTrajectoryOutsidePreviewWindow(contactSequence);

      setTerminalConditions();

      if (previewWindowCalculator.activeSegmentChanged())
      {
         resetActiveSet();
      }


      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                                    gravityZ,
                                                    omega.getValue(),
                                                    currentCoMVelocity,
                                                    planningWindow,
                                                    startVRPPositions,
                                                    endVRPPositions,
                                                    false);

      commandProvider.reset();
      mpcCommands.clear();

      computeMatrixHelpers(planningWindow);
      computeObjectives(planningWindow);

      mpcAssemblyTime.stopMeasurement();
      mpcQPTime.startMeasurement();
      DMatrixRMaj solutionCoefficients = solveQP();
      mpcQPTime.stopMeasurement();

      mpcExtractionTime.startMeasurement();
      if (solutionCoefficients != null)
      {
         extractSolution(solutionCoefficients);
      }

      if (cornerPointViewer != null)
         cornerPointViewer.updateCornerPoints(linearTrajectoryHandler, previewWindowCalculator.getFullPlanningSequence());

      if (trajectoryViewer != null)
      {
         updateCoMTrajectoryViewer();
      }
      mpcExtractionTime.stopMeasurement();
      mpcTotalTime.stopMeasurement();
   }

   protected abstract void initializeIndexHandler();

   protected void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> contactSequence)
   {
      List<ContactPlaneProvider> planningWindow = previewWindowCalculator.getPlanningWindow();

      linearTrajectoryHandler.solveForTrajectoryOutsidePreviewWindow(contactSequence);
      linearTrajectoryHandler.computeOutsidePreview(planningWindow.get(planningWindow.size() - 1).getTimeInterval().getEndTime());
   }

   protected void setTerminalConditions()
   {
      comPositionAtEndOfWindow.set(linearTrajectoryHandler.getDesiredCoMPositionOutsidePreview());
      comVelocityAtEndOfWindow.set(linearTrajectoryHandler.getDesiredCoMVelocityOutsidePreview());
      dcmAtEndOfWindow.set(linearTrajectoryHandler.getDesiredDCMPositionOutsidePreview());
      vrpAtEndOfWindow.set(linearTrajectoryHandler.getDesiredVRPPositionOutsidePreview());
   }

   protected void extractSolution(DMatrixRMaj solutionCoefficients)
   {
      List<ContactPlaneProvider> planningWindow = previewWindowCalculator.getPlanningWindow();
      linearTrajectoryHandler.extractSolutionForPreviewWindow(solutionCoefficients, planningWindow, contactPlaneHelperPool, previewWindowCalculator.getFullPlanningSequence(), omega.getValue());
   }

   protected void computeMatrixHelpers(List<ContactPlaneProvider> contactSequence)
   {
      contactPlaneHelperPool.clear();

      for (int sequenceId = 0; sequenceId < contactSequence.size(); sequenceId++)
      {
         ContactPlaneProvider contact = contactSequence.get(sequenceId);
         double duration = contact.getTimeInterval().getDuration();

         RecyclingArrayList<MPCContactPlane> contactPlaneHelpers = contactPlaneHelperPool.add();
         contactPlaneHelpers.clear();

         double objectiveForce = gravityZ / contact.getNumberOfContactPlanes();
         for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         {
            MPCContactPlane contactPlaneHelper = contactPlaneHelpers.add();
            contactPlaneHelper.setMaxNormalForce(maxContactForce);
            contactPlaneHelper.computeBasisVectors(contact.getContactsInBodyFrame(contactId), contact.getContactPose(contactId), mu);
            contactPlaneHelper.computeAccelerationIntegrationMatrix(duration, omega.getValue(), objectiveForce);
         }
      }
   }

   protected void computeObjectives(List<ContactPlaneProvider> contactSequence)
   {
      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      vrpTrackingCostToGo0.setToNaN();
      vrpTrackingCostToGo1.setToNaN();
      vrpTrackingCostToGo2.setToNaN();

      mpcCommands.addCommand(computeInitialCoMPositionObjective(commandProvider.getNextCoMPositionCommand()));
      if (includeVelocityObjective)
      {
         mpcCommands.addCommand(computeInitialCoMVelocityObjective(commandProvider.getNextCoMVelocityCommand()));
      }
      double initialDuration = contactSequence.get(0).getTimeInterval().getDuration();

      if (contactSequence.get(0).getContactState().isLoadBearing())
      {
         mpcCommands.addCommand(computeVRPTrackingObjective(commandProvider.getNextVRPTrackingCommand(),
                                                            startVRPPositions.get(0),
                                                            endVRPPositions.get(0),
                                                            0,
                                                            initialDuration,
                                                            vrpTrackingConsumer0));
         if (includeRhoMinInequality)
            mpcCommands.addCommand(computeMinForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), 0, 0.0));
         if (includeRhoMaxInequality)
            mpcCommands.addCommand(computeMaxForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), 0, 0.0));
      }

      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int nextSequence = transition + 1;

         double firstSegmentDuration = contactSequence.get(transition).getTimeInterval().getDuration();

         mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextComPositionContinuityCommand(), transition, firstSegmentDuration));
         mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextComVelocityContinuityCommand(), transition, firstSegmentDuration));

         if (contactSequence.get(transition).getContactState().isLoadBearing() && contactSequence.get(nextSequence).getContactState().isLoadBearing())
            mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextVRPPositionContinuityCommand(), transition, firstSegmentDuration));

         if (contactSequence.get(transition).getContactState().isLoadBearing())
         {
            if (includeRhoMinInequality)
               mpcCommands.addCommand(computeMinForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), transition, firstSegmentDuration));
            if (includeRhoMaxInequality)
               mpcCommands.addCommand(computeMaxForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), transition, firstSegmentDuration));
         }
         double nextDuration = Math.min(contactSequence.get(nextSequence).getTimeInterval().getDuration(), sufficientlyLongTime);

         if (contactSequence.get(nextSequence).getContactState().isLoadBearing())
         {
            DoubleConsumer costToGoConsumer = null;
            if (nextSequence == 1)
               costToGoConsumer = vrpTrackingConsumer1;
            else if (nextSequence == 2)
               costToGoConsumer = vrpTrackingConsumer2;
            mpcCommands.addCommand(computeVRPTrackingObjective(commandProvider.getNextVRPTrackingCommand(),
                                                               startVRPPositions.get(nextSequence),
                                                               endVRPPositions.get(nextSequence),
                                                               nextSequence,
                                                               nextDuration,
                                                               costToGoConsumer));
            if (includeRhoMinInequality)
               mpcCommands.addCommand(computeMinForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), nextSequence, 0.0));
            if (includeRhoMaxInequality)
               mpcCommands.addCommand(computeMaxForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), nextSequence, 0.0));
         }
      }

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequence.get(numberOfPhases - 1);
      double finalDuration = Math.min(lastContactPhase.getTimeInterval().getDuration(), sufficientlyLongTime);
      mpcCommands.addCommand(computeCoMPositionObjective(commandProvider.getNextCoMPositionCommand(),
                                                         comPositionAtEndOfWindow,
                                                         numberOfPhases - 1,
                                                         finalDuration));
      mpcCommands.addCommand(computeCoMVelocityObjective(commandProvider.getNextCoMVelocityCommand(),
                                                         comVelocityAtEndOfWindow,
                                                         numberOfPhases - 1,
                                                         finalDuration));

      //      mpcCommands.addCommand(computeDCMPositionObjective(commandProvider.getNextDCMPositionCommand(), dcmAtEndOfWindow, numberOfPhases - 1, finalDuration));
      mpcCommands.addCommand(computeVRPPositionObjective(commandProvider.getNextVRPPositionCommand(), vrpAtEndOfWindow, numberOfPhases - 1, finalDuration));
      if (includeRhoMinInequality)
         mpcCommands.addCommand(computeMinForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), numberOfPhases - 1, finalDuration));
      if (includeRhoMaxInequality)
         mpcCommands.addCommand(computeMaxForceObjective(commandProvider.getNextRhoAccelerationObjectiveCommand(), numberOfPhases - 1, finalDuration));
   }

   private MPCCommand<?> computeInitialCoMPositionObjective(CoMPositionCommand objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(initialComWeight);
      objectiveToPack.setConstraintType(ConstraintType.EQUALITY);
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(0.0);
      objectiveToPack.setObjective(currentCoMPosition);
      objectiveToPack.setCostToGoConsumer(initialComPositionConsumer);
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
      //      objectiveToPack.setConstraintType(ConstraintType.EQUALITY);
      objectiveToPack.setWeight(initialComWeight);
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(0.0);
      objectiveToPack.setObjective(currentCoMVelocity);
      objectiveToPack.setCostToGoConsumer(initialComVelocityConsumer);
      for (int i = 0; i < contactPlaneHelperPool.get(0).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(0).get(i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeContinuityObjective(MPCContinuityCommand continuityObjectiveToPack, int firstSegmentNumber, double firstSegmentDuration)
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

   private MPCCommand<?> computeMinForceObjective(RhoAccelerationObjectiveCommand valueObjective, int segmentNumber, double constraintTime)
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

   private MPCCommand<?> computeMaxForceObjective(RhoAccelerationObjectiveCommand valueObjective, int segmentNumber, double constraintTime)
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
         MPCContactPlane contactPlaneHelper = contactPlaneHelperPool.get(segmentNumber).get(i);
         objectiveSize += contactPlaneHelper.getRhoSize();
         valueObjective.addContactPlaneHelper(contactPlaneHelper);
      }

      int rowStart = 0;
      DMatrixRMaj objectiveVector = valueObjective.getObjectiveVector();
      objectiveVector.reshape(objectiveSize, 1);
      for (int i = 0; i < contactPlaneHelperPool.get(segmentNumber).size(); i++)
      {
         MPCContactPlane contactPlaneHelper = contactPlaneHelperPool.get(segmentNumber).get(i);
         MatrixTools.setMatrixBlock(objectiveVector, rowStart, 0, contactPlaneHelper.getRhoMaxMatrix(), 0, 0, contactPlaneHelper.getRhoSize(), 1, 1.0);

         rowStart += contactPlaneHelper.getRhoSize();
      }

      return valueObjective;
   }

   private MPCCommand<?> computeVRPTrackingObjective(VRPTrackingCommand objectiveToPack,
                                                     FramePoint3DReadOnly desiredStartVRPPosition,
                                                     FramePoint3DReadOnly desiredEndVRPPosition,
                                                     int segmentNumber,
                                                     double segmentDuration,
                                                     DoubleConsumer costToGoConsumer)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(vrpTrackingWeight);
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setSegmentDuration(segmentDuration);
      objectiveToPack.setStartVRP(desiredStartVRPPosition);
      objectiveToPack.setEndVRP(desiredEndVRPPosition);
      objectiveToPack.setCostToGoConsumer(costToGoConsumer);
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

   private MPCCommand<?> computeCoMPositionObjective(CoMPositionCommand objectiveToPack,
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

   private MPCCommand<?> computeCoMVelocityObjective(CoMVelocityCommand objectiveToPack,
                                                     FrameVector3DReadOnly desiredVelocity,
                                                     int segmentNumber,
                                                     double timeOfObjective)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeOfObjective(timeOfObjective);
      objectiveToPack.setObjective(desiredVelocity);
      objectiveToPack.setConstraintType(ConstraintType.EQUALITY);
      for (int i = 0; i < contactPlaneHelperPool.get(segmentNumber).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(segmentNumber).get(i));
      }

      return objectiveToPack;
   }


   private MPCCommand<?> computeVRPPositionObjective(VRPPositionCommand objectiveToPack,
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

   protected abstract void resetActiveSet();

   protected abstract DMatrixRMaj solveQP();

   public void compute(double timeInPhase)
   {
      compute(timeInPhase,
              desiredCoMPosition,
              desiredCoMVelocity,
              desiredCoMAcceleration,
              desiredDCMPosition,
              desiredDCMVelocity,
              desiredVRPPosition,
              desiredVRPVelocity,
              desiredECMPPosition);
   }

   protected void updateCoMTrajectoryViewer()
   {
      trajectoryViewer.compute(this, currentTimeInState.getDoubleValue());
   }

   public void compute(double timeInPhase,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFrameVector3DBasics vrpVelocityToPack,
                       FixedFramePoint3DBasics ecmpPositionToPack)
   {
      linearTrajectoryHandler.compute(timeInPhase);

      comPositionToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredCoMPosition());
      comVelocityToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredCoMVelocity());
      comAccelerationToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredCoMAcceleration());
      dcmPositionToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredDCMPosition());
      dcmVelocityToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredDCMVelocity());
      vrpPositionToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredVRPPosition());
      vrpVelocityToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredVRPVelocity());

      ecmpPositionToPack.setMatchingFrame(vrpPositionToPack);
      double nominalHeight = gravityZ / MathTools.square(omega.getValue());
      ecmpPositionToPack.set(desiredVRPPosition);
      ecmpPositionToPack.subZ(nominalHeight);
   }

   public int getCurrentSegmentIndex()
   {
      return linearTrajectoryHandler.getComTrajectory().getCurrentSegmentIndex();
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      linearTrajectoryHandler.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void setCurrentCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition,
                                           FrameVector3DReadOnly centerOfMassVelocity,
                                           double timeInState)
   {
      this.currentCoMPosition.setMatchingFrame(centerOfMassPosition);
      this.currentCoMVelocity.setMatchingFrame(centerOfMassVelocity);
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

   public List<? extends Polynomial3DReadOnly> getVRPTrajectories()
   {
      return linearTrajectoryHandler.getVrpTrajectories();
   }

   public List<ContactPlaneProvider> getContactStateProviders()
   {
      return linearTrajectoryHandler.getFullPlanningSequence();
   }
}
