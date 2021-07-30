package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPlaneForceViewer;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.LinearMPCTrajectoryViewer;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.MPCCornerPointViewer;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.IntUnaryOperator;
import java.util.function.Supplier;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLongTime;

public abstract class EuclideanModelPredictiveController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean debug = true;
   private static final boolean useSlackVariablesForRhoBounds = true;
   private static final double firstSegmentSlackWeight = 1e5;

   protected static final int numberOfBasisVectorsPerContactPoint = 4;
   private final double maxContactForce;

   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixRMaj solutionCoefficients = new DMatrixRMaj(0, 0);
   private final double mass;
   protected final DoubleProvider omega;
   protected final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private static final double mu = 0.8;

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   protected final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredVRPVelocity = new FrameVector3D(worldFrame);
   private final FixedFramePoint3DBasics desiredECMPPosition = new FramePoint3D(worldFrame);

   private final RecyclingArrayList<RecyclingArrayList<FramePoint3D>> startVRPPositions = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(FramePoint3D::new));
   private final RecyclingArrayList<RecyclingArrayList<FrameVector3D>> startVRPVelocities = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(FrameVector3D::new));
   private final RecyclingArrayList<RecyclingArrayList<FramePoint3D>> endVRPPositions = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(FramePoint3D::new));
   private final RecyclingArrayList<RecyclingArrayList<FrameVector3D>> endVRPVelocities = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(FrameVector3D::new));

   protected final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition", worldFrame, registry);
   protected final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);

   protected final YoDouble currentTimeInState = new YoDouble("currentTimeInState", registry);
   protected final YoFramePoint3D comPositionAtEndOfWindow = new YoFramePoint3D("comPositionAtEndOfWindow", worldFrame, registry);
   protected final YoFrameVector3D comVelocityAtEndOfWindow = new YoFrameVector3D("comVelocityAtEndOfWindow", worldFrame, registry);
   protected final YoFramePoint3D dcmAtEndOfWindow = new YoFramePoint3D("dcmAtEndOfWindow", worldFrame, registry);
   private final YoFramePoint3D vrpAtEndOfWindow = new YoFramePoint3D("vrpAtEndOfWindow", worldFrame, registry);

   protected final MPCParameters mpcParameters;
   protected final MPCContactHandler contactHandler;

   protected final PreviewWindowCalculator previewWindowCalculator;
   final LinearMPCTrajectoryHandler linearTrajectoryHandler;
   protected final WrenchMPCTrajectoryHandler wrenchTrajectoryHandler;

   private final LinearMPCIndexHandler indexHandler;

   protected final CommandProvider commandProvider = new CommandProvider();
   final MPCCommandList mpcCommands = new MPCCommandList();

   private final ExecutionTimer mpcTotalTime = new ExecutionTimer("mpcTotalTime", registry);
   private final ExecutionTimer mpcAssemblyTime = new ExecutionTimer("mpcAssemblyTime", registry);
   private final ExecutionTimer mpcQPTime = new ExecutionTimer("mpcQPTime", registry);
   private final ExecutionTimer mpcExtractionTime = new ExecutionTimer("mpcExtractionTime", registry);

   protected final YoBoolean useWarmStart = new YoBoolean("mpcUseWarmStart", registry);

   protected final DMatrixRMaj previousSolution = new DMatrixRMaj(1, 1);
   protected final TIntArrayList activeInequalityConstraints = new TIntArrayList();

   private MPCCornerPointViewer cornerPointViewer = null;
   private LinearMPCTrajectoryViewer trajectoryViewer = null;

   public EuclideanModelPredictiveController(LinearMPCIndexHandler indexHandler,
                                             MPCParameters mpcParameters,
                                             double mass,
                                             double gravityZ,
                                             double nominalCoMHeight,
                                             YoRegistry parentRegistry)
   {
      this.mpcParameters = mpcParameters;
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);
      this.mass = mass;
      this.omega = omega;
      this.indexHandler = indexHandler;

      this.maxContactForce = 2.0 * Math.abs(gravityZ);

      previewWindowCalculator = new PreviewWindowCalculator(registry);
      linearTrajectoryHandler = new LinearMPCTrajectoryHandler(indexHandler, gravityZ, nominalCoMHeight, registry);
      wrenchTrajectoryHandler = new WrenchMPCTrajectoryHandler(registry);
      contactHandler = new MPCContactHandler(indexHandler, gravityZ, mass);


      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      useWarmStart.set(true);

      parentRegistry.addChild(registry);
   }

   public void setCornerPointViewer(MPCCornerPointViewer viewer)
   {
      cornerPointViewer = viewer;
   }

   public void setupCoMTrajectoryViewer(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      trajectoryViewer = new LinearMPCTrajectoryViewer(registry, yoGraphicsListRegistry);
      YoGraphicPosition previewEndPosition = new YoGraphicPosition("Preview End CoM Position", comPositionAtEndOfWindow, 0.02, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicVector previewEndVelocity = new YoGraphicVector("Preview End CoM Velocity", comPositionAtEndOfWindow, comVelocityAtEndOfWindow, 0.05, YoAppearance.Red());

      yoGraphicsListRegistry.registerYoGraphic("End Of preview Window", previewEndPosition);
      yoGraphicsListRegistry.registerYoGraphic("End Of preview Window", previewEndVelocity);
   }

   public void setContactPlaneViewers(Supplier<ContactPlaneForceViewer> viewerSupplier)
   {
      contactHandler.setContactPlaneViewers(viewerSupplier);
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
      List<PreviewWindowSegment> planningWindow = previewWindowCalculator.getPlanningWindow();

      initializeIndexHandler();

      solveForTrajectoryOutsidePreviewWindow(contactSequence);

      setTerminalConditions();

      if (previewWindowCalculator.activeSegmentChanged())
      {
         resetActiveSet();
      }


      MPCTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                   gravityZ,
                                   omega.getValue(),
                                   currentCoMVelocity,
                                   planningWindow,
                                   startVRPPositions,
                                   endVRPPositions,
                                   false);
      MPCTools.computeVRPVelocites(planningWindow, startVRPVelocities, endVRPVelocities);

      commandProvider.reset();
      mpcCommands.clear();

      contactHandler.computeMatrixHelpers(planningWindow, linearTrajectoryHandler.getPlanningWindowForSolution(), omega.getValue());
      computeObjectives(planningWindow);

      mpcAssemblyTime.stopMeasurement();
      mpcQPTime.startMeasurement();
      NativeMatrix solutionCoefficients = solveQP();
      mpcQPTime.stopMeasurement();

      mpcExtractionTime.startMeasurement();
      if (solutionCoefficients != null)
      {
         this.solutionCoefficients.reshape(indexHandler.getTotalProblemSize(), 1);
         solutionCoefficients.get(this.solutionCoefficients);
         extractSolution(this.solutionCoefficients);

         if (debug)
         {
            linearTrajectoryHandler.compute(currentTimeInState.getDoubleValue() + previewWindowCalculator.getPreviewWindowDuration());
//            if (mpcParameters.getFinalCoMPositionConstraintType() == ConstraintType.EQUALITY && !linearTrajectoryHandler.getDesiredCoMPosition().epsilonEquals(comPositionAtEndOfWindow, 1e-4))
//               LogTools.error("CoM at the end of the preview window  is " + linearTrajectoryHandler.getDesiredCoMPosition() + " but should be " + comPositionAtEndOfWindow);
         }
      }

//      if (cornerPointViewer != null)
//         cornerPointViewer.updateCornerPoints(linearTrajectoryHandler, previewWindowCalculator.getFullPlanningSequence());

      updateCoMTrajectoryViewer();


      mpcExtractionTime.stopMeasurement();
      mpcTotalTime.stopMeasurement();
   }

   protected void assembleActiveSet(IntUnaryOperator startIndexGetter)
   {
      activeInequalityConstraints.reset();
      previousSolution.reshape(indexHandler.getTotalProblemSize(), 1);

      int inequalityStartIndex = 0;
      int lowerBoundStartIndex = 0;
      int upperBoundStartIndex = 0;

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         ActiveSetData activeSetData = contactHandler.getActiveSetData(segmentId);

         MatrixTools.setMatrixBlock(previousSolution,
                                    startIndexGetter.applyAsInt(segmentId),
                                    0,
                                    activeSetData.getPreviousSolution(),
                                    0,
                                    0,
                                    indexHandler.getVariablesInSegment(segmentId),
                                    1,
                                    1.0);

         for (int i = 0; i < activeSetData.getNumberOfActiveInequalityConstraints(); i++)
         {
            activeInequalityConstraints.add(activeSetData.getActiveInequalityIndex(i) + inequalityStartIndex);
         }

         inequalityStartIndex += activeSetData.getNumberOfInequalityConstraints();
      }
   }

   protected void extractNewActiveSetData(boolean foundSolution, LinearMPCQPSolver qpSolver, IntUnaryOperator startIndexGetter)
   {
      TIntList activeInequalityIndices = qpSolver.getActiveInequalityIndices();

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         ActiveSetData activeSetData = contactHandler.getActiveSetData(segmentId);
         activeSetData.clearActiveSet();

         if (foundSolution)
         {
            MatrixTools.setMatrixBlock(activeSetData.getPreviousSolution(),
                                       0,
                                       0,
                                       qpSolver.getSolution(),
                                       startIndexGetter.applyAsInt(segmentId),
                                       0,
                                       indexHandler.getVariablesInSegment(segmentId),
                                       1,
                                       1.0);
         }
      }

      for (int i = 0; i < activeInequalityIndices.size(); i++)
      {
         assignActiveConstraintIndexToCorrectSegment(contactHandler, activeInequalityIndices.get(i));
      }
   }

   private int assignActiveConstraintIndexToCorrectSegment(MPCContactHandler contactHandler, int activeConstraintIndex)
   {
      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         ActiveSetData activeSetData = contactHandler.getActiveSetData(segmentId);
         if (activeConstraintIndex < activeSetData.getNumberOfInequalityConstraints())
         {
            activeSetData.addActiveInequalityConstraint(activeConstraintIndex);
            return segmentId;
         }
         else
            activeConstraintIndex -= activeSetData.getNumberOfActiveInequalityConstraints();
      }

      return -1;
   }

   protected abstract void initializeIndexHandler();

   protected void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> contactSequence)
   {
      List<PreviewWindowSegment> planningWindow = previewWindowCalculator.getPlanningWindow();

      linearTrajectoryHandler.solveForTrajectoryOutsidePreviewWindow(contactSequence);
      linearTrajectoryHandler.computeOutsidePreview(planningWindow.get(planningWindow.size() - 1).getEndTime());
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
      List<PreviewWindowSegment> planningWindow = previewWindowCalculator.getPlanningWindow();
      linearTrajectoryHandler.extractSolutionForPreviewWindow(solutionCoefficients,
                                                              planningWindow,
                                                              contactHandler.getContactPlanes(),
                                                              previewWindowCalculator.getFullPlanningSequence(),
                                                              omega.getValue());
      wrenchTrajectoryHandler.extractSolutionForPreviewWindow(planningWindow, contactHandler.getContactPlanes(), mass, omega.getValue());
   }

   protected void computeObjectives(List<PreviewWindowSegment> contactSequence)
   {
      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      for (int i = 0; i < numberOfPhases; i++)
         contactHandler.getActiveSetData(i).resetConstraintCounter();

      computeInitialPhaseObjectives();
      computeObjectivesForCurrentPhase(contactSequence.get(0), 0);

      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int nextSequence = transition + 1;

         computeTransitionObjectives(contactSequence.get(transition), contactSequence.get(nextSequence), transition);
         computeObjectivesForCurrentPhase(contactSequence.get(nextSequence), nextSequence);
      }

      // set terminal constraint
      computeFinalPhaseObjectives(contactSequence.get(numberOfPhases - 1), numberOfPhases - 1);
   }

   protected void computeInitialPhaseObjectives()
   {
      mpcCommands.addCommand(computeInitialCoMPositionObjective(commandProvider.getNextCoMPositionCommand()));
      if (mpcParameters.includeInitialCoMVelocityObjective())
      {
         mpcCommands.addCommand(computeInitialCoMVelocityObjective(commandProvider.getNextCoMVelocityCommand()));
      }
   }

   protected void computeTransitionObjectives(PreviewWindowSegment currentContact, PreviewWindowSegment nextContact, int currentSegmentNumber)
   {
      double firstSegmentDuration = currentContact.getDuration();

      mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextComPositionContinuityCommand(), currentSegmentNumber, firstSegmentDuration));
      mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextComVelocityContinuityCommand(), currentSegmentNumber, firstSegmentDuration));

      if (currentContact.getContactState().isLoadBearing() && nextContact.getContactState().isLoadBearing())
         mpcCommands.addCommand(computeContinuityObjective(commandProvider.getNextVRPPositionContinuityCommand(), currentSegmentNumber, firstSegmentDuration));
   }

   protected void computeObjectivesForCurrentPhase(PreviewWindowSegment contactPlaneProvider, int segmentNumber)
   {
      if (!contactPlaneProvider.getContactState().isLoadBearing())
         return;

      double segmentDuration = contactPlaneProvider.getDuration();
      double startTime = contactPlaneProvider.getStartTime();

      for (int phaseNumber = 0; phaseNumber < contactPlaneProvider.getNumberOfContactPhasesInSegment(); phaseNumber++)
      {
         TimeIntervalReadOnly timeInterval = contactPlaneProvider.getTimeInterval(phaseNumber);

         mpcCommands.addCommand(computeVRPTrackingObjective(commandProvider.getNextVRPTrackingCommand(),
                                                            startVRPPositions.get(segmentNumber).get(phaseNumber),
                                                            startVRPVelocities.get(segmentNumber).get(phaseNumber),
                                                            endVRPPositions.get(segmentNumber).get(phaseNumber),
                                                            endVRPVelocities.get(segmentNumber).get(phaseNumber),
                                                            segmentNumber,
                                                            timeInterval.getStartTime() - startTime,
                                                            timeInterval.getEndTime() - startTime,
                                                            null));
      }
      if (mpcParameters.includeForceMinimization())
      {
         for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
            mpcCommands.addCommand(computeForceMinimizationObjective(commandProvider.getForceTrackingCommand(), segmentNumber, segmentDuration, i));
      }
      if (mpcParameters.includeRhoMinimization())
         mpcCommands.addCommand(computeRhoMinimizationObjective(commandProvider.getRhoMinimizationCommand(), segmentNumber, segmentDuration));
      if (mpcParameters.includeRhoRateMinimization())
         mpcCommands.addCommand(computeRhoRateMinimizationObjective(commandProvider.getRhoRateMinimizationCommand(), segmentNumber, segmentDuration));
      if (mpcParameters.includeRhoMinInequality())
         mpcCommands.addCommand(computeMinForceObjective(commandProvider.getNextRhoBoundCommand(), segmentNumber, segmentDuration));
      if (mpcParameters.includeRhoMaxInequality())
         mpcCommands.addCommand(computeMaxForceObjective(commandProvider.getNextNormalForceBoundCommand(), segmentNumber, segmentDuration));
   }

   protected void computeFinalPhaseObjectives(PreviewWindowSegment lastContactPhase, int segmentNumber)
   {
      double finalDuration = Math.min(lastContactPhase.getDuration(), sufficientlyLongTime);
      if (mpcParameters.includeFinalCoMPositionObjective())
      {
         mpcCommands.addCommand(computeFinalCoMPositionObjective(commandProvider.getNextCoMPositionCommand(),
                                                                 comPositionAtEndOfWindow,
                                                                 segmentNumber,
                                                                 finalDuration));
      }
      if (mpcParameters.includeFinalCoMVelocityObjective())
      {
         mpcCommands.addCommand(computeFinalCoMVelocityObjective(commandProvider.getNextCoMVelocityCommand(),
                                                                 comVelocityAtEndOfWindow,
                                                                 segmentNumber,
                                                                 finalDuration));
      }
      if (mpcParameters.includeFinalDCMPositionObjective())
      {
         mpcCommands.addCommand(computeFinalDCMPositionObjective(commandProvider.getNextDCMPositionCommand(), dcmAtEndOfWindow, segmentNumber, finalDuration));
      }
      if (lastContactPhase.getContactState().isLoadBearing())
         mpcCommands.addCommand(computeVRPPositionObjective(commandProvider.getNextVRPPositionCommand(), vrpAtEndOfWindow, segmentNumber, finalDuration));
   }

   private MPCCommand<?> computeInitialCoMPositionObjective(CoMPositionCommand objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(mpcParameters.getInitialComWeight());
      objectiveToPack.setConstraintType(mpcParameters.getInitialCoMPositionConstraintType());
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(0.0);
      objectiveToPack.setObjective(currentCoMPosition);
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(0); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(0, i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeInitialCoMVelocityObjective(CoMVelocityCommand objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setConstraintType(mpcParameters.getInitialCoMVelocityConstraintType());
      objectiveToPack.setWeight(mpcParameters.getInitialComVelocityWeight());
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setTimeOfObjective(0.0);
      objectiveToPack.setObjective(currentCoMVelocity);
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(0); i++)
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(0, i));

      return objectiveToPack;
   }

   private MPCCommand<?> computeContinuityObjective(MPCContinuityCommand continuityObjectiveToPack, int firstSegmentNumber, double firstSegmentDuration)
   {
      continuityObjectiveToPack.clear();
      continuityObjectiveToPack.setOmega(omega.getValue());
      continuityObjectiveToPack.setFirstSegmentNumber(firstSegmentNumber);
      continuityObjectiveToPack.setFirstSegmentDuration(firstSegmentDuration);
      continuityObjectiveToPack.setConstraintType(ConstraintType.EQUALITY);

      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(firstSegmentNumber); i++)
         continuityObjectiveToPack.addFirstSegmentContactPlaneHelper(contactHandler.getContactPlane(firstSegmentNumber, i));

      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(firstSegmentNumber + 1); i++)
         continuityObjectiveToPack.addSecondSegmentContactPlaneHelper(contactHandler.getContactPlane(firstSegmentNumber + 1, i));

      return continuityObjectiveToPack;
   }

   private MPCCommand<?> computeMinForceObjective(RhoBoundCommand valueObjective, int segmentNumber, double segmentDuration)
   {
      int numberOfContactPlanes = contactHandler.getNumberOfContactPlanesInSegment(segmentNumber);
      if (numberOfContactPlanes < 1)
         return null;

      valueObjective.clear();
      if (useSlackVariablesForRhoBounds)
         valueObjective.setSlackVariableWeight(firstSegmentSlackWeight / MathTools.pow(10.0, segmentNumber));
      valueObjective.setOmega(omega.getValue());
      valueObjective.setSegmentDuration(segmentDuration);
      valueObjective.setSegmentNumber(segmentNumber);
      valueObjective.setConstraintType(ConstraintType.GEQ_INEQUALITY);
      for (int i = 0; i < numberOfContactPlanes; i++)
      {
         MPCContactPlane contactPlane = contactHandler.getContactPlane(segmentNumber, i);
         valueObjective.addContactPlane(contactPlane, mpcParameters.getMinRhoValue());
         contactHandler.getActiveSetData(segmentNumber).addInequalityConstraints(4 * contactPlane.getRhoSize());
      }

      return valueObjective;
   }

   private MPCCommand<?> computeMaxForceObjective(NormalForceBoundCommand valueObjective, int segmentNumber, double segmentDuration)
   {
      valueObjective.clear();
      valueObjective.setOmega(omega.getValue());
      valueObjective.setSegmentDuration(segmentDuration);
      valueObjective.setSegmentNumber(segmentNumber);
      valueObjective.setConstraintType(ConstraintType.LEQ_INEQUALITY);

      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
      {
         MPCContactPlane contactPlane = contactHandler.getContactPlane(segmentNumber, i);
         valueObjective.addContactPlane(contactPlane, maxContactForce);
         // FIXME This is wrong
         contactHandler.getActiveSetData(segmentNumber).addInequalityConstraints(contactPlane.getRhoSize());
      }

      return valueObjective;
   }

   private MPCCommand<?> computeVRPTrackingObjective(VRPTrackingCommand objectiveToPack,
                                                     FramePoint3DReadOnly desiredStartVRPPosition,
                                                     FrameVector3DReadOnly desiredStartVRPVelocity,
                                                     FramePoint3DReadOnly desiredEndVRPPosition,
                                                     FrameVector3DReadOnly desiredEndVRPVelocity,
                                                     int segmentNumber,
                                                     double startTime,
                                                     double endTime,
                                                     DoubleConsumer costToGoConsumer)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(mpcParameters.getVRPTrackingWeight());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeInterval(startTime, endTime);
      objectiveToPack.setStartVRP(desiredStartVRPPosition);
      objectiveToPack.setStartVRPVelocity(desiredStartVRPVelocity);
      objectiveToPack.setEndVRP(desiredEndVRPPosition);
      objectiveToPack.setEndVRPVelocity(desiredEndVRPVelocity);
      objectiveToPack.setCostToGoConsumer(costToGoConsumer);
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, i));

      return objectiveToPack;
   }

   private final FrameVector3DReadOnly zeroVector = new FrameVector3D();
   private final FrameVector3D gravityVector = new FrameVector3D();

   private MPCCommand<?> computeForceMinimizationObjective(ForceTrackingCommand objectiveToPack, int segmentNumber, double segmentDuration, int contactNumber)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(mpcParameters.getForceTrackingWeight());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setSegmentDuration(segmentDuration);
      objectiveToPack.setObjectiveValue(zeroVector);
      objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, contactNumber));

      return objectiveToPack;
   }

   private MPCCommand<?> computeRhoMinimizationObjective(RhoTrackingCommand objectiveToPack, int segmentNumber, double segmentDuration)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(mpcParameters.getRhoTrackingWeight());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setSegmentDuration(segmentDuration);
      gravityVector.setZ(-gravityZ * mass);
      gravityVector.scale(1.0 / contactHandler.getNumberOfContactPlanesInSegment(segmentNumber));
      int numberOfRhos = 0;
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
      {
         MPCContactPlane contactPlane = contactHandler.getContactPlane(segmentNumber, i);
         objectiveToPack.addContactPlaneHelper(contactPlane);
         numberOfRhos += contactPlane.getRhoSize();
      }
      gravityVector.scale(1.0 / numberOfRhos);
      objectiveToPack.setObjectiveValue(Math.abs(gravityVector.getZ()) / mu);


      return objectiveToPack;
   }


   private MPCCommand<?> computeRhoRateMinimizationObjective(RhoRateTrackingCommand objectiveToPack, int segmentNumber, double segmentDuration)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setWeight(mpcParameters.getRhoRateTrackingWeight());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setSegmentDuration(segmentDuration);
      objectiveToPack.setObjectiveValue(0.0);
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, i));
      }

      return objectiveToPack;
   }

   private MPCCommand<?> computeFinalDCMPositionObjective(DCMPositionCommand objectiveToPack,
                                                          FramePoint3DReadOnly desiredPosition,
                                                          int segmentNumber,
                                                          double timeOfObjective)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeOfObjective(timeOfObjective);
      objectiveToPack.setObjective(desiredPosition);
      objectiveToPack.setConstraintType(mpcParameters.getFinalDCMPositionConstraintType());
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, i));

      return objectiveToPack;
   }

   private MPCCommand<?> computeFinalCoMPositionObjective(CoMPositionCommand objectiveToPack,
                                                          FramePoint3DReadOnly desiredPosition,
                                                          int segmentNumber,
                                                          double timeOfObjective)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeOfObjective(timeOfObjective);
      objectiveToPack.setObjective(desiredPosition);
      objectiveToPack.setWeight(mpcParameters.getFinalComWeight());
      objectiveToPack.setConstraintType(mpcParameters.getFinalCoMPositionConstraintType());
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, i));

      return objectiveToPack;
   }

   private MPCCommand<?> computeFinalCoMVelocityObjective(CoMVelocityCommand objectiveToPack,
                                                          FrameVector3DReadOnly desiredVelocity,
                                                          int segmentNumber,
                                                          double timeOfObjective)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(segmentNumber);
      objectiveToPack.setTimeOfObjective(timeOfObjective);
      objectiveToPack.setObjective(desiredVelocity);
      objectiveToPack.setWeight(mpcParameters.getFinalComWeight());
      objectiveToPack.setConstraintType(mpcParameters.getFinalCoMVelocityConstraintType());
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, i));

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
      objectiveToPack.setWeight(mpcParameters.getFinalVRPWeight());
      objectiveToPack.setConstraintType(mpcParameters.getFinalVRPPositionConstraintType());
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
         objectiveToPack.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, i));

      return objectiveToPack;
   }

   protected abstract void resetActiveSet();

   protected abstract NativeMatrix solveQP();

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
      if (trajectoryViewer != null)
         trajectoryViewer.compute(this, currentTimeInState.getDoubleValue(), previewWindowCalculator.getPreviewWindowDuration());
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
      linearTrajectoryHandler.computeOutsidePreview(timeInPhase);
      wrenchTrajectoryHandler.compute(timeInPhase);

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

   public FramePoint3DReadOnly getReferenceCoMPosition()
   {
      return linearTrajectoryHandler.getDesiredCoMPositionOutsidePreview();
   }

   public FrameVector3DReadOnly getReferenceCoMVelocity()
   {
      return linearTrajectoryHandler.getDesiredCoMVelocityOutsidePreview();
   }

   public FramePoint3DReadOnly getReferenceDCMPosition()
   {
      return linearTrajectoryHandler.getDesiredDCMPositionOutsidePreview();
   }

   public FramePoint3DReadOnly getReferenceVRPPosition()
   {
      return linearTrajectoryHandler.getDesiredVRPPositionOutsidePreview();
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

   public List<PreviewWindowSegment> getContactStateProviders()
   {
      return linearTrajectoryHandler.getFullPlanningSequence();
   }

   public boolean hasTrajectories()
   {
      return linearTrajectoryHandler.hasTrajectory() && wrenchTrajectoryHandler.hasTrajectory();
   }

   public void reset()
   {
      linearTrajectoryHandler.clearTrajectory();
      wrenchTrajectoryHandler.clearTrajectory();
   }

   public MultipleCoMSegmentTrajectoryGenerator getCoMTrajectory()
   {
      if (!hasTrajectories())
         throw new RuntimeException("CoM Trajectories are not calculated");

      return linearTrajectoryHandler.getComTrajectory();
   }

   public List<? extends List<MPCContactPlane>> getContactPlanes()
   {
      return contactHandler.getContactPlanes();
   }
}
