package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class ICPPlanner
{
   private static final boolean VISUALIZE = true;
   private static final double ICP_CORNER_POINT_SIZE = 0.004;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "icpPlanner";

   private final BooleanYoVariable isStanding = new BooleanYoVariable(namePrefix + "IsStanding", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable(namePrefix + "IsInitialTransfer", registry);
   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable(namePrefix + "IsDoubleSupport", registry);
   private final BooleanYoVariable hasBeenWokenUp = new BooleanYoVariable(namePrefix + "HasBeenWokenUp", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(namePrefix + "TimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable(namePrefix + "isDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(namePrefix + "DoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(namePrefix + "SingleSupportTime", registry);
   private final DoubleYoVariable minSingleSupportDuration = new DoubleYoVariable(namePrefix + "MinSingleSupportTime", registry);
   private final DoubleYoVariable timeCorrectionFactor = new DoubleYoVariable(namePrefix + "TimeCorrectionFactor", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable(namePrefix + "InitialTransferDuration", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable(namePrefix + "DoubleSupportSplitFraction", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable(namePrefix + "RemainingTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable(namePrefix + "NumberFootstepsToConsider", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable(namePrefix + "NumberFootstepsToStop", registry);

   private final YoFramePointInMultipleFrames singleSupportInitialICP;
   private final YoFrameVector singleSupportInitialICPVelocity = new YoFrameVector(namePrefix + "SingleSupportInitialICPVelocity", worldFrame, registry);

   private final YoFramePointInMultipleFrames singleSupportFinalICP;
   private final YoFrameVector singleSupportFinalICPVelocity = new YoFrameVector(namePrefix + "SingleSupportFinalICPVelocity", worldFrame, registry);

   private final YoFramePoint desiredCentroidalMomentumPivotPosition = new YoFramePoint(namePrefix + "DesiredCentroidalMomentumPosition", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint(namePrefix + "DesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector(namePrefix + "DesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector(namePrefix + "DesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable(namePrefix + "Omega0", registry);

   private final BooleanYoVariable useTwoConstantCMPsPerSupport = new BooleanYoVariable(namePrefix + "UseTwoConstantCMPsPerSupport", registry);
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable(namePrefix + "TimeSpentOnExitCMPInPercentOfStepTime", registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   private final ArrayList<YoFramePointInMultipleFrames> entryCornerPoints = new ArrayList<YoFramePointInMultipleFrames>();
   private final ArrayList<YoFramePointInMultipleFrames> exitCornerPoints = new ArrayList<YoFramePointInMultipleFrames>();

   private final ICPPlannerTrajectoryGenerator icpTrajectoryGenerator;
   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final FramePoint tempConstantCMP = new FramePoint();
   private final FramePoint tempICP = new FramePoint();

   public ICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, CapturePointPlannerParameters icpPlannerParameters, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      isStanding.set(true);
      hasBeenWokenUp.set(false);

      int numberOfCoefficients = icpPlannerParameters.getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory();
      icpTrajectoryGenerator = new ICPPlannerTrajectoryGenerator(namePrefix + "Trajectory", numberOfCoefficients, registry);
      doubleSupportInitialTransferDuration.set(icpPlannerParameters.getDoubleSupportInitialTransferDuration());
      numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());
      footstepsToStop.set(icpPlannerParameters.getNumberOfFootstepsToStop());
      isDoneTimeThreshold.set(icpPlannerParameters.getIsDoneTimeThreshold());
      doubleSupportSplitFraction.set(icpPlannerParameters.getDoubleSupportSplitFraction());
      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      useTwoConstantCMPsPerSupport.set(icpPlannerParameters.useTwoCMPsPerSupport());
      
      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      minSingleSupportDuration.set(0.3); // TODO Need to be extracted
      timeCorrectionFactor.set(0.5); // TODO Need to be extracted

      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet, numberFootstepsToConsider.getIntegerValue(), registry);
      double cmpForwardOffset = icpPlannerParameters.getReferenceCMPForwardOffset();
      double cmpInsideOffset = icpPlannerParameters.getReferenceCMPInsideOffset();
      referenceCMPsCalculator.setSymmetricCMPConstantOffsets(cmpForwardOffset, cmpInsideOffset);
      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.setMinMaxForwardCMPLocationFromFootCenter(icpPlannerParameters.getMinReferenceCMPForwardOffset(), icpPlannerParameters.getMaxReferenceCMPForwardOffset());
      referenceCMPsCalculator.setSafeDistanceFromSupportEdges(icpPlannerParameters.getCMPSafeDistanceAwayFromSupportEdges());

      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.put(robotSide, contactableFeet.get(robotSide).getSoleFrame());
      }


      singleSupportInitialICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportInitialICP", registry, worldFrame, soleFrames.get(RobotSide.LEFT), soleFrames.get(RobotSide.RIGHT));
      singleSupportFinalICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportFinalICP", registry, worldFrame, soleFrames.get(RobotSide.LEFT), soleFrames.get(RobotSide.RIGHT));

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePointInMultipleFrames earlyCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "EntryCornerPoints" + i, registry, worldFrame, soleFrames.get(RobotSide.LEFT), soleFrames.get(RobotSide.RIGHT));
         entryCornerPoints.add(earlyCornerPoint);

         YoFramePointInMultipleFrames lateCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "ExitCornerPoints" + i, registry, worldFrame, soleFrames.get(RobotSide.LEFT), soleFrames.get(RobotSide.RIGHT));
         exitCornerPoints.add(lateCornerPoint);
      }

      parentRegistry.addChild(registry);

      if (VISUALIZE && yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("ICPPlanner");
      ArtifactList artifactList = new ArtifactList("ICPPlanner");

      referenceCMPsCalculator.createVisualizerForConstantCMPs(yoGraphicsList, artifactList);

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoGraphicPosition icpEarlyCornerPointsViz = new YoGraphicPosition("EntryCornerPoints" + i, entryCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly(), ICP_CORNER_POINT_SIZE, YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpEarlyCornerPointsViz);
         artifactList.add(icpEarlyCornerPointsViz.createArtifact());

         YoGraphicPosition lateCornerPointsViz = new YoGraphicPosition("ExitCornerPoints" + i, exitCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly(), ICP_CORNER_POINT_SIZE, YoAppearance.Blue(), GraphicType.BALL);

         yoGraphicsList.add(lateCornerPointsViz);
         artifactList.add(lateCornerPointsViz.createArtifact());
      }

      YoGraphicPosition singleSupportInitialICPViz = new YoGraphicPosition("singleSupportInitialICP", singleSupportInitialICP.buildUpdatedYoFramePointForVisualizationOnly(), 0.004, YoAppearance.Chocolate(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(singleSupportInitialICPViz);
      artifactList.add(singleSupportInitialICPViz.createArtifact());

      YoGraphicPosition singleSupportFinalICPViz = new YoGraphicPosition("singleSupportFinalICP", singleSupportFinalICP.buildUpdatedYoFramePointForVisualizationOnly(), 0.004, YoAppearance.Chocolate(), GraphicType.BALL);
      yoGraphicsList.add(singleSupportFinalICPViz);
      artifactList.add(singleSupportFinalICPViz.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearPlan()
   {
      referenceCMPsCalculator.clear();
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      referenceCMPsCalculator.addUpcomingFootstep(footstep);
   }

   public void setDesiredCapturePointState(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity)
   {
      desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);
   }

   public void setDesiredCapturePointState(YoFramePoint currentDesiredCapturePointPosition, YoFrameVector currentDesiredCapturePointVelocity)
   {
      desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);
   }

   public void setDesiredCapturePointState(FramePoint2d currentDesiredCapturePointPosition, FrameVector2d currentDesiredCapturePointVelocity)
   {
      desiredCapturePointPosition.setXY(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.setXY(currentDesiredCapturePointVelocity);
   }

   public void setDesiredCapturePointState(YoFramePoint2d currentDesiredCapturePointPosition, YoFrameVector2d currentDesiredCapturePointVelocity)
   {
      desiredCapturePointPosition.setXY(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.setXY(currentDesiredCapturePointVelocity);
   }

   public void initializeDoubleSupport(double initialTime, RobotSide transferToSide)
   {
      this.transferToSide.set(transferToSide);
      this.supportSide.set(null);

      if (transferToSide == null) transferToSide = RobotSide.LEFT;
      RobotSide transferFromSide = transferToSide.getOppositeSide();
      ReferenceFrame transerFromSoleFrame = soleFrames.get(transferFromSide);
      ReferenceFrame transferToSoleFrame = soleFrames.get(transferToSide);

      isDoubleSupport.set(true);

      this.initialTime.set(initialTime);

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromDoubleSupport(isStanding.getBooleanValue(), transferToSide);
      ArrayList<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
      ArrayList<YoFramePoint> exitCMPs = referenceCMPsCalculator.getExitCMPs();
      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      double doubleSupportFractionTime = doubleSupportDuration.getDoubleValue() * doubleSupportSplitFraction.getDoubleValue();
      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);

      if (referenceCMPsCalculator.isDoneWalking())
      {
         singleSupportInitialICP.setIncludingFrame(entryCMPs.get(0));
         singleSupportInitialICP.changeFrame(transerFromSoleFrame);
         singleSupportFinalICP.setIncludingFrame(singleSupportInitialICP);
         singleSupportInitialICPVelocity.set(0.0, 0.0, 0.0);
         setCornerPointsToNaN();
      }
      else
      {
         if (useTwoConstantCMPsPerSupport.getBooleanValue())
            CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, steppingDuration, exitCMPDurationInPercentOfStepTime.getDoubleValue(), omega0.getDoubleValue());
         else
            CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, entryCMPs, false, steppingDuration, omega0.getDoubleValue());

         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportFractionTime, entryCornerPoints.get(1), entryCMPs.get(1), singleSupportInitialICP);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), 0.0, singleSupportInitialICP, entryCMPs.get(1), singleSupportInitialICPVelocity);

         singleSupportInitialICP.changeFrame(transferToSoleFrame);
         singleSupportFinalICP.changeFrame(transferToSoleFrame);
         entryCornerPoints.get(0).changeFrame(transerFromSoleFrame);
         switchFrameOfRemainingCornerPoints(1, transferToSoleFrame);
      }
      if (isStanding.getBooleanValue() && !referenceCMPsCalculator.isDoneWalking())
      {
         isInitialTransfer.set(true);
         isStanding.set(false);
      }

      DoubleYoVariable doubleSupportTimeToUse = isInitialTransfer.getBooleanValue() ? doubleSupportInitialTransferDuration : doubleSupportDuration;
      
      icpTrajectoryGenerator.setTrajectoryTime(doubleSupportTimeToUse.getDoubleValue());
      icpTrajectoryGenerator.setInitialConditions(desiredCapturePointPosition, desiredCapturePointVelocity, transerFromSoleFrame);
      icpTrajectoryGenerator.setFinalConditions(singleSupportInitialICP, singleSupportInitialICPVelocity, transferToSoleFrame);
      icpTrajectoryGenerator.initialize();
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      this.transferToSide.set(null);
      this.supportSide.set(supportSide);


      isInitialTransfer.set(false);
      isDoubleSupport.set(false);
      this.initialTime.set(initialTime);

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromSingleSupport(supportSide);
      ArrayList<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      double doubleSupportFractionTime = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue();

      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);

      ReferenceFrame supportSoleFrame = soleFrames.get(supportSide);
      if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         ArrayList<YoFramePoint> exitCMPs = referenceCMPsCalculator.getExitCMPs();
         double exitToEntryPointShiftDuration = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
         double exitPointToDoubleSupportDuration = exitToEntryPointShiftDuration - doubleSupportDuration.getDoubleValue() * (1.0 - doubleSupportSplitFraction.getDoubleValue());
         
         CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, steppingDuration, exitCMPDurationInPercentOfStepTime.getDoubleValue(), omega0.getDoubleValue());
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportFractionTime, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICP);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), doubleSupportFractionTime, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICPVelocity);

         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), exitPointToDoubleSupportDuration, exitCornerPoints.get(0), exitCMPs.get(0), singleSupportFinalICP);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), exitPointToDoubleSupportDuration, exitCornerPoints.get(0), exitCMPs.get(0), singleSupportFinalICPVelocity);

         
         icpTrajectoryGenerator.setInitialConditions(desiredCapturePointPosition, desiredCapturePointVelocity, supportSoleFrame);
         icpTrajectoryGenerator.setFinalConditions(singleSupportFinalICP, singleSupportFinalICPVelocity, supportSoleFrame);
         icpTrajectoryGenerator.setTrajectoryTime(singleSupportDuration.getDoubleValue());
         icpTrajectoryGenerator.initialize();
      }
      else
      {
         CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, entryCMPs, false, steppingDuration, omega0.getDoubleValue());
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportFractionTime, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICP);
      }

      singleSupportInitialICP.changeFrame(supportSoleFrame);
      singleSupportFinalICP.changeFrame(supportSoleFrame);
      switchFrameOfRemainingCornerPoints(0, supportSoleFrame);
   }

   private void setCornerPointsToNaN()
   {
      for (int i = 0; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).setToNaN();
      for (int i = 0; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).setToNaN();
   }

   private void switchCornerPointsToWorldFrame()
   {
      switchFrameOfRemainingCornerPoints(0, worldFrame);
   }

   private void switchFrameOfRemainingCornerPoints(int fromIndex, ReferenceFrame frameToSwitchTo)
   {
      for (int i = fromIndex; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).switchCurrentReferenceFrame(frameToSwitchTo);
      for (int i = fromIndex; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).switchCurrentReferenceFrame(frameToSwitchTo);
   }

   public void updatePlanForSingleSupportDisturbances(double time, FramePoint actualCapturePointPosition)
   {
      initializeSingleSupport(initialTime.getDoubleValue(), supportSide.getEnumValue());

      YoFramePoint constantCMP = referenceCMPsCalculator.getNextEntryCMP();
      double actualDistanceDueToDisturbance = constantCMP.distance(actualCapturePointPosition);
      double expectedDistanceAccordingToPlan = constantCMP.distance(singleSupportInitialICP);

      double correctedTimeInCurrentState = Math.log(actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan) / omega0.getDoubleValue();

      computeTimeInCurrentState(time);
      double deltaTimeToBeAccounted = correctedTimeInCurrentState - timeInCurrentState.getDoubleValue();

      if (!Double.isNaN(deltaTimeToBeAccounted))
      {
         deltaTimeToBeAccounted *= timeCorrectionFactor.getDoubleValue();
         deltaTimeToBeAccounted = MathTools.clipToMinMax(deltaTimeToBeAccounted, 0.0, Math.max(0.0, computeAndReturnTimeRemaining(time) - minSingleSupportDuration.getDoubleValue()));
         
         initialTime.sub(deltaTimeToBeAccounted);
      }
   }

   private void computeDesiredCapturePoint(double time)
   {
      update();
      referenceCMPsCalculator.update();

      if (isDoubleSupport.getBooleanValue() || useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         icpTrajectoryGenerator.compute(time);
         icpTrajectoryGenerator.packLinearData(desiredCapturePointPosition, desiredCapturePointVelocity, desiredCapturePointAcceleration);
      }
      else
      {
         referenceCMPsCalculator.getNextEntryCMP(tempConstantCMP);
         singleSupportInitialICP.getFrameTupleIncludingFrame(tempICP);
         tempICP.changeFrame(worldFrame);
         time = MathTools.clipToMinMax(time, 0.0, singleSupportDuration.getDoubleValue());
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), time, tempICP, tempConstantCMP, desiredCapturePointPosition);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), time, tempICP, tempConstantCMP, desiredCapturePointVelocity);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), time, tempICP, tempConstantCMP, desiredCapturePointAcceleration);
      }
   }

   private void update()
   {
      singleSupportInitialICP.notifyVariableChangedListeners();
      singleSupportFinalICP.notifyVariableChangedListeners();

      for (int i = 0; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).notifyVariableChangedListeners();
      for (int i = 0; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).notifyVariableChangedListeners();
   }

   private void computeDesiredCentroidalMomentumPivot()
   {
      if (isDoubleSupport.getBooleanValue() || useTwoConstantCMPsPerSupport.getBooleanValue())
         CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredCapturePointPosition, desiredCapturePointVelocity, omega0.getDoubleValue(), desiredCentroidalMomentumPivotPosition);
      else
         desiredCentroidalMomentumPivotPosition.set(referenceCMPsCalculator.getNextEntryCMP());
   }

   public void packDesiredCapturePointPositionVelocityAndAcceleration(FramePoint desiredCapturePointPositionToPack,
         FrameVector desiredCapturePointVelocityToPack, FrameVector desiredCapturePointAccelerationToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
      desiredCapturePointAcceleration.getFrameTupleIncludingFrame(desiredCapturePointAccelerationToPack);
   }

   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   protected void packDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack, YoFrameVector desiredCapturePointVelocityToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPositionToPack.set(desiredCapturePointPosition);
      desiredCapturePointVelocityToPack.set(desiredCapturePointVelocity);
   }

   public void packDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      computeDesiredCentroidalMomentumPivot();

      desiredCentroidalMomentumPivotPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   protected void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   public double computeAndReturnTimeRemaining(double time)
   {
      computeTimeInCurrentState(time);

      DoubleYoVariable stateDuration;

      if (isDoubleSupport.getBooleanValue())
         stateDuration = isInitialTransfer.getBooleanValue() ? doubleSupportInitialTransferDuration : doubleSupportDuration;
      else
         stateDuration = singleSupportDuration;

      remainingTime.set(stateDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
      return remainingTime.getDoubleValue();
   }

   public void setInitialDoubleSupportTime(double time)
   {
      doubleSupportInitialTransferDuration.set(time);
   }

   public void setDoubleSupportTime(double time)
   {
      doubleSupportDuration.set(time);
   }

   public void setSingleSupportTime(double time)
   {
      singleSupportDuration.set(time);
   }

   public void setDoubleSupportSplitFraction(double doubleSupportSplitFraction)
   {
      this.doubleSupportSplitFraction.set(doubleSupportSplitFraction);
   }

   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   public boolean isInDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   public void getSingleSupportInitialCapturePointPosition(FramePoint capturePointPositionToPack)
   {
      singleSupportInitialICP.getFrameTupleIncludingFrame(capturePointPositionToPack);
   }

   public double computeAndReturnTimeInCurrentState(double time)
   {
      computeTimeInCurrentState(time);
      return timeInCurrentState.getDoubleValue();
   }

   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      entryCornerPoints.get(1).getFrameTupleIncludingFrame(finalDesiredCapturePointPositionToPack);
   }

   public void reset(double time)
   {
      isStanding.set(true);

      clearPlan();
      initializeDoubleSupport(time, null);
   }

   public boolean getHasBeenWokenUp()
   {
      return hasBeenWokenUp.getBooleanValue();
   }

   public void wakeUp()
   {
      hasBeenWokenUp.set(true);
   }

   public boolean isDone(double time)
   {
      double timeRemaining = computeAndReturnTimeRemaining(time);
      return (timeRemaining <= isDoneTimeThreshold.getDoubleValue());
   }
}
