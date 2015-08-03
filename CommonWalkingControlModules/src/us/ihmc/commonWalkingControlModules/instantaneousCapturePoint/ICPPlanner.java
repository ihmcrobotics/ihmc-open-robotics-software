package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(namePrefix + "DoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(namePrefix + "SingleSupportTime", registry);
   private final DoubleYoVariable minSingleSupportDuration = new DoubleYoVariable(namePrefix + "MinSingleSupportTime", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable(namePrefix + "InitialTransferDuration", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable(namePrefix + "DoubleSupportSplitFraction", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable(namePrefix + "RemainingTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable(namePrefix + "NumberFootstepsToConsider", registry);

   private final YoFramePointInMultipleFrames singleSupportInitialICP;
   private final YoFrameVector singleSupportInitialICPVelocity = new YoFrameVector(namePrefix + "SingleSupportInitialICPVelocity", worldFrame, registry);

   private final YoFramePointInMultipleFrames singleSupportFinalICP;
   private final YoFrameVector singleSupportFinalICPVelocity = new YoFrameVector(namePrefix + "SingleSupportFinalICPVelocity", worldFrame, registry);

   private final YoFramePoint desiredCentroidalMomentumPivotPosition = new YoFramePoint(namePrefix + "DesiredCentroidalMomentumPosition", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint(namePrefix + "DesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector(namePrefix + "DesiredCapturePointVelocity", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable(namePrefix + "Omega0", registry);

   private final BooleanYoVariable requestedHoldPosition = new BooleanYoVariable(namePrefix + "RequestedHoldPosition", registry);
   private final BooleanYoVariable isHoldingPosition = new BooleanYoVariable(namePrefix + "IsHoldingPosition", registry);
   private final YoFramePoint actualICPToHold = new YoFramePoint(namePrefix + "ActualCapturePointToHold", worldFrame, registry);

   private final BooleanYoVariable useTwoConstantCMPsPerSupport = new BooleanYoVariable(namePrefix + "UseTwoConstantCMPsPerSupport", registry);
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable(namePrefix + "TimeSpentOnExitCMPInPercentOfStepTime", registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   private final ArrayList<YoFramePointInMultipleFrames> entryCornerPoints = new ArrayList<YoFramePointInMultipleFrames>();
   private final ArrayList<YoFramePointInMultipleFrames> exitCornerPoints = new ArrayList<YoFramePointInMultipleFrames>();

   private final ICPPlannerTrajectoryGenerator icpDoubleSupportTrajectoryGenerator;
   private final ICPPlannerSegmentedTrajectoryGenerator icpSingleSupportTrajectoryGenerator;
   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final FramePoint tempConstantCMP = new FramePoint();
   private final FramePoint tempICP = new FramePoint();

   public ICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         CapturePointPlannerParameters icpPlannerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      isStanding.set(true);
      hasBeenWokenUp.set(false);

      actualICPToHold.setToNaN();
      isHoldingPosition.set(false);

      icpDoubleSupportTrajectoryGenerator = new ICPPlannerTrajectoryGenerator(namePrefix + "DoubleSupport", worldFrame, registry);
      icpSingleSupportTrajectoryGenerator = new ICPPlannerSegmentedTrajectoryGenerator(namePrefix + "SingleSupport", worldFrame, omega0, yoGraphicsListRegistry, registry);
      icpSingleSupportTrajectoryGenerator.setMaximumSplineDuration(icpPlannerParameters.getMaxDurationForSmoothingEntryToExitCMPSwitch());
      doubleSupportInitialTransferDuration.set(icpPlannerParameters.getDoubleSupportInitialTransferDuration());
      numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());
      doubleSupportSplitFraction.set(icpPlannerParameters.getDoubleSupportSplitFraction());
      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      useTwoConstantCMPsPerSupport.set(icpPlannerParameters.useTwoCMPsPerSupport());
      
      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      minSingleSupportDuration.set(0.3); // TODO Need to be extracted

      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet, numberFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      midFeetZUpFrame = bipedSupportPolygons.getMidFeetZUpFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.put(robotSide, contactableFeet.get(robotSide).getSoleFrame());
      }

      ReferenceFrame[] framesToRegister = new ReferenceFrame[]{worldFrame, midFeetZUpFrame, soleFrames.get(RobotSide.LEFT), soleFrames.get(RobotSide.RIGHT)};
      singleSupportInitialICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportInitialICP", registry, framesToRegister);
      singleSupportFinalICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportFinalICP", registry, framesToRegister);

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePointInMultipleFrames earlyCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "EntryCornerPoints" + i, registry, framesToRegister);
         entryCornerPoints.add(earlyCornerPoint);

         YoFramePointInMultipleFrames lateCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "ExitCornerPoints" + i, registry, framesToRegister);
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

      YoGraphicPosition singleSupportInitialICPViz = new YoGraphicPosition("singleSupportInitialICP", singleSupportInitialICP.buildUpdatedYoFramePointForVisualizationOnly(), 0.004, YoAppearance.Chocolate(), GraphicType.SOLID_BALL);
      yoGraphicsList.add(singleSupportInitialICPViz);
      artifactList.add(singleSupportInitialICPViz.createArtifact());

      YoGraphicPosition singleSupportFinalICPViz = new YoGraphicPosition("singleSupportFinalICP", singleSupportFinalICP.buildUpdatedYoFramePointForVisualizationOnly(), 0.004, YoAppearance.Chocolate(), GraphicType.BALL);
      yoGraphicsList.add(singleSupportFinalICPViz);
      artifactList.add(singleSupportFinalICPViz.createArtifact());

      icpSingleSupportTrajectoryGenerator.createVisualizers(yoGraphicsList, artifactList);

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
      // Do not set the Z to zero!
      desiredCapturePointPosition.checkReferenceFrameMatch(currentDesiredCapturePointPosition);
      desiredCapturePointPosition.setX(currentDesiredCapturePointPosition.getX());
      desiredCapturePointPosition.setY(currentDesiredCapturePointPosition.getY());

      desiredCapturePointVelocity.checkReferenceFrameMatch(currentDesiredCapturePointVelocity);
      desiredCapturePointVelocity.setX(currentDesiredCapturePointVelocity.getX());
      desiredCapturePointVelocity.setY(currentDesiredCapturePointVelocity.getY());
   }

   public void setDesiredCapturePointState(YoFramePoint2d currentDesiredCapturePointPosition, YoFrameVector2d currentDesiredCapturePointVelocity)
   {
      // Do not set the Z to zero!
      desiredCapturePointPosition.checkReferenceFrameMatch(currentDesiredCapturePointPosition);
      desiredCapturePointPosition.setX(currentDesiredCapturePointPosition.getX());
      desiredCapturePointPosition.setY(currentDesiredCapturePointPosition.getY());

      desiredCapturePointVelocity.checkReferenceFrameMatch(currentDesiredCapturePointVelocity);
      desiredCapturePointVelocity.setX(currentDesiredCapturePointVelocity.getX());
      desiredCapturePointVelocity.setY(currentDesiredCapturePointVelocity.getY());
   }

   public void holdCurrentICP(double initialTime, FramePoint actualICPToHold)
   {
      this.actualICPToHold.set(actualICPToHold);
      requestedHoldPosition.set(true);
   }

   public void initializeDoubleSupport(double initialTime, RobotSide transferToSide)
   {
      this.transferToSide.set(transferToSide);
      this.supportSide.set(null);

      if (transferToSide == null) transferToSide = RobotSide.LEFT;
      RobotSide transferFromSide = transferToSide.getOppositeSide();
      ReferenceFrame transferFromSoleFrame = soleFrames.get(transferFromSide);
      ReferenceFrame transferToSoleFrame = soleFrames.get(transferToSide);

      isDoubleSupport.set(true);
      icpSingleSupportTrajectoryGenerator.hideVisualization();

      this.initialTime.set(initialTime);

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromDoubleSupport(isStanding.getBooleanValue(), transferToSide);
      referenceCMPsCalculator.update();
      ArrayList<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
      ArrayList<YoFramePoint> exitCMPs = referenceCMPsCalculator.getExitCMPs();
      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      double doubleSupportTimeSpentBeforeEntryCornerPoint = doubleSupportDuration.getDoubleValue() * doubleSupportSplitFraction.getDoubleValue();
      double doubleSupportTimeSpentAfterEntryCornerPoint = doubleSupportDuration.getDoubleValue() * (1.0 - doubleSupportSplitFraction.getDoubleValue());
      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);

      boolean isDoneWalking = referenceCMPsCalculator.isDoneWalking();

      ReferenceFrame initialFrame;

      if (isStanding.getBooleanValue())
      {
         initialFrame = midFeetZUpFrame;
      }
      else
      {
         tempICP.setToZero(midFeetZUpFrame);
         tempICP.changeFrame(worldFrame);
         double distanceFromDesiredICPToMidfeetZUpFrame = desiredCapturePointPosition.getXYPlaneDistance(tempICP);
         tempICP.setToZero(transferFromSoleFrame);
         tempICP.changeFrame(worldFrame);
         double distanceFromDesiredICPToTransferFromSoleFrame = desiredCapturePointPosition.getXYPlaneDistance(tempICP);
         
         if (distanceFromDesiredICPToMidfeetZUpFrame < distanceFromDesiredICPToTransferFromSoleFrame)
            initialFrame = midFeetZUpFrame;
         else
            initialFrame = transferFromSoleFrame;
      }
      ReferenceFrame finalFrame = isDoneWalking ? midFeetZUpFrame : transferToSoleFrame;

      if (requestedHoldPosition.getBooleanValue())
      {
         desiredCapturePointPosition.set(actualICPToHold);
         desiredCapturePointVelocity.setToZero();
         singleSupportInitialICP.setIncludingFrame(actualICPToHold);
         singleSupportFinalICP.setIncludingFrame(actualICPToHold);
         singleSupportInitialICPVelocity.set(0.0, 0.0, 0.0);
         setCornerPointsToNaN();
         actualICPToHold.setToNaN();
         isDoneWalking = true;
         requestedHoldPosition.set(false);
         isHoldingPosition.set(true);
      }
      else if (isDoneWalking)
      {
         singleSupportInitialICP.setIncludingFrame(entryCMPs.get(0));
         singleSupportFinalICP.setIncludingFrame(singleSupportInitialICP);
         singleSupportInitialICPVelocity.set(0.0, 0.0, 0.0);
         setCornerPointsToNaN();
         isHoldingPosition.set(false);
      }
      else
      {
         if (useTwoConstantCMPsPerSupport.getBooleanValue())
         {
            double exitPointToDoubleSupportDuration = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue() - doubleSupportTimeSpentAfterEntryCornerPoint;

            CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, steppingDuration, exitCMPDurationInPercentOfStepTime.getDoubleValue(), omega0.getDoubleValue());
            CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), exitPointToDoubleSupportDuration, exitCornerPoints.get(1), exitCMPs.get(1), singleSupportFinalICP);
            exitCornerPoints.get(0).changeFrame(initialFrame);
            exitCornerPoints.get(1).changeFrame(finalFrame);
         }
         else
         {
            CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, entryCMPs, false, steppingDuration, omega0.getDoubleValue());
            CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportTimeSpentBeforeEntryCornerPoint + singleSupportDuration.getDoubleValue(), entryCornerPoints.get(1), entryCMPs.get(1), singleSupportFinalICP);
         }

         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportTimeSpentBeforeEntryCornerPoint, entryCornerPoints.get(1), entryCMPs.get(1), singleSupportInitialICP);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), 0.0, singleSupportInitialICP, entryCMPs.get(1), singleSupportInitialICPVelocity);

         entryCornerPoints.get(0).changeFrame(initialFrame);
         entryCornerPoints.get(1).changeFrame(finalFrame);
         changeFrameOfRemainingCornerPoints(2, worldFrame);
         isHoldingPosition.set(false);
      }

      singleSupportInitialICP.changeFrame(finalFrame);
      singleSupportFinalICP.changeFrame(worldFrame);

      DoubleYoVariable doubleSupportTimeToUse = isStanding.getBooleanValue() ? doubleSupportInitialTransferDuration : doubleSupportDuration;

      icpDoubleSupportTrajectoryGenerator.setTrajectoryTime(doubleSupportTimeToUse.getDoubleValue());

      icpDoubleSupportTrajectoryGenerator.setInitialConditions(desiredCapturePointPosition, desiredCapturePointVelocity, initialFrame);
      icpDoubleSupportTrajectoryGenerator.setFinalConditions(singleSupportInitialICP, singleSupportInitialICPVelocity, finalFrame);
      icpDoubleSupportTrajectoryGenerator.initialize();

      if (isStanding.getBooleanValue() && !isDoneWalking)
      {
         isInitialTransfer.set(true);
         isStanding.set(false);
      }
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      initializeSingleSupport(initialTime, supportSide, true);
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide, boolean startFromCurrentDesiredICP)
   {
      this.transferToSide.set(null);
      this.supportSide.set(supportSide);
      isHoldingPosition.set(false);


      isInitialTransfer.set(false);
      isDoubleSupport.set(false);
      this.initialTime.set(initialTime);

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromSingleSupport(supportSide);
      referenceCMPsCalculator.update();
      ArrayList<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
      ArrayList<YoFramePoint> exitCMPs = referenceCMPsCalculator.getExitCMPs();
      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      double doubleSupportTimeSpentBeforeEntryCornerPoint = doubleSupportDuration.getDoubleValue() * doubleSupportSplitFraction.getDoubleValue();
      double doubleSupportTimeSpentAfterEntryCornerPoint = doubleSupportDuration.getDoubleValue() * (1.0 - doubleSupportSplitFraction.getDoubleValue());
      double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
      double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);

      ReferenceFrame supportSoleFrame = soleFrames.get(supportSide);
      if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         double timeRemainingOnEntryCMP = totalTimeSpentOnEntryCMP - doubleSupportTimeSpentBeforeEntryCornerPoint;
         double timeToSpendOnFinalCMPBeforeDoubleSupport = totalTimeSpentOnExitCMP - doubleSupportTimeSpentAfterEntryCornerPoint;

         CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, steppingDuration, exitCMPDurationInPercentOfStepTime.getDoubleValue(), omega0.getDoubleValue());
         if (startFromCurrentDesiredICP)
         {
            singleSupportInitialICP.setIncludingFrame(desiredCapturePointPosition);
            singleSupportInitialICPVelocity.set(desiredCapturePointVelocity);
         }
         else
         {
            CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportTimeSpentBeforeEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICP);
            CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), doubleSupportTimeSpentBeforeEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICPVelocity);
         }

         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeToSpendOnFinalCMPBeforeDoubleSupport, exitCornerPoints.get(0), exitCMPs.get(0), singleSupportFinalICP);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeToSpendOnFinalCMPBeforeDoubleSupport, exitCornerPoints.get(0), exitCMPs.get(0), singleSupportFinalICPVelocity);

         icpSingleSupportTrajectoryGenerator.setBoundaryICP(singleSupportInitialICP, singleSupportFinalICP);
         icpSingleSupportTrajectoryGenerator.setCornerPoints(entryCornerPoints.get(0), exitCornerPoints.get(0));
         icpSingleSupportTrajectoryGenerator.setReferenceCMPs(entryCMPs.get(0), exitCMPs.get(0));
         icpSingleSupportTrajectoryGenerator.setReferenceFrames(supportSoleFrame, worldFrame);
         icpSingleSupportTrajectoryGenerator.setTrajectoryTime(timeRemainingOnEntryCMP, timeToSpendOnFinalCMPBeforeDoubleSupport);
         icpSingleSupportTrajectoryGenerator.initialize();

         exitCornerPoints.get(0).changeFrame(supportSoleFrame);
      }
      else
      {
         CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, entryCMPs, false, steppingDuration, omega0.getDoubleValue());
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportTimeSpentBeforeEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICP);
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), doubleSupportTimeSpentBeforeEntryCornerPoint + singleSupportDuration.getDoubleValue(), entryCornerPoints.get(0), entryCMPs.get(0), singleSupportFinalICP);
      }

      singleSupportInitialICP.changeFrame(supportSoleFrame);
      entryCornerPoints.get(0).changeFrame(supportSoleFrame);
      singleSupportFinalICP.changeFrame(worldFrame);
      changeFrameOfRemainingCornerPoints(1, worldFrame);
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
      for (int i = 0; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).switchCurrentReferenceFrame(worldFrame);
      for (int i = 0; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).switchCurrentReferenceFrame(worldFrame);
   }

   private void changeFrameOfRemainingCornerPoints(int fromIndex, ReferenceFrame desiredFrame)
   {
      for (int i = fromIndex; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).changeFrame(desiredFrame);
      for (int i = fromIndex; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).changeFrame(desiredFrame);
   }

   public void updatePlanForSingleSupportDisturbances(double time, FramePoint actualCapturePointPosition)
   {
      initializeSingleSupport(initialTime.getDoubleValue(), supportSide.getEnumValue(), false);

      if (isDone(time))
         return;

      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(time, actualCapturePointPosition);

      if (Double.isNaN(deltaTimeToBeAccounted))
         return;

      // Ensure that we don't shift the time by more than what's remaining
      deltaTimeToBeAccounted = Math.min(deltaTimeToBeAccounted, computeAndReturnTimeRemaining(time));
      // Ensure the time shift won't imply a single support that's crazy short
      deltaTimeToBeAccounted = Math.min(deltaTimeToBeAccounted, singleSupportDuration.getDoubleValue() - minSingleSupportDuration.getDoubleValue());

      initialTime.sub(deltaTimeToBeAccounted);
   }

   public double estimateTimeRemainingForStateUnderDisturbance(double time, FramePoint actualCapturePointPosition)
   {
      if (isDone(time))
         return 0.0;

      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(time, actualCapturePointPosition);

      if (Double.isNaN(deltaTimeToBeAccounted))
         return 0.0;

      double estimatedTimeRemaining = computeAndReturnTimeRemaining(time) - deltaTimeToBeAccounted;
      estimatedTimeRemaining = MathTools.clipToMinMax(estimatedTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      return estimatedTimeRemaining;
   }

   private final FramePoint2d desiredICP2d = new FramePoint2d();
   private final FramePoint2d finalICP2d = new FramePoint2d();
   private final FrameLineSegment2d desiredICPToFinalICPLineSegment = new FrameLineSegment2d();
   private final FramePoint2d actualICP2d = new FramePoint2d();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(double time, FramePoint actualCapturePointPosition)
   {
      computeDesiredCapturePoint(computeAndReturnTimeInCurrentState(time));
      computeDesiredCentroidalMomentumPivot();
      desiredCapturePointPosition.getFrameTuple2dIncludingFrame(desiredICP2d);
      singleSupportFinalICP.getFrameTuple2dIncludingFrame(finalICP2d);

      if (desiredICP2d.distance(finalICP2d) < 1.0e-10)
         return Double.NaN;

      desiredICPToFinalICPLineSegment.set(desiredICP2d, finalICP2d);
      actualICP2d.setByProjectionOntoXYPlaneIncludingFrame(actualCapturePointPosition);
      desiredICPToFinalICPLineSegment.orthogonalProjection(actualICP2d);

      double actualDistanceDueToDisturbance = desiredCentroidalMomentumPivotPosition.getXYPlaneDistance(actualICP2d);
      double expectedDistanceAccordingToPlan = desiredCentroidalMomentumPivotPosition.getXYPlaneDistance(desiredCapturePointPosition);

      computeTimeInCurrentState(time);
      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega0.getDoubleValue();
   }

   private void computeDesiredCapturePoint(double timeInCurrentState)
   {
      update();
      referenceCMPsCalculator.update();

      if (isDoubleSupport.getBooleanValue() || useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         if (isDoubleSupport.getBooleanValue())
         {
            icpDoubleSupportTrajectoryGenerator.compute(timeInCurrentState);
            icpDoubleSupportTrajectoryGenerator.get(desiredCapturePointPosition);
            icpDoubleSupportTrajectoryGenerator.packVelocity(desiredCapturePointVelocity);
         }
         else
         {
            icpSingleSupportTrajectoryGenerator.compute(timeInCurrentState);
            icpSingleSupportTrajectoryGenerator.get(desiredCapturePointPosition);
            icpSingleSupportTrajectoryGenerator.packVelocity(desiredCapturePointVelocity);
         }
      }
      else
      {
         referenceCMPsCalculator.getNextEntryCMP(tempConstantCMP);
         singleSupportInitialICP.getFrameTupleIncludingFrame(tempICP);
         tempICP.changeFrame(worldFrame);
         timeInCurrentState = MathTools.clipToMinMax(timeInCurrentState, 0.0, singleSupportDuration.getDoubleValue());
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeInCurrentState, tempICP, tempConstantCMP, desiredCapturePointPosition);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeInCurrentState, tempICP, tempConstantCMP, desiredCapturePointVelocity);
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
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredCapturePointPosition, desiredCapturePointVelocity, omega0.getDoubleValue(), desiredCentroidalMomentumPivotPosition);
   }

   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack, double time)
   {
      computeTimeInCurrentState(time);
      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   public void packDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack, YoFrameVector desiredCapturePointVelocityToPack, double time)
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

   public void setMinimumSingleSupportTimeForDisturbanceRecovery(double minTime)
   {
      minSingleSupportDuration.set(minTime);
   }

   public double getInitialTransferDuration()
   {
      return doubleSupportInitialTransferDuration.getDoubleValue();
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
      return computeAndReturnTimeRemaining(time) <= 0.0;
   }
}
