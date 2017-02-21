package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ReferenceCentroidalMomentumPivotLocationsCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double CMP_POINT_SIZE = 0.005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   /**
    * <li> When using only one CMP per support:
    * Desired constant CMP locations for the entire single support phases. </li>
    * <li> When using two CMPs per support:
    * Desired CMP locations on the feet in the early single support phase. </li>
    * */
   private final List<YoFramePointInMultipleFrames> entryCMPs = new ArrayList<>();
   private final List<YoFramePoint> entryCMPsInWorldFrameReadOnly = new ArrayList<>();

   /**
    * Only used when computing two CMPs per support.
    * Desired CMP locations on the feet in the late single support phase.
    */
   private final List<YoFramePointInMultipleFrames> exitCMPs = new ArrayList<>();
   private final List<YoFramePoint> exitCMPsInWorldFrameReadOnly = new ArrayList<>();

   private final BooleanYoVariable isDoneWalking;
   private final DoubleYoVariable maxForwardEntryCMPOffset;
   private final DoubleYoVariable minForwardEntryCMPOffset;
   private final DoubleYoVariable maxForwardExitCMPOffset;
   private final DoubleYoVariable minForwardExitCMPOffset;
   private final DoubleYoVariable footstepHeightThresholdToPutExitCMPOnToesSteppingDown;
   private final DoubleYoVariable footstepLengthThresholdToPutExitCMPOnToesSteppingDown;

   private final DoubleYoVariable stepLengthToCMPOffsetFactor;
   private final DoubleYoVariable footstepLengthThresholdToPutExitCMPOnToes;

   private final BooleanYoVariable putExitCMPOnToes;

   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> soleZUpFrames;
   private final FrameConvexPolygon2d predictedSupportPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleZUpFrame = new SideDependentList<>();

   private final SideDependentList<YoFrameVector2d> entryCMPUserOffsets = new SideDependentList<>();
   private final SideDependentList<YoFrameVector2d> exitCMPUserOffsets = new SideDependentList<>();

   private final IntegerYoVariable numberOfUpcomingFootsteps;
   private final List<Footstep> upcomingFootsteps = new ArrayList<>();

   private final FramePoint cmp = new FramePoint();
   private final FramePoint firstCMP = new FramePoint();
   private final FramePoint secondCMP = new FramePoint();

   private final FramePoint soleFrameOrigin = new FramePoint();
   private final FrameVector soleToSoleFrameVector = new FrameVector();

   private final FramePoint2d cmp2d = new FramePoint2d();
   private final FramePoint2d previousExitCMP2d = new FramePoint2d();
   private final FramePoint2d firstEntryCMPForSingleSupport = new FramePoint2d();
   private final SideDependentList<ConvexPolygon2d> defaultFootPolygons = new SideDependentList<>();
   private final FrameConvexPolygon2d tempSupportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempSupportPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();

   private final DoubleYoVariable safeDistanceFromCMPToSupportEdges;
   private final DoubleYoVariable safeDistanceFromCMPToSupportEdgesWhenSteppingDown;

   private final FramePoint2d centroidOfUpcomingFootstep = new FramePoint2d();
   private final FramePoint2d centroidOfCurrentFootstep = new FramePoint2d();
   private final FramePoint2d centroidOfFootstepToConsider = new FramePoint2d();

   private final FramePoint tempFramePoint = new FramePoint();

   private boolean useTwoCMPsPerSupport = false;
   private boolean useExitCMPOnToesForSteppingDown = false;

   /**
    * By default the CMPs for the last step are centered between the foot support polygon centroids. This parameter (default 0.5)
    * specifies where on the line connecting the centroids the CMPs are placed.
    */
   private final DoubleYoVariable percentageChickenSupport;

   public ReferenceCentroidalMomentumPivotLocationsCalculator(String namePrefix, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, int numberFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      firstEntryCMPForSingleSupport.setToNaN();

      isDoneWalking = new BooleanYoVariable(namePrefix + "IsDoneWalking", registry);
      maxForwardEntryCMPOffset = new DoubleYoVariable(namePrefix + "MaxForwardEntryCMPOffset", registry);
      minForwardEntryCMPOffset = new DoubleYoVariable(namePrefix + "MinForwardEntryCMPOffset", registry);
      maxForwardExitCMPOffset = new DoubleYoVariable(namePrefix + "MaxForwardExitCMPOffset", registry);
      minForwardExitCMPOffset = new DoubleYoVariable(namePrefix + "MinForwardExitCMPOffset", registry);
      safeDistanceFromCMPToSupportEdges = new DoubleYoVariable(namePrefix + "SafeDistanceFromCMPToSupportEdges", registry);
      footstepHeightThresholdToPutExitCMPOnToesSteppingDown = new DoubleYoVariable(namePrefix + "FootstepHeightThresholdToPutExitCMPOnToesSteppingDown", registry);
      footstepLengthThresholdToPutExitCMPOnToesSteppingDown = new DoubleYoVariable(namePrefix + "FootstepLengthThresholdToPutExitCMPOnToesSteppingDown", registry);
      safeDistanceFromCMPToSupportEdgesWhenSteppingDown = new DoubleYoVariable(namePrefix + "SafeDistanceFromCMPToSupportEdgesWhenSteppingDown", registry);

      stepLengthToCMPOffsetFactor = new DoubleYoVariable(namePrefix + "StepLengthToCMPOffsetFactor", registry);
      footstepLengthThresholdToPutExitCMPOnToes = new DoubleYoVariable(namePrefix + "FootstepLengthThresholdToPutExitCMPOnToes", registry);

      numberOfUpcomingFootsteps = new IntegerYoVariable(namePrefix + "NumberOfUpcomingFootsteps", registry);

      putExitCMPOnToes = new BooleanYoVariable(namePrefix + "ToeOffInSingleSupport", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon.getConvexPolygon2d());

         String sidePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         YoFrameVector2d entryCMPUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "EntryCMPConstantOffsets", null, registry);
         entryCMPUserOffsets.put(robotSide, entryCMPUserOffset);
         YoFrameVector2d exitCMPUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "ExitCMPConstantOffsets", null, registry);
         exitCMPUserOffsets.put(robotSide, exitCMPUserOffset);
         supportFootPolygonsInSoleZUpFrame.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(robotSide));
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleZUpFrame.get(robotSide)); // Just to allocate memory
      }

      midFeetZUpFrame = bipedSupportPolygons.getMidFeetZUpFrame();
      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, midFeetZUpFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT)};

      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         YoFramePointInMultipleFrames entryConstantCMP = new YoFramePointInMultipleFrames(namePrefix + "EntryCMP" + i, parentRegistry, framesToRegister);
         entryConstantCMP.setToNaN();
         entryCMPs.add(entryConstantCMP);
         entryCMPsInWorldFrameReadOnly.add(entryConstantCMP.buildUpdatedYoFramePointForVisualizationOnly());

         YoFramePointInMultipleFrames exitConstantCMP = new YoFramePointInMultipleFrames(namePrefix + "ExitCMP" + i, parentRegistry, framesToRegister);
         exitConstantCMP.setToNaN();
         exitCMPs.add(exitConstantCMP);
         exitCMPsInWorldFrameReadOnly.add(exitConstantCMP.buildUpdatedYoFramePointForVisualizationOnly());
      }

      percentageChickenSupport = new DoubleYoVariable("PercentageChickenSupport", registry);
      percentageChickenSupport.set(0.5);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (int i = 0; i < entryCMPs.size(); i++)
         entryCMPs.get(i).notifyVariableChangedListeners();
      for (int i = 0; i < exitCMPs.size(); i++)
         exitCMPs.get(i).notifyVariableChangedListeners();
   }

   public void initializeParameters(CapturePointPlannerParameters icpPlannerParameters)
   {
      useTwoCMPsPerSupport = icpPlannerParameters.useTwoCMPsPerSupport();
      safeDistanceFromCMPToSupportEdges.set(icpPlannerParameters.getCMPSafeDistanceAwayFromSupportEdges());

      maxForwardEntryCMPOffset.set(icpPlannerParameters.getMaxEntryCMPForwardOffset());
      minForwardEntryCMPOffset.set(icpPlannerParameters.getMinEntryCMPForwardOffset());

      maxForwardExitCMPOffset.set(icpPlannerParameters.getMaxExitCMPForwardOffset());
      minForwardExitCMPOffset.set(icpPlannerParameters.getMinExitCMPForwardOffset());

      stepLengthToCMPOffsetFactor.set(icpPlannerParameters.getStepLengthToCMPOffsetFactor());

      double entryCMPForwardOffset = icpPlannerParameters.getEntryCMPForwardOffset();
      double entryCMPInsideOffset = icpPlannerParameters.getEntryCMPInsideOffset();
      double exitCMPForwardOffset = icpPlannerParameters.getExitCMPForwardOffset();
      double exitCMPInsideOffset = icpPlannerParameters.getExitCMPInsideOffset();
      setSymmetricEntryCMPConstantOffsets(entryCMPForwardOffset, entryCMPInsideOffset);
      setSymmetricExitCMPConstantOffsets(exitCMPForwardOffset, exitCMPInsideOffset);

      useExitCMPOnToesForSteppingDown = icpPlannerParameters.useExitCMPOnToesForSteppingDown();
      footstepHeightThresholdToPutExitCMPOnToesSteppingDown.set(icpPlannerParameters.getStepHeightThresholdForExitCMPOnToesWhenSteppingDown());
      footstepLengthThresholdToPutExitCMPOnToesSteppingDown.set(icpPlannerParameters.getStepLengthThresholdForExitCMPOnToesWhenSteppingDown());
      safeDistanceFromCMPToSupportEdgesWhenSteppingDown.set(icpPlannerParameters.getCMPSafeDistanceAwayFromToesWhenSteppingDown());

      putExitCMPOnToes.set(icpPlannerParameters.putExitCMPOnToes());
      footstepLengthThresholdToPutExitCMPOnToes.set(icpPlannerParameters.getStepLengthThresholdForExitCMPOnToes());
   }

   public void setUseTwoCMPsPerSupport(boolean useTwoCMPsPerSupport)
   {
      this.useTwoCMPsPerSupport = useTwoCMPsPerSupport;
   }

   public void setSafeDistanceFromSupportEdges(double distance)
   {
      safeDistanceFromCMPToSupportEdges.set(distance);
   }

   public void setMinMaxForwardEntryCMPLocationFromFootCenter(double minX, double maxX)
   {
      maxForwardEntryCMPOffset.set(maxX);
      minForwardEntryCMPOffset.set(minX);
   }

   public void setMinMaxForwardExitCMPLocationFromFootCenter(double minX, double maxX)
   {
      maxForwardExitCMPOffset.set(maxX);
      minForwardExitCMPOffset.set(minX);
   }

   public void setSymmetricEntryCMPConstantOffsets(double entryCMPForwardOffset, double entryCMPInsideOffset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d entryCMPUserOffset = entryCMPUserOffsets.get(robotSide);
         entryCMPUserOffset.setX(entryCMPForwardOffset);
         entryCMPUserOffset.setY(robotSide.negateIfLeftSide(entryCMPInsideOffset));
      }
   }

   public void setSymmetricExitCMPConstantOffsets(double exitCMPForwardOffset, double exitCMPInsideOffset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d exitCMPUserOffset = exitCMPUserOffsets.get(robotSide);
         exitCMPUserOffset.setX(exitCMPForwardOffset);
         exitCMPUserOffset.setY(robotSide.negateIfLeftSide(exitCMPInsideOffset));
      }
   }

   public void createVisualizerForConstantCMPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      for (int i = 0; i < entryCMPs.size(); i++)
      {
         YoGraphicPosition entryCMPViz = new YoGraphicPosition("Entry CMP" + i, entryCMPsInWorldFrameReadOnly.get(i), CMP_POINT_SIZE, YoAppearance.Green(),
               GraphicType.SOLID_BALL);
         yoGraphicsList.add(entryCMPViz);
         artifactList.add(entryCMPViz.createArtifact());
      }

      for (int i = 0; i < exitCMPs.size(); i++)
      {
         YoGraphicPosition exitCMPViz = new YoGraphicPosition("Exit CMP" + i, exitCMPsInWorldFrameReadOnly.get(i), CMP_POINT_SIZE, YoAppearance.Green(),
               GraphicType.BALL);
         yoGraphicsList.add(exitCMPViz);
         artifactList.add(exitCMPViz.createArtifact());
      }
   }

   public void clear()
   {
      upcomingFootsteps.clear();
   }

   public void addUpcomingFootstep(Footstep footstep)
   {
      if (footstep != null)
         upcomingFootsteps.add(footstep);
   }

   public int getNumberOfFootstepRegistered()
   {
      return upcomingFootsteps.size();
   }

   public void computeReferenceCMPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      RobotSide transferFromSide = transferToSide.getOppositeSide();
      int numberOfUpcomingFootsteps = upcomingFootsteps.size();
      this.numberOfUpcomingFootsteps.set(numberOfUpcomingFootsteps);
      int cmpIndex = 0;
      boolean noUpcomingFootsteps = numberOfUpcomingFootsteps == 0;
      isDoneWalking.set(noUpcomingFootsteps);
      ReferenceFrame transferToSoleFrame = soleZUpFrames.get(transferToSide);
      ReferenceFrame transferFromSoleFrame = soleZUpFrames.get(transferFromSide);

      if (atAStop || noUpcomingFootsteps)
      {
         FrameConvexPolygon2d footA = supportFootPolygonsInSoleZUpFrame.get(transferFromSide);
         FrameConvexPolygon2d footB = supportFootPolygonsInSoleZUpFrame.get(transferFromSide.getOppositeSide());
         computeFinalCMPBetweenSupportFeet(cmpIndex, footA, footB);
         cmpIndex++;

         if (noUpcomingFootsteps)
         {
            setRemainingCMPsToDuplicateLastComputedCMP(0);
            return;
         }
      }
      else
      {
         if (useTwoCMPsPerSupport)
         {
            entryCMPs.get(cmpIndex).setToNaN();
         }
         else
         {
            computeEntryCMPForSupportFoot(cmp, transferFromSide, null, null);
            cmp.changeFrame(transferFromSoleFrame);
            entryCMPs.get(cmpIndex).setIncludingFrame(cmp);
         }
         boolean isUpcomingFootstepLast = noUpcomingFootsteps;
         computeExitCMPForSupportFoot(cmp, transferFromSide, supportFootPolygonsInSoleZUpFrame.get(transferToSide).getCentroid(), isUpcomingFootstepLast);
         cmp.changeFrame(transferFromSoleFrame);
         exitCMPs.get(cmpIndex).setIncludingFrame(cmp);
         cmpIndex++;
      }

      computeEntryCMPForSupportFoot(cmp, transferToSide, supportFootPolygonsInSoleZUpFrame.get(transferFromSide).getCentroid(), exitCMPs.get(cmpIndex - 1));
      cmp.changeFrame(transferToSoleFrame);
      entryCMPs.get(cmpIndex).setIncludingFrame(cmp);
      firstEntryCMPForSingleSupport.setByProjectionOntoXYPlaneIncludingFrame(cmp);
      computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootsteps.get(0));
      boolean isUpcomingFootstepLast = upcomingFootsteps.size() == 1;
      computeExitCMPForSupportFoot(cmp, transferToSide, centroidOfUpcomingFootstep, isUpcomingFootstepLast);
      cmp.changeFrame(transferToSoleFrame);
      exitCMPs.get(cmpIndex).setIncludingFrame(cmp);
      cmpIndex++;

      computeReferenceCMPsWithUpcomingFootsteps(transferToSide, numberOfUpcomingFootsteps, cmpIndex);
      changeFrameOfCMPs(2, worldFrame);
   }

   public void computeReferenceCMPsStartingFromSingleSupport(RobotSide supportSide)
   {
      int numberOfUpcomingFootsteps = upcomingFootsteps.size();
      this.numberOfUpcomingFootsteps.set(numberOfUpcomingFootsteps);
      int constantCMPIndex = 0;
      boolean onlyOneUpcomingFootstep = numberOfUpcomingFootsteps == 1;
      isDoneWalking.set(onlyOneUpcomingFootstep);

      // Can happen if this method is the first to be called, so it should pretty much never happen.
      if (firstEntryCMPForSingleSupport.containsNaN())
         computeEntryCMPForSupportFoot(cmp, supportSide, null, null);
      else
         cmp.setXYIncludingFrame(firstEntryCMPForSingleSupport);

      ReferenceFrame supportSoleFrame = soleZUpFrames.get(supportSide);
      cmp.changeFrame(supportSoleFrame);
      entryCMPs.get(constantCMPIndex).setIncludingFrame(cmp);
      computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootsteps.get(0));
      boolean isUpcomingFootstepLast = upcomingFootsteps.size() == 1;
      computeExitCMPForSupportFoot(cmp, supportSide, centroidOfUpcomingFootstep, isUpcomingFootstepLast);
      cmp.changeFrame(supportSoleFrame);
      exitCMPs.get(constantCMPIndex).setIncludingFrame(cmp);
      constantCMPIndex++;

      if (onlyOneUpcomingFootstep)
      {
         predictedSupportPolygon.clear(upcomingFootsteps.get(0).getSoleReferenceFrame());
         addPredictedContactPointsToPolygon(upcomingFootsteps.get(0), predictedSupportPolygon);
         predictedSupportPolygon.update();
         computeFinalCMPBetweenSupportFeet(constantCMPIndex, supportFootPolygonsInSoleZUpFrame.get(supportSide), predictedSupportPolygon);
         setRemainingCMPsToDuplicateLastComputedCMP(constantCMPIndex);
         return;
      }

      computeReferenceCMPsWithUpcomingFootsteps(supportSide, numberOfUpcomingFootsteps, constantCMPIndex);
      changeFrameOfCMPs(1, worldFrame);
   }

   private void computeReferenceCMPsWithUpcomingFootsteps(RobotSide firstSupportSide, int numberOfUpcomingFootsteps, int cmpIndex)
   {
      FramePoint2d centroidInSoleFrameOfPreviousSupportFoot = supportFootPolygonsInSoleZUpFrame.get(firstSupportSide).getCentroid();

      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         Footstep currentFootstep = upcomingFootsteps.get(i);
         computeFootstepCentroid(centroidOfCurrentFootstep, currentFootstep);

         FramePoint2d centroidOfNextFootstep = null;
         int indexOfUpcomingFootstep = i + 1;
         if (i < upcomingFootsteps.size() - 1)
         {
            computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootsteps.get(indexOfUpcomingFootstep));
            centroidOfNextFootstep = centroidOfUpcomingFootstep;
         }

         boolean isUpcomingFootstepLast = indexOfUpcomingFootstep >= upcomingFootsteps.size();
         if (isUpcomingFootstepLast)
         {
            predictedSupportPolygon.clear(currentFootstep.getSoleReferenceFrame());
            addPredictedContactPointsToPolygon(currentFootstep, predictedSupportPolygon);
            predictedSupportPolygon.update();
            computeFinalCMPBetweenSupportFeet(cmpIndex, supportFootPolygonsInSoleZUpFrame.get(firstSupportSide), predictedSupportPolygon);
         }
         else
         {
            computeExitCMPForFootstep(cmp, currentFootstep, centroidOfNextFootstep, isUpcomingFootstepLast);
            cmp.changeFrame(soleZUpFrames.get(firstSupportSide));
            exitCMPs.get(cmpIndex).setIncludingFrame(cmp);

            YoFramePoint previousExitCMP = exitCMPs.get(cmpIndex - 1);
            computeEntryCMPForFootstep(cmp, currentFootstep, centroidInSoleFrameOfPreviousSupportFoot, previousExitCMP);
            cmp.changeFrame(soleZUpFrames.get(firstSupportSide));
            entryCMPs.get(cmpIndex).setIncludingFrame(cmp);
         }

         cmpIndex++;
         centroidInSoleFrameOfPreviousSupportFoot = centroidOfCurrentFootstep;

         if (cmpIndex >= entryCMPs.size())
            break;
      }
      setRemainingCMPsToDuplicateLastComputedCMP(cmpIndex - 1);
   }

   private void setRemainingCMPsToDuplicateLastComputedCMP(int lastComputedCMPIndex)
   {
      for (int i = lastComputedCMPIndex + 1; i < entryCMPs.size(); i++)
         entryCMPs.get(i).setIncludingFrame(entryCMPs.get(lastComputedCMPIndex));
      for (int i = lastComputedCMPIndex + 1; i < exitCMPs.size(); i++)
         exitCMPs.get(i).setIncludingFrame(exitCMPs.get(lastComputedCMPIndex));
   }

   private void changeFrameOfCMPs(int fromIndex, ReferenceFrame desiredFrame)
   {
      for (int i = fromIndex; i < entryCMPs.size(); i++)
         entryCMPs.get(i).changeFrame(desiredFrame);
      for (int i = fromIndex; i < exitCMPs.size(); i++)
         exitCMPs.get(i).changeFrame(desiredFrame);
   }

   private void computeFootstepCentroid(FramePoint2d centroidToPack, Footstep footstep)
   {
      predictedSupportPolygon.clear(footstep.getSoleReferenceFrame());
      addPredictedContactPointsToPolygon(footstep, predictedSupportPolygon);
      predictedSupportPolygon.update();
      predictedSupportPolygon.getCentroid(centroidToPack);
   }

   private void computePredictedSupportCentroid(FramePoint2d centroidToPack, Footstep footstep, Footstep nextFootstep)
   {
      predictedSupportPolygon.clear(worldFrame);
      addPredictedContactPointsToPolygon(footstep, predictedSupportPolygon);
      addPredictedContactPointsToPolygon(nextFootstep, predictedSupportPolygon);
      predictedSupportPolygon.update();
      predictedSupportPolygon.getCentroid(centroidToPack);
   }

   private void addPredictedContactPointsToPolygon(Footstep footstep, FrameConvexPolygon2d convexPolygonToExtend)
   {
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();

      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         int numberOfContactPoints = predictedContactPoints.size();
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            tempFramePoint.setXYIncludingFrame(footstep.getSoleReferenceFrame(), predictedContactPoints.get(i));
            convexPolygonToExtend.addVertexByProjectionOntoXYPlane(tempFramePoint);
         }
      }
      else
      {
         ConvexPolygon2d defaultPolygon = defaultFootPolygons.get(footstep.getRobotSide());
         for (int i = 0; i < defaultPolygon.getNumberOfVertices(); i++)
         {
            tempFramePoint.setXYIncludingFrame(footstep.getSoleReferenceFrame(), defaultPolygon.getVertex(i));
            convexPolygonToExtend.addVertexByProjectionOntoXYPlane(tempFramePoint);
         }
      }
   }

   private void computeEntryCMPForSupportFoot(FramePoint entryCMPToPack, RobotSide robotSide, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot,
         YoFramePoint previousLateCMP)
   {
      ReferenceFrame soleFrame = soleZUpFrames.get(robotSide);
      tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleZUpFrame.get(robotSide));
      tempSupportPolygon.changeFrame(soleFrame);

      computeEntryCMP(entryCMPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfPreviousSupportFoot, previousLateCMP);
   }

   private void computeEntryCMPForFootstep(FramePoint entryCMPToPack, Footstep footstep, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot,
         YoFramePoint previousExitCMP)
   {
      ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      RobotSide robotSide = footstep.getRobotSide();

      if (predictedContactPoints != null)
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
      else
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

      computeEntryCMP(entryCMPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfPreviousSupportFoot, previousExitCMP);
   }

   private void computeEntryCMP(FramePoint entryCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FrameConvexPolygon2d footSupportPolygon, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot,
         YoFramePoint previousExitCMP)
   {
      if (useTwoCMPsPerSupport)
      {
         if (centroidInSoleFrameOfPreviousSupportFoot != null)
            centroidOfFootstepToConsider.setIncludingFrame(centroidInSoleFrameOfPreviousSupportFoot);
         else
            centroidOfFootstepToConsider.setToZero(soleFrame);
         centroidOfFootstepToConsider.changeFrameAndProjectToXYPlane(soleFrame);

         if (previousExitCMP != null)
         {
            previousExitCMP.getFrameTuple2dIncludingFrame(previousExitCMP2d);
            previousExitCMP2d.changeFrameAndProjectToXYPlane(soleFrame);
            // Choose the laziest option
            if (Math.abs(previousExitCMP2d.getX()) < Math.abs(centroidOfFootstepToConsider.getX()))
               centroidOfFootstepToConsider.set(previousExitCMP2d);
         }

         constrainCMPAccordingToSupportPolygonAndUserOffsets(cmp2d, footSupportPolygon, centroidOfFootstepToConsider, entryCMPUserOffsets.get(robotSide),
               minForwardEntryCMPOffset.getDoubleValue(), maxForwardEntryCMPOffset.getDoubleValue());
      }
      else
      {
         cmp2d.setIncludingFrame(footSupportPolygon.getCentroid());
         YoFrameVector2d offset = entryCMPUserOffsets.get(robotSide);
         cmp2d.add(offset.getX(), offset.getY());
      }

      entryCMPToPack.setXYIncludingFrame(cmp2d);
      entryCMPToPack.changeFrame(worldFrame);
   }

   private void computeExitCMPForSupportFoot(FramePoint exitCMPToPack, RobotSide robotSide, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot,
         boolean isUpcomingFootstepLast)
   {
      if (useTwoCMPsPerSupport)
      {
         ReferenceFrame soleFrame = soleZUpFrames.get(robotSide);
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleZUpFrame.get(robotSide));
         tempSupportPolygon.changeFrame(soleFrame);

         computeExitCMP(exitCMPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfUpcomingSupportFoot, isUpcomingFootstepLast);
      }
      else
      {
         exitCMPToPack.setToNaN(worldFrame);
      }
   }

   private void computeExitCMPForFootstep(FramePoint exitCMPToPack, Footstep footstep, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot,
         boolean isUpcomingFootstepLast)
   {
      if (useTwoCMPsPerSupport)
      {
         ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
         List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
         RobotSide robotSide = footstep.getRobotSide();

         if (predictedContactPoints != null)
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
         else
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

         computeExitCMP(exitCMPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfUpcomingSupportFoot, isUpcomingFootstepLast);
      }
      else
      {
         exitCMPToPack.setToNaN(worldFrame);
      }
   }

   private void computeExitCMP(FramePoint exitCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FrameConvexPolygon2d footSupportPolygon, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot,
         boolean isUpcomingFootstepLast)
   {
      if (centroidInSoleFrameOfUpcomingSupportFoot != null)
         centroidOfFootstepToConsider.setIncludingFrame(centroidInSoleFrameOfUpcomingSupportFoot);
      else
         centroidOfFootstepToConsider.setToZero(soleFrame);
      centroidOfFootstepToConsider.changeFrameAndProjectToXYPlane(soleFrame);

      boolean putCMPOnToes = false;

      if (!isUpcomingFootstepLast && centroidInSoleFrameOfUpcomingSupportFoot != null)
      {
         if (putExitCMPOnToes.getBooleanValue())
         {
            soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
            soleFrameOrigin.changeFrame(soleFrame);
            soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);

            putCMPOnToes = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCMPOnToes.getDoubleValue();
         }
         else if (useExitCMPOnToesForSteppingDown)
         {
            soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
            soleFrameOrigin.changeFrame(soleFrame);
            soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);
            boolean isSteppingForwardEnough = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCMPOnToesSteppingDown.getDoubleValue();
            soleToSoleFrameVector.changeFrame(worldFrame);
            boolean isSteppingDownEnough = soleToSoleFrameVector.getZ() < -footstepHeightThresholdToPutExitCMPOnToesSteppingDown.getDoubleValue();

            putCMPOnToes = isSteppingForwardEnough && isSteppingDownEnough;
         }
      }

      if (putCMPOnToes)
      {
         putExitCMPOnToes(footSupportPolygon, cmp2d);
      }
      else
      {
         constrainCMPAccordingToSupportPolygonAndUserOffsets(cmp2d, footSupportPolygon, centroidOfFootstepToConsider, exitCMPUserOffsets.get(robotSide),
               minForwardExitCMPOffset.getDoubleValue(), maxForwardExitCMPOffset.getDoubleValue());
      }

      exitCMPToPack.setXYIncludingFrame(cmp2d);
      exitCMPToPack.changeFrame(worldFrame);
   }

   private void putExitCMPOnToes(FrameConvexPolygon2d footSupportPolygon, FramePoint2d exitCMPToPack)
   {
      // Set x to have the CMP slightly inside the support polygon
      exitCMPToPack.setToZero(footSupportPolygon.getReferenceFrame());
      exitCMPToPack.setX(footSupportPolygon.getMaxX() - 1.6e-2);
      exitCMPToPack.setY(footSupportPolygon.getCentroid().getY());

      // Then constrain the computed CMP to be inside a safe support region
      tempSupportPolygonForShrinking.setIncludingFrameAndUpdate(footSupportPolygon);
      convexPolygonShrinker.shrinkConstantDistanceInto(tempSupportPolygonForShrinking, safeDistanceFromCMPToSupportEdgesWhenSteppingDown.getDoubleValue(),
            footSupportPolygon);

      footSupportPolygon.orthogonalProjection(exitCMPToPack);
   }

   private void constrainCMPAccordingToSupportPolygonAndUserOffsets(FramePoint2d cmpToPack, FrameConvexPolygon2d footSupportPolygon,
         FramePoint2d centroidOfFootstepToConsider, YoFrameVector2d cmpOffset, double minForwardCMPOffset, double maxForwardCMPOffset)
   {
      // First constrain the computed CMP to the given min/max along the x-axis.
      FramePoint2d footSupportCentroid = footSupportPolygon.getCentroid();
      double cmpXOffsetFromCentroid = stepLengthToCMPOffsetFactor.getDoubleValue() * (centroidOfFootstepToConsider.getX() - footSupportCentroid.getX()) + cmpOffset.getX();
      cmpXOffsetFromCentroid = MathTools.clipToMinMax(cmpXOffsetFromCentroid, minForwardCMPOffset, maxForwardCMPOffset);

      cmpToPack.setIncludingFrame(footSupportCentroid);
      cmpToPack.add(cmpXOffsetFromCentroid, cmpOffset.getY());

      // Then constrain the computed CMP to be inside a safe support region
      tempSupportPolygonForShrinking.setIncludingFrameAndUpdate(footSupportPolygon);
      convexPolygonShrinker.shrinkConstantDistanceInto(tempSupportPolygonForShrinking, safeDistanceFromCMPToSupportEdges.getDoubleValue(), footSupportPolygon);

      footSupportPolygon.orthogonalProjection(cmpToPack);
   }

   private final FramePoint2d tempCentroid = new FramePoint2d();
   private final FramePoint tempCentroid3d = new FramePoint();
   private final FrameConvexPolygon2d tempFootPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d upcomingSupport = new FrameConvexPolygon2d();
   private void computeFinalCMPBetweenSupportFeet(int cmpIndex, FrameConvexPolygon2d footA, FrameConvexPolygon2d footB)
   {
      footA.getCentroid(tempCentroid);
      firstCMP.setXYIncludingFrame(tempCentroid);
      firstCMP.changeFrame(worldFrame);

      footB.getCentroid(tempCentroid);
      secondCMP.setXYIncludingFrame(tempCentroid);
      secondCMP.changeFrame(worldFrame);

      upcomingSupport.clear(worldFrame);
      tempFootPolygon.setIncludingFrame(footA);
      tempFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      upcomingSupport.addVertices(tempFootPolygon);
      tempFootPolygon.setIncludingFrame(footB);
      tempFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      upcomingSupport.addVertices(tempFootPolygon);
      upcomingSupport.update();

      entryCMPs.get(cmpIndex).switchCurrentReferenceFrame(worldFrame);
      exitCMPs.get(cmpIndex).switchCurrentReferenceFrame(worldFrame);

      upcomingSupport.getCentroid(tempCentroid);
      tempCentroid3d.setXYIncludingFrame(tempCentroid);

      double chicken = MathTools.clipToMinMax(percentageChickenSupport.getDoubleValue(), 0.0, 1.0);
      if (chicken <= 0.5)
         entryCMPs.get(cmpIndex).interpolate(firstCMP, tempCentroid3d, chicken * 2.0);
      else
         entryCMPs.get(cmpIndex).interpolate(tempCentroid3d, secondCMP, (chicken-0.5) * 2.0);
      exitCMPs.get(cmpIndex).set(entryCMPs.get(cmpIndex));
   }

   public List<YoFramePoint> getEntryCMPs()
   {
      return entryCMPsInWorldFrameReadOnly;
   }

   public List<YoFramePoint> getExitCMPs()
   {
      return exitCMPsInWorldFrameReadOnly;
   }

   public YoFramePoint getNextEntryCMP()
   {
      return entryCMPsInWorldFrameReadOnly.get(0);
   }

   public void getNextEntryCMP(FramePoint entryCMPToPack)
   {
      entryCMPsInWorldFrameReadOnly.get(0).getFrameTupleIncludingFrame(entryCMPToPack);
   }

   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      exitCMPsInWorldFrameReadOnly.get(0).getFrameTupleIncludingFrame(entryCMPToPack);
   }

   public boolean isDoneWalking()
   {
      return isDoneWalking.getBooleanValue();
   }

   public void setCarefulFootholdPercentage(double percentage)
   {
      percentageChickenSupport.set(MathTools.clipToMinMax(percentage, 0.0, 1.0));
   }
}
