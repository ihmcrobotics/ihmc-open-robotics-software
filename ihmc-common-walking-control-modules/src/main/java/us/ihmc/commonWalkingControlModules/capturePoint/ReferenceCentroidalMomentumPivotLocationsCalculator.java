package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
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

   private final YoBoolean isDoneWalking;
   private final YoDouble maxForwardEntryCMPOffset;
   private final YoDouble minForwardEntryCMPOffset;
   private final YoDouble maxForwardExitCMPOffset;
   private final YoDouble minForwardExitCMPOffset;
   private final YoDouble footstepHeightThresholdToPutExitCMPOnToesSteppingDown;
   private final YoDouble footstepLengthThresholdToPutExitCMPOnToesSteppingDown;

   private final YoDouble stepLengthToCMPOffsetFactor;
   private final YoDouble footstepLengthThresholdToPutExitCMPOnToes;

   private final YoBoolean putExitCMPOnToes;
   private final YoDouble exitCMPForwardSafetyMarginOnToes;

   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> soleZUpFrames;
   private final FrameConvexPolygon2d predictedSupportPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleZUpFrame = new SideDependentList<>();

   private final SideDependentList<YoFrameVector2d> entryCMPUserOffsets = new SideDependentList<>();
   private final SideDependentList<YoFrameVector2d> exitCMPUserOffsets = new SideDependentList<>();

   private final YoInteger numberOfUpcomingFootsteps;
   private final List<Footstep> upcomingFootsteps = new ArrayList<>();

   private final FramePoint3D cmp = new FramePoint3D();
   private final FramePoint3D firstCMP = new FramePoint3D();
   private final FramePoint3D secondCMP = new FramePoint3D();

   private final FramePoint3D soleFrameOrigin = new FramePoint3D();
   private final FrameVector3D soleToSoleFrameVector = new FrameVector3D();

   private final FramePoint2D cmp2d = new FramePoint2D();
   private final FramePoint2D previousExitCMP2d = new FramePoint2D();
   private final FramePoint2D firstEntryCMPForSingleSupport = new FramePoint2D();
   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();
   private final FrameConvexPolygon2d tempSupportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempSupportPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   private final YoDouble safeDistanceFromCMPToSupportEdges;
   private final YoDouble safeDistanceFromCMPToSupportEdgesWhenSteppingDown;

   private final FramePoint2D centroidOfUpcomingFootstep = new FramePoint2D();
   private final FramePoint2D centroidOfCurrentFootstep = new FramePoint2D();
   private final FramePoint2D centroidOfFootstepToConsider = new FramePoint2D();

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private boolean useTwoCMPsPerSupport = false;
   private boolean useExitCMPOnToesForSteppingDown = false;

   /**
    * By default the CMPs for the last step are centered between the foot support polygon centroids. This parameter (default 0.5)
    * specifies where on the line connecting the centroids the CMPs are placed.
    */
   private final YoDouble percentageChickenSupport;

   public ReferenceCentroidalMomentumPivotLocationsCalculator(String namePrefix, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, int numberFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      firstEntryCMPForSingleSupport.setToNaN();

      isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);
      maxForwardEntryCMPOffset = new YoDouble(namePrefix + "MaxForwardEntryCMPOffset", registry);
      minForwardEntryCMPOffset = new YoDouble(namePrefix + "MinForwardEntryCMPOffset", registry);
      maxForwardExitCMPOffset = new YoDouble(namePrefix + "MaxForwardExitCMPOffset", registry);
      minForwardExitCMPOffset = new YoDouble(namePrefix + "MinForwardExitCMPOffset", registry);
      safeDistanceFromCMPToSupportEdges = new YoDouble(namePrefix + "SafeDistanceFromCMPToSupportEdges", registry);
      footstepHeightThresholdToPutExitCMPOnToesSteppingDown = new YoDouble(namePrefix + "FootstepHeightThresholdToPutExitCMPOnToesSteppingDown", registry);
      footstepLengthThresholdToPutExitCMPOnToesSteppingDown = new YoDouble(namePrefix + "FootstepLengthThresholdToPutExitCMPOnToesSteppingDown", registry);
      safeDistanceFromCMPToSupportEdgesWhenSteppingDown = new YoDouble(namePrefix + "SafeDistanceFromCMPToSupportEdgesWhenSteppingDown", registry);

      stepLengthToCMPOffsetFactor = new YoDouble(namePrefix + "StepLengthToCMPOffsetFactor", registry);
      footstepLengthThresholdToPutExitCMPOnToes = new YoDouble(namePrefix + "FootstepLengthThresholdToPutExitCMPOnToes", registry);

      numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);

      putExitCMPOnToes = new YoBoolean(namePrefix + "PutExitCMPOnToes", registry);
      exitCMPForwardSafetyMarginOnToes = new YoDouble(namePrefix + "ExitCMPForwardSafetyMarginOnToes", registry);

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

      percentageChickenSupport = new YoDouble("PercentageChickenSupport", registry);
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

   public void initializeParameters(CoPPlannerParameters icpPlannerParameters)
   {
      useTwoCMPsPerSupport = icpPlannerParameters.getNumberOfCoPWayPointsPerFoot() > 1;
      safeDistanceFromCMPToSupportEdges.set(icpPlannerParameters.getCoPSafeDistanceAwayFromSupportEdges());

      CoPPointName exitCoPName = icpPlannerParameters.getExitCoPName();
      CoPPointName entryCoPName = icpPlannerParameters.getEntryCoPName();

      EnumMap<CoPPointName, Vector2D> cmpForwardOffsetBounds = icpPlannerParameters.getCoPForwardOffsetBoundsInFoot();
      Vector2D entryCMPForwardOffsetBounds = cmpForwardOffsetBounds.get(entryCoPName);
      Vector2D exitCMPForwardOffsetBounds = cmpForwardOffsetBounds.get(exitCoPName);

      minForwardEntryCMPOffset.set(entryCMPForwardOffsetBounds.getX());
      maxForwardEntryCMPOffset.set(entryCMPForwardOffsetBounds.getY());

      minForwardExitCMPOffset.set(exitCMPForwardOffsetBounds.getX());
      maxForwardExitCMPOffset.set(exitCMPForwardOffsetBounds.getY());

      stepLengthToCMPOffsetFactor.set(icpPlannerParameters.getStepLengthToCoPOffsetFactors().get(exitCoPName));

      EnumMap<CoPPointName, Vector2D> cmpOffsets = icpPlannerParameters.getCoPOffsetsInFootFrame();
      Vector2D entryCMPOffsets = cmpOffsets.get(entryCoPName);
      Vector2D exitCMPOffsets = cmpOffsets.get(exitCoPName);
      double entryCMPForwardOffset = entryCMPOffsets.getX();
      double entryCMPInsideOffset = entryCMPOffsets.getY();
      double exitCMPForwardOffset = exitCMPOffsets.getX();
      double exitCMPInsideOffset = exitCMPOffsets.getY();

      setSymmetricEntryCMPConstantOffsets(entryCMPForwardOffset, entryCMPInsideOffset);
      setSymmetricExitCMPConstantOffsets(exitCMPForwardOffset, exitCMPInsideOffset);

      useExitCMPOnToesForSteppingDown = icpPlannerParameters.useExitCoPOnToesForSteppingDown();
      footstepHeightThresholdToPutExitCMPOnToesSteppingDown.set(icpPlannerParameters.getStepHeightThresholdForExitCoPOnToesWhenSteppingDown());
      footstepLengthThresholdToPutExitCMPOnToesSteppingDown.set(icpPlannerParameters.getStepLengthThresholdForExitCoPOnToesWhenSteppingDown());
      safeDistanceFromCMPToSupportEdgesWhenSteppingDown.set(icpPlannerParameters.getCoPSafeDistanceAwayFromToesWhenSteppingDown());

      putExitCMPOnToes.set(icpPlannerParameters.putExitCoPOnToes());
      exitCMPForwardSafetyMarginOnToes.set(icpPlannerParameters.getExitCoPForwardSafetyMarginOnToes());
      footstepLengthThresholdToPutExitCMPOnToes.set(icpPlannerParameters.getStepLengthThresholdForExitCoPOnToes());
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
      {
         if (!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
            upcomingFootsteps.add(footstep);
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }
   }

   public int getNumberOfFootstepRegistered()
   {
      return upcomingFootsteps.size();
   }

   public void computeReferenceCMPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      int cmpIndex = computeSupportFeetReferenceCMPsDuringDoubleSupport(atAStop, transferToSide);
      int numberOfUpcomingFootsteps = this.numberOfUpcomingFootsteps.getIntegerValue();
      boolean noUpcomingFootsteps = numberOfUpcomingFootsteps == 0;
      if (noUpcomingFootsteps)
         return;

      computeReferenceCMPsWithUpcomingFootsteps(transferToSide, numberOfUpcomingFootsteps, cmpIndex);
      changeFrameOfCMPs(2, worldFrame);
   }

   public int computeSupportFeetReferenceCMPsDuringDoubleSupport(boolean atAStop, RobotSide transferToSide)
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
            return cmpIndex;
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
      firstEntryCMPForSingleSupport.setIncludingFrame(cmp);
      computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootsteps.get(0));
      boolean isUpcomingFootstepLast = upcomingFootsteps.size() == 1;
      computeExitCMPForSupportFoot(cmp, transferToSide, centroidOfUpcomingFootstep, isUpcomingFootstepLast);
      cmp.changeFrame(transferToSoleFrame);
      exitCMPs.get(cmpIndex).setIncludingFrame(cmp);
      cmpIndex++;

      return cmpIndex;
   }

   public void computeReferenceCMPsStartingFromSingleSupport(RobotSide supportSide)
   {
      boolean computeMore = computeSupportFootReferenceCMPsDuringSingleSupport(supportSide);

      if (!computeMore)
         return;

      int constantCMPIndex = 1;
      int numberOfUpcomingFootsteps = this.numberOfUpcomingFootsteps.getIntegerValue();
      computeReferenceCMPsWithUpcomingFootsteps(supportSide, numberOfUpcomingFootsteps, constantCMPIndex);
      changeFrameOfCMPs(1, worldFrame);
   }

   public boolean computeSupportFootReferenceCMPsDuringSingleSupport(RobotSide supportSide)
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
         cmp.setIncludingFrame(firstEntryCMPForSingleSupport, 0.0);

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
         return false;
      }

      return true;
   }



   private void computeReferenceCMPsWithUpcomingFootsteps(RobotSide firstSupportSide, int numberOfUpcomingFootsteps, int cmpIndex)
   {
      FramePoint2D centroidInSoleFrameOfPreviousSupportFoot = supportFootPolygonsInSoleZUpFrame.get(firstSupportSide).getCentroid();

      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         Footstep currentFootstep = upcomingFootsteps.get(i);
         computeFootstepCentroid(centroidOfCurrentFootstep, currentFootstep);

         FramePoint2D centroidOfNextFootstep = null;
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

   private void computeFootstepCentroid(FramePoint2D centroidToPack, Footstep footstep)
   {
      predictedSupportPolygon.clear(footstep.getSoleReferenceFrame());
      addPredictedContactPointsToPolygon(footstep, predictedSupportPolygon);
      predictedSupportPolygon.update();
      predictedSupportPolygon.getCentroid(centroidToPack);
   }

   private void computePredictedSupportCentroid(FramePoint2D centroidToPack, Footstep footstep, Footstep nextFootstep)
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
            tempFramePoint.setIncludingFrame(footstep.getSoleReferenceFrame(), predictedContactPoints.get(i), 0.0);
            convexPolygonToExtend.addVertexByProjectionOntoXYPlane(tempFramePoint);
         }
      }
      else
      {
         ConvexPolygon2D defaultPolygon = defaultFootPolygons.get(footstep.getRobotSide());
         for (int i = 0; i < defaultPolygon.getNumberOfVertices(); i++)
         {
            tempFramePoint.setIncludingFrame(footstep.getSoleReferenceFrame(), defaultPolygon.getVertex(i), 0.0);
            convexPolygonToExtend.addVertexByProjectionOntoXYPlane(tempFramePoint);
         }
      }
   }

   private void computeEntryCMPForSupportFoot(FramePoint3D entryCMPToPack, RobotSide robotSide, FramePoint2D centroidInSoleFrameOfPreviousSupportFoot,
         YoFramePoint previousLateCMP)
   {
      ReferenceFrame soleFrame = soleZUpFrames.get(robotSide);
      tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleZUpFrame.get(robotSide));
      tempSupportPolygon.changeFrame(soleFrame);

      computeEntryCMP(entryCMPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfPreviousSupportFoot, previousLateCMP);
   }

   private void computeEntryCMPForFootstep(FramePoint3D entryCMPToPack, Footstep footstep, FramePoint2D centroidInSoleFrameOfPreviousSupportFoot,
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

   private void computeEntryCMP(FramePoint3D entryCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FrameConvexPolygon2d footSupportPolygon, FramePoint2D centroidInSoleFrameOfPreviousSupportFoot,
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

      entryCMPToPack.setIncludingFrame(cmp2d, 0.0);
      entryCMPToPack.changeFrame(worldFrame);
   }

   private void computeExitCMPForSupportFoot(FramePoint3D exitCMPToPack, RobotSide robotSide, FramePoint2D centroidInSoleFrameOfUpcomingSupportFoot,
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

   private void computeExitCMPForFootstep(FramePoint3D exitCMPToPack, Footstep footstep, FramePoint2D centroidInSoleFrameOfUpcomingSupportFoot,
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

   private void computeExitCMP(FramePoint3D exitCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FrameConvexPolygon2d footSupportPolygon, FramePoint2D centroidInSoleFrameOfUpcomingSupportFoot,
         boolean isUpcomingFootstepLast)
   {
      if (centroidInSoleFrameOfUpcomingSupportFoot != null)
         centroidOfFootstepToConsider.setIncludingFrame(centroidInSoleFrameOfUpcomingSupportFoot);
      else
         centroidOfFootstepToConsider.setToZero(soleFrame);
      centroidOfFootstepToConsider.changeFrameAndProjectToXYPlane(soleFrame);

      boolean polygonIsAPoint = footSupportPolygon.getArea() == 0.0;
      boolean putCMPOnToesWalking = false;
      boolean putCMPOnToesSteppingDown = false;

      if (!isUpcomingFootstepLast && centroidInSoleFrameOfUpcomingSupportFoot != null && !polygonIsAPoint)
      {
         soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
         soleFrameOrigin.changeFrame(soleFrame);
         soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);
         boolean isSteppingForwardEnough = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCMPOnToesSteppingDown.getDoubleValue();
         soleToSoleFrameVector.changeFrame(worldFrame);
         boolean isSteppingDownEnough = soleToSoleFrameVector.getZ() < -footstepHeightThresholdToPutExitCMPOnToesSteppingDown.getDoubleValue();

         if (isSteppingDownEnough)
         {
            putCMPOnToesSteppingDown = isSteppingForwardEnough && isSteppingDownEnough;
         }
         else if (putExitCMPOnToes.getBooleanValue())
         {
            soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
            soleFrameOrigin.changeFrame(soleFrame);
            soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);

            putCMPOnToesWalking = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCMPOnToes.getDoubleValue();
         }
      }


      if (polygonIsAPoint)
      {
         cmp2d.setToZero(footSupportPolygon.getReferenceFrame());
         cmp2d.set(footSupportPolygon.getVertex(0));
      }
      else if (putCMPOnToesWalking)
      {
         constrainCMPAccordingToSupportPolygonAndUserOffsets(cmp2d, footSupportPolygon, centroidOfFootstepToConsider, exitCMPUserOffsets.get(robotSide),
                                                             minForwardExitCMPOffset.getDoubleValue(), footSupportPolygon.getMaxX(),
                                                             exitCMPForwardSafetyMarginOnToes.getDoubleValue());
      }
      else if (putCMPOnToesSteppingDown)
      {
         putExitCMPOnToes(cmp2d, footSupportPolygon, 0.0);
      }
      else
      {
         constrainCMPAccordingToSupportPolygonAndUserOffsets(cmp2d, footSupportPolygon, centroidOfFootstepToConsider, exitCMPUserOffsets.get(robotSide),
               minForwardExitCMPOffset.getDoubleValue(), maxForwardExitCMPOffset.getDoubleValue());
      }

      exitCMPToPack.setIncludingFrame(cmp2d, 0.0);
      exitCMPToPack.changeFrame(worldFrame);
   }

   private void putExitCMPOnToes(FramePoint2D exitCMPToPack, FrameConvexPolygon2d footSupportPolygon, double exitCMPInsideOffset)
   {
      // Set x to have the CMP slightly inside the support polygon
      exitCMPToPack.setToZero(footSupportPolygon.getReferenceFrame());
      exitCMPToPack.setX(footSupportPolygon.getMaxX() - exitCMPForwardSafetyMarginOnToes.getDoubleValue());
      exitCMPToPack.setY(footSupportPolygon.getCentroid().getY() + exitCMPInsideOffset);

      // Then constrain the computed CMP to be inside a safe support region
      tempSupportPolygonForShrinking.setIncludingFrameAndUpdate(footSupportPolygon);
      convexPolygonShrinker.scaleConvexPolygon(tempSupportPolygonForShrinking, safeDistanceFromCMPToSupportEdgesWhenSteppingDown.getDoubleValue(),
            footSupportPolygon);

      footSupportPolygon.orthogonalProjection(exitCMPToPack);
   }

   private void constrainCMPAccordingToSupportPolygonAndUserOffsets(FramePoint2D cmpToPack, FrameConvexPolygon2d footSupportPolygon,
                                                                    FramePoint2D centroidOfFootstepToConsider, YoFrameVector2d cmpOffset,
                                                                    double minForwardCMPOffset, double maxForwardCMPOffset)
   {
      constrainCMPAccordingToSupportPolygonAndUserOffsets(cmpToPack, footSupportPolygon, centroidOfFootstepToConsider, cmpOffset, minForwardCMPOffset,
                                                          maxForwardCMPOffset, safeDistanceFromCMPToSupportEdges.getDoubleValue());
   }

   private void constrainCMPAccordingToSupportPolygonAndUserOffsets(FramePoint2D cmpToPack, FrameConvexPolygon2d footSupportPolygon,
                                                                    FramePoint2D centroidOfFootstepToConsider, YoFrameVector2d cmpOffset,
                                                                    double minForwardCMPOffset, double maxForwardCMPOffset, double safeDistanceFromCMPToSupportEdges)
   {
      // First constrain the computed CMP to the given min/max along the x-axis.
      FramePoint2D footSupportCentroid = footSupportPolygon.getCentroid();
      double cmpXOffsetFromCentroid = stepLengthToCMPOffsetFactor.getDoubleValue() * (centroidOfFootstepToConsider.getX() - footSupportCentroid.getX()) + cmpOffset.getX();
      cmpXOffsetFromCentroid = MathTools.clamp(cmpXOffsetFromCentroid, minForwardCMPOffset, maxForwardCMPOffset);

      cmpToPack.setIncludingFrame(footSupportCentroid);
      cmpToPack.add(cmpXOffsetFromCentroid, cmpOffset.getY());

      // Then constrain the computed CMP to be inside a safe support region
      tempSupportPolygonForShrinking.setIncludingFrameAndUpdate(footSupportPolygon);
      convexPolygonShrinker.scaleConvexPolygon(tempSupportPolygonForShrinking, safeDistanceFromCMPToSupportEdges, footSupportPolygon);

      footSupportPolygon.orthogonalProjection(cmpToPack);
   }

   private final FramePoint2D tempCentroid = new FramePoint2D();
   private final FramePoint3D tempCentroid3d = new FramePoint3D();
   private final FrameConvexPolygon2d tempFootPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d upcomingSupport = new FrameConvexPolygon2d();
   private void computeFinalCMPBetweenSupportFeet(int cmpIndex, FrameConvexPolygon2d footA, FrameConvexPolygon2d footB)
   {
      footA.getCentroid(tempCentroid);
      firstCMP.setIncludingFrame(tempCentroid, 0.0);
      firstCMP.changeFrame(worldFrame);

      footB.getCentroid(tempCentroid);
      secondCMP.setIncludingFrame(tempCentroid, 0.0);
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
      tempCentroid3d.setIncludingFrame(tempCentroid, 0.0);

      double chicken = MathTools.clamp(percentageChickenSupport.getDoubleValue(), 0.0, 1.0);
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

   public void getNextEntryCMP(FramePoint3D entryCMPToPack)
   {
      entryCMPsInWorldFrameReadOnly.get(0).getFrameTupleIncludingFrame(entryCMPToPack);
   }

   public void getNextExitCMP(FramePoint3D entryCMPToPack)
   {
      exitCMPsInWorldFrameReadOnly.get(0).getFrameTupleIncludingFrame(entryCMPToPack);
   }

   public boolean isDoneWalking()
   {
      return isDoneWalking.getBooleanValue();
   }

   public void setCarefulFootholdPercentage(double percentage)
   {
      percentageChickenSupport.set(MathTools.clamp(percentage, 0.0, 1.0));
   }
}
