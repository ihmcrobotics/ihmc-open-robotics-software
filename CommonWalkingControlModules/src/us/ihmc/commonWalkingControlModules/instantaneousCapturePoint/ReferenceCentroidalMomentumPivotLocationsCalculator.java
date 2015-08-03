package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class ReferenceCentroidalMomentumPivotLocationsCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double CMP_POINT_SIZE = 0.005;

   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   /**
    * <li> When using only one CMP per support:
    * Desired constant CMP locations for the entire single support phases. </li>
    * <li> When using two CMPs per support:
    * Desired CMP locations on the feet in the early single support phase. </li>
    * */
   private final ArrayList<YoFramePointInMultipleFrames> entryCMPs = new ArrayList<YoFramePointInMultipleFrames>();
   private final ArrayList<YoFramePoint> entryCMPsInWorldFrameReadOnly = new ArrayList<YoFramePoint>();

   /**
    * Only used when computing two CMPs per support.
    * Desired CMP locations on the feet in the late single support phase.
    */
   private final ArrayList<YoFramePointInMultipleFrames> exitCMPs = new ArrayList<YoFramePointInMultipleFrames>();
   private final ArrayList<YoFramePoint> exitCMPsInWorldFrameReadOnly = new ArrayList<YoFramePoint>();
   
   private final BooleanYoVariable isDoneWalking;
   private final DoubleYoVariable maxForwardEntryCMPOffset;
   private final DoubleYoVariable minForwardEntryCMPOffset;
   private final DoubleYoVariable maxForwardExitCMPOffset; 
   private final DoubleYoVariable minForwardExitCMPOffset; 
   private final DoubleYoVariable footstepHeightThresholdToPutExitCMPOnToes;
   private final DoubleYoVariable footstepLengthThresholdToPutExitCMPOnToes;

   private final DoubleYoVariable stepLengthToCMPOffsetFactor;

   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final FrameConvexPolygon2d supportPolygon;
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleFrame = new SideDependentList<>();

   private final SideDependentList<YoFrameVector2d> entryCMPUserOffsets = new SideDependentList<>();
   private final SideDependentList<YoFrameVector2d> exitCMPUserOffsets = new SideDependentList<>();

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();

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
   private final FramePoint2d centroidOfPreviousFootstep = new FramePoint2d();
   private final FramePoint2d centroidOfFootstepToConsider = new FramePoint2d();

   private boolean useTwoCMPsPerSupport = false;
   private boolean useExitCMPOnToesForSteppingDown = false;

   public ReferenceCentroidalMomentumPivotLocationsCalculator(String namePrefix, BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         int numberFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      firstEntryCMPForSingleSupport.setToNaN();

      isDoneWalking = new BooleanYoVariable(namePrefix + "IsDoneWalking", registry);
      maxForwardEntryCMPOffset = new DoubleYoVariable(namePrefix + "MaxForwardEntryCMPOffset", registry);
      minForwardEntryCMPOffset = new DoubleYoVariable(namePrefix + "MinForwardEntryCMPOffset", registry);
      maxForwardExitCMPOffset = new DoubleYoVariable(namePrefix + "MaxForwardExitCMPOffset", registry);
      minForwardExitCMPOffset = new DoubleYoVariable(namePrefix + "MinForwardExitCMPOffset", registry);
      safeDistanceFromCMPToSupportEdges = new DoubleYoVariable(namePrefix + "SafeDistanceFromCMPToSupportEdges", registry);
      footstepHeightThresholdToPutExitCMPOnToes = new DoubleYoVariable(namePrefix + "FootstepHeightThresholdToPutExitCMPOnToes", registry);
      footstepLengthThresholdToPutExitCMPOnToes = new DoubleYoVariable(namePrefix + "FootstepLengthThresholdToPutExitCMPOnToes", registry);
      safeDistanceFromCMPToSupportEdgesWhenSteppingDown = new DoubleYoVariable(namePrefix + "SafeDistanceFromCMPToSupportEdgesWhenSteppingDown", registry);

      stepLengthToCMPOffsetFactor = new DoubleYoVariable(namePrefix + "StepLengthToCMPOffsetFactor", registry);

      supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();

      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.put(robotSide, contactableFeet.get(robotSide).getSoleFrame());
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon.getConvexPolygon2d());

         String sidePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         YoFrameVector2d entryCMPUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "EntryCMPConstantOffsets", null, registry);
         entryCMPUserOffsets.put(robotSide, entryCMPUserOffset);
         YoFrameVector2d exitCMPUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "ExitCMPConstantOffsets", null, registry);
         exitCMPUserOffsets.put(robotSide, exitCMPUserOffset);
         supportFootPolygonsInSoleFrame.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide));
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleFrame.get(robotSide)); // Just to allocate memory
      }

      midFeetZUpFrame = bipedSupportPolygons.getMidFeetZUpFrame();
      ReferenceFrame[] framesToRegister = new ReferenceFrame[]{worldFrame, midFeetZUpFrame, soleFrames.get(RobotSide.LEFT), soleFrames.get(RobotSide.RIGHT)};

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
      footstepHeightThresholdToPutExitCMPOnToes.set(icpPlannerParameters.getStepHeightThresholdForExitCMPOnToesWhenSteppingDown());
      footstepLengthThresholdToPutExitCMPOnToes.set(icpPlannerParameters.getStepLengthThresholdForExitCMPOnToesWhenSteppingDown());
      safeDistanceFromCMPToSupportEdgesWhenSteppingDown.set(icpPlannerParameters.getCMPSafeDistanceAwayFromToesWhenSteppingDown());
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
         YoGraphicPosition entryCMPViz = new YoGraphicPosition("Entry CMP" + i, entryCMPsInWorldFrameReadOnly.get(i), CMP_POINT_SIZE, YoAppearance.Green(), GraphicType.SOLID_BALL);
         yoGraphicsList.add(entryCMPViz);
         artifactList.add(entryCMPViz.createArtifact());
      }

      for (int i = 0; i < exitCMPs.size(); i++)
      {
         YoGraphicPosition exitCMPViz = new YoGraphicPosition("Exit CMP" + i, exitCMPsInWorldFrameReadOnly.get(i), CMP_POINT_SIZE, YoAppearance.Green(), GraphicType.BALL);
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

   public void computeReferenceCMPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      RobotSide transferFromSide = transferToSide.getOppositeSide();
      int numberOfUpcomingFootsteps = upcomingFootsteps.size();
      int cmpIndex = 0;
      boolean noUpcomingFootsteps = numberOfUpcomingFootsteps == 0;
      isDoneWalking.set(noUpcomingFootsteps);
      ReferenceFrame transferToSoleFrame = soleFrames.get(transferToSide);
      ReferenceFrame transferFromSoleFrame = soleFrames.get(transferFromSide);

      if (atAStop || noUpcomingFootsteps)
      {
         entryCMPs.get(cmpIndex).setXYIncludingFrame(supportPolygon.getCentroid());
         exitCMPs.get(cmpIndex).setXYIncludingFrame(supportPolygon.getCentroid());
         
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
         computeExitCMPForSupportFoot(cmp, transferFromSide, supportFootPolygonsInSoleFrame.get(transferToSide).getCentroid(), isUpcomingFootstepLast);
         cmp.changeFrame(transferFromSoleFrame);
         exitCMPs.get(cmpIndex).setIncludingFrame(cmp);
         cmpIndex++;
      }

      computeEntryCMPForSupportFoot(cmp, transferToSide, supportFootPolygonsInSoleFrame.get(transferFromSide).getCentroid(), exitCMPs.get(cmpIndex - 1));
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
      int constantCMPIndex = 0;
      boolean onlyOneUpcomingFootstep = numberOfUpcomingFootsteps == 1;
      isDoneWalking.set(onlyOneUpcomingFootstep);

      // Can happen if this method is the first to be called, so it should pretty much never happen.
      if (firstEntryCMPForSingleSupport.containsNaN())
         computeEntryCMPForSupportFoot(cmp, supportSide, null, null);
      else
         cmp.setXYIncludingFrame(firstEntryCMPForSingleSupport);

      ReferenceFrame supportSoleFrame = soleFrames.get(supportSide);
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
         firstCMP.setXYIncludingFrame(supportFootPolygonsInSoleFrame.get(supportSide).getCentroid());
         computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootsteps.get(0));
         secondCMP.setXYIncludingFrame(centroidOfUpcomingFootstep);
         firstCMP.changeFrame(supportSoleFrame);
         secondCMP.changeFrame(supportSoleFrame);
         entryCMPs.get(constantCMPIndex).switchCurrentReferenceFrame(supportSoleFrame);
         exitCMPs.get(constantCMPIndex).switchCurrentReferenceFrame(supportSoleFrame);
         entryCMPs.get(constantCMPIndex).interpolate(firstCMP, secondCMP, 0.5);
         exitCMPs.get(constantCMPIndex).interpolate(firstCMP, secondCMP, 0.5);
         setRemainingCMPsToDuplicateLastComputedCMP(constantCMPIndex);
         return;
      }

      computeReferenceCMPsWithUpcomingFootsteps(supportSide, numberOfUpcomingFootsteps, constantCMPIndex);
      changeFrameOfCMPs(1, worldFrame);
   }

   private void computeReferenceCMPsWithUpcomingFootsteps(RobotSide firstSupportSide, int numberOfUpcomingFootsteps, int cmpIndex)
   {
      FramePoint2d centroidInSoleFrameOfPreviousSupportFoot = supportFootPolygonsInSoleFrame.get(firstSupportSide).getCentroid();

      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         Footstep currentFootstep = upcomingFootsteps.get(i);
         FramePoint2d centroidOfNextFootstep = null;
         int indexOfUpcomingFootstep = i + 1;
         if (i < upcomingFootsteps.size() - 1)
         {
            computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootsteps.get(indexOfUpcomingFootstep));
            centroidOfNextFootstep = centroidOfUpcomingFootstep;
         }
         
         boolean isUpcomingFootstepLast = indexOfUpcomingFootstep >= upcomingFootsteps.size();
         computeExitCMPForFootstep(cmp, currentFootstep, centroidOfNextFootstep, isUpcomingFootstepLast);
         cmp.changeFrame(soleFrames.get(firstSupportSide));
         exitCMPs.get(cmpIndex).setIncludingFrame(cmp);

         YoFramePoint previousExitCMP = exitCMPs.get(cmpIndex - 1);
         computeEntryCMPForFootstep(cmp, currentFootstep, centroidInSoleFrameOfPreviousSupportFoot, previousExitCMP);
         cmp.changeFrame(soleFrames.get(firstSupportSide));
         entryCMPs.get(cmpIndex).setIncludingFrame(cmp);
         
         cmpIndex++;
         computeFootstepCentroid(centroidOfPreviousFootstep, currentFootstep);
         centroidInSoleFrameOfPreviousSupportFoot = centroidOfPreviousFootstep;

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
      List<Point2d> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         centroidToPack.setToZero(footstep.getSoleReferenceFrame());
         int numberOfContactPoints = predictedContactPoints.size();
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            centroidToPack.setX(centroidToPack.getX() + predictedContactPoints.get(i).getX() / numberOfContactPoints);
            centroidToPack.setY(centroidToPack.getY() + predictedContactPoints.get(i).getY() / numberOfContactPoints);
         }
      }
      else
      {
         centroidToPack.setIncludingFrame(footstep.getSoleReferenceFrame(), defaultFootPolygons.get(footstep.getRobotSide()).getCentroid());
      }
   }

   private void computeEntryCMPForSupportFoot(FramePoint entryCMPToPack, RobotSide robotSide, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot, YoFramePoint previousLateCMP)
   {
      ReferenceFrame soleFrame = soleFrames.get(robotSide);
      tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleFrame.get(robotSide));
      tempSupportPolygon.changeFrame(soleFrame);

      computeEntryCMP(entryCMPToPack, robotSide, soleFrame, centroidInSoleFrameOfPreviousSupportFoot, previousLateCMP);
   }

   private void computeEntryCMPForFootstep(FramePoint entryCMPToPack, Footstep footstep, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot, YoFramePoint previousLateCMP)
   {
      ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
      List<Point2d> predictedContactPoints = footstep.getPredictedContactPoints();
      RobotSide robotSide = footstep.getRobotSide();

      if (predictedContactPoints != null)
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
      else
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

      computeEntryCMP(entryCMPToPack, robotSide, soleFrame, centroidInSoleFrameOfPreviousSupportFoot, previousLateCMP);
   }

   private void computeEntryCMP(FramePoint entryCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot, YoFramePoint previousLateCMP)
   {
      if (useTwoCMPsPerSupport)
      {
         if (centroidInSoleFrameOfPreviousSupportFoot != null)
            centroidOfFootstepToConsider.setIncludingFrame(centroidInSoleFrameOfPreviousSupportFoot);
         else
            centroidOfFootstepToConsider.setToZero(soleFrame);
         centroidOfFootstepToConsider.changeFrameAndProjectToXYPlane(soleFrame);

         if (previousLateCMP != null)
         {
            previousLateCMP.getFrameTuple2dIncludingFrame(previousExitCMP2d);
            previousExitCMP2d.changeFrameAndProjectToXYPlane(soleFrame);
            // Choose the laziest option
            if (Math.abs(previousExitCMP2d.getX()) < Math.abs(centroidOfFootstepToConsider.getX()))
               centroidOfFootstepToConsider.set(previousExitCMP2d);
         }

         constrainCMPAccordingToSupportPolygonAndUserOffsets(cmp2d, centroidOfFootstepToConsider, entryCMPUserOffsets.get(robotSide), minForwardEntryCMPOffset.getDoubleValue(), maxForwardEntryCMPOffset.getDoubleValue());
      }
      else
      {
         cmp2d.setIncludingFrame(tempSupportPolygon.getCentroid());
         YoFrameVector2d offset = entryCMPUserOffsets.get(robotSide);
         cmp2d.add(offset.getX(), offset.getY());
      }

      entryCMPToPack.setXYIncludingFrame(cmp2d);
      entryCMPToPack.changeFrame(worldFrame);
   }

   private void computeExitCMPForSupportFoot(FramePoint exitCMPToPack, RobotSide robotSide, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot, boolean isUpcomingFootstepLast)
   {
      if (useTwoCMPsPerSupport)
      {
         ReferenceFrame soleFrame = soleFrames.get(robotSide);
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleFrame.get(robotSide));
         tempSupportPolygon.changeFrame(soleFrame);
         
         computeExitCMP(exitCMPToPack, robotSide, soleFrame, centroidInSoleFrameOfUpcomingSupportFoot, isUpcomingFootstepLast);
      }
      else
      {
         exitCMPToPack.setToNaN(worldFrame);
      }
   }

   private void computeExitCMPForFootstep(FramePoint exitCMPToPack, Footstep footstep, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot, boolean isUpcomingFootstepLast)
   {
      if (useTwoCMPsPerSupport)
      {
         ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
         List<Point2d> predictedContactPoints = footstep.getPredictedContactPoints();
         RobotSide robotSide = footstep.getRobotSide();

         if (predictedContactPoints != null)
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
         else
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

         computeExitCMP(exitCMPToPack, robotSide, soleFrame, centroidInSoleFrameOfUpcomingSupportFoot, isUpcomingFootstepLast);
      }
      else
      {
         exitCMPToPack.setToNaN(worldFrame);
      }
   }

   private void computeExitCMP(FramePoint exitCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot, boolean isUpcomingFootstepLast)
   {
      if (centroidInSoleFrameOfUpcomingSupportFoot != null)
         centroidOfFootstepToConsider.setIncludingFrame(centroidInSoleFrameOfUpcomingSupportFoot);
      else
         centroidOfFootstepToConsider.setToZero(soleFrame);
      centroidOfFootstepToConsider.changeFrameAndProjectToXYPlane(soleFrame);

      boolean putCMPOnToes = false;

      if (useExitCMPOnToesForSteppingDown && !isUpcomingFootstepLast)
      {
         if (centroidInSoleFrameOfUpcomingSupportFoot != null)
         {
            soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
            soleFrameOrigin.changeFrame(soleFrame);
            soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);
            boolean isSteppingForwardEnough = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCMPOnToes.getDoubleValue();
            soleToSoleFrameVector.changeFrame(worldFrame);
            boolean isSteppingDownEnough = soleToSoleFrameVector.getZ() < - footstepHeightThresholdToPutExitCMPOnToes.getDoubleValue();

            putCMPOnToes = isSteppingForwardEnough && isSteppingDownEnough;
         }
      }

      if (putCMPOnToes)
         putExitCMPOnToes(cmp2d);
      else
         constrainCMPAccordingToSupportPolygonAndUserOffsets(cmp2d, centroidOfFootstepToConsider, exitCMPUserOffsets.get(robotSide), minForwardExitCMPOffset.getDoubleValue(), maxForwardExitCMPOffset.getDoubleValue());

      exitCMPToPack.setXYIncludingFrame(cmp2d);
      exitCMPToPack.changeFrame(worldFrame);
   }

   private void putExitCMPOnToes(FramePoint2d exitCMPToPack)
   {
      // Set x to have the CMP slightly inside the support polygon
      exitCMPToPack.setToZero(tempSupportPolygon.getReferenceFrame());
      exitCMPToPack.setX(tempSupportPolygon.getMaxX() - 1.6e-2);
      exitCMPToPack.setY(tempSupportPolygon.getCentroid().getY());

      // Then constrain the computed CMP to be inside a safe support region
      tempSupportPolygonForShrinking.setIncludingFrameAndUpdate(tempSupportPolygon);
      convexPolygonShrinker.shrinkConstantDistanceInto(tempSupportPolygonForShrinking, safeDistanceFromCMPToSupportEdgesWhenSteppingDown.getDoubleValue(), tempSupportPolygon);
      
      tempSupportPolygon.orthogonalProjection(exitCMPToPack);
   }

   private void constrainCMPAccordingToSupportPolygonAndUserOffsets(FramePoint2d cmpToPack, FramePoint2d centroidOfFootstepToConsider, YoFrameVector2d cmpOffset, double minForwardCMPOffset, double maxForwardCMPOffset)
   {
      // First constrain the computed CMP to the given min/max along the x-axis.
      double cmpXOffsetFromCentroid = stepLengthToCMPOffsetFactor.getDoubleValue() * centroidOfFootstepToConsider.getX() + cmpOffset.getX();
      cmpXOffsetFromCentroid = MathTools.clipToMinMax(cmpXOffsetFromCentroid, minForwardCMPOffset, maxForwardCMPOffset);

      cmpToPack.setIncludingFrame(tempSupportPolygon.getCentroid());
      cmpToPack.add(cmpXOffsetFromCentroid, cmpOffset.getY());
      
      // Then constrain the computed CMP to be inside a safe support region
      tempSupportPolygonForShrinking.setIncludingFrameAndUpdate(tempSupportPolygon);
      convexPolygonShrinker.shrinkConstantDistanceInto(tempSupportPolygonForShrinking, safeDistanceFromCMPToSupportEdges.getDoubleValue(), tempSupportPolygon);

      tempSupportPolygon.orthogonalProjection(cmpToPack);
   }

   public ArrayList<YoFramePoint> getEntryCMPs()
   {
      return entryCMPsInWorldFrameReadOnly;
   }

   public ArrayList<YoFramePoint> getExitCMPs()
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
}
