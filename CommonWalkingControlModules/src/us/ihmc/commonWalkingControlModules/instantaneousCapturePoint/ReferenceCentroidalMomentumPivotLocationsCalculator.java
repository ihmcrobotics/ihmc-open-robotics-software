package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
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
   private static final boolean PUT_EXIT_CMP_ON_TOES_FOR_STEPS_DOWN = false;
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
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygons = new SideDependentList<>();

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

   private final DoubleYoVariable safeMarginDistanceForFootPolygon = new DoubleYoVariable("safeMarginDistanceForFootPolygon", registry);

   private boolean useTwoCMPsPerSupport = false;

   public ReferenceCentroidalMomentumPivotLocationsCalculator(String namePrefix, BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         int numberFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      firstEntryCMPForSingleSupport.setToNaN();

      isDoneWalking = new BooleanYoVariable(namePrefix + "IsDoneWalking", registry);
      maxForwardEntryCMPOffset = new DoubleYoVariable(namePrefix + "MaxForwardEntryCMPOffset", registry);
      minForwardEntryCMPOffset = new DoubleYoVariable(namePrefix + "MinForwardEntryCMPOffset", registry);
      maxForwardExitCMPOffset = new DoubleYoVariable(namePrefix + "MaxForwardExitCMPOffset", registry);
      minForwardExitCMPOffset = new DoubleYoVariable(namePrefix + "MinForwardExitCMPOffset", registry);
      footstepHeightThresholdToPutExitCMPOnToes = new DoubleYoVariable(namePrefix + "FootstepHeightThresholdToPutExitCMPOnToes", registry);
      footstepHeightThresholdToPutExitCMPOnToes.set(0.15); // TODO Magic number. Need to be tuned on the real robot and to be extracted.
      footstepLengthThresholdToPutExitCMPOnToes = new DoubleYoVariable(namePrefix + "FootstepLengthThresholdToPutExitCMPOnToes", registry);
      footstepLengthThresholdToPutExitCMPOnToes.set(0.15); // TODO Magic number. Need to be tuned on the real robot and to be extracted.

      stepLengthToCMPOffsetFactor = new DoubleYoVariable(namePrefix + "StepLengthToCMPOffsetFactor", registry);
      stepLengthToCMPOffsetFactor.set(1.0 / 3.0); // TODO Magic number. Need to be tuned on the real robot and to be extracted.

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
         supportFootPolygons.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide));
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygons.get(robotSide)); // Just to allocate memory
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

   public void setUseTwoCMPsPerSupport(boolean useTwoCMPsPerSupport)
   {
      this.useTwoCMPsPerSupport = useTwoCMPsPerSupport;
   }

   public void setSafeDistanceFromSupportEdges(double distance)
   {
      safeMarginDistanceForFootPolygon.set(distance);
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
         computeEntryCMPForSupportFoot(firstCMP, RobotSide.LEFT, null, null);
         computeEntryCMPForSupportFoot(secondCMP, RobotSide.RIGHT, null, null);
         firstCMP.changeFrame(midFeetZUpFrame);
         secondCMP.changeFrame(midFeetZUpFrame);
         entryCMPs.get(cmpIndex).switchCurrentReferenceFrame(midFeetZUpFrame);
         exitCMPs.get(cmpIndex).switchCurrentReferenceFrame(midFeetZUpFrame);
         entryCMPs.get(cmpIndex).interpolate(firstCMP, secondCMP, 0.5);
         exitCMPs.get(cmpIndex).interpolate(firstCMP, secondCMP, 0.5);
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
         computeExitCMPForSupportFoot(cmp, transferFromSide, transferToSoleFrame);
         cmp.changeFrame(transferFromSoleFrame);
         exitCMPs.get(cmpIndex).setIncludingFrame(cmp);
         cmpIndex++;
      }

      computeEntryCMPForSupportFoot(cmp, transferToSide, transferFromSoleFrame, exitCMPs.get(cmpIndex - 1));
      cmp.changeFrame(transferToSoleFrame);
      entryCMPs.get(cmpIndex).setIncludingFrame(cmp);
      firstEntryCMPForSingleSupport.setByProjectionOntoXYPlaneIncludingFrame(cmp);
      computeExitCMPForSupportFoot(cmp, transferToSide, upcomingFootsteps.get(0).getSoleReferenceFrame());
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
      computeExitCMPForSupportFoot(cmp, supportSide, upcomingFootsteps.get(0).getSoleReferenceFrame());
      cmp.changeFrame(supportSoleFrame);
      exitCMPs.get(constantCMPIndex).setIncludingFrame(cmp);
      constantCMPIndex++;

      if (onlyOneUpcomingFootstep)
      {
         computeEntryCMPForSupportFoot(firstCMP, supportSide, null, null);
         computeEntryCMPForFootstep(secondCMP, upcomingFootsteps.get(0), null, null);
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
      ReferenceFrame soleFrameOfPreviousFootstep = soleFrames.get(firstSupportSide);

      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         Footstep currentFootstep = upcomingFootsteps.get(i);
         ReferenceFrame soleOfNextFootstep = null;
         if (i < upcomingFootsteps.size() - 1) soleOfNextFootstep = upcomingFootsteps.get(i + 1).getSoleReferenceFrame();
         
         computeExitCMPForFootstep(cmp, currentFootstep, soleOfNextFootstep);
         cmp.changeFrame(soleFrames.get(firstSupportSide));
         exitCMPs.get(cmpIndex).setIncludingFrame(cmp);

         YoFramePoint previousExitCMP = exitCMPs.get(cmpIndex - 1);
         computeEntryCMPForFootstep(cmp, currentFootstep, soleFrameOfPreviousFootstep, previousExitCMP);
         cmp.changeFrame(soleFrames.get(firstSupportSide));
         entryCMPs.get(cmpIndex).setIncludingFrame(cmp);
         
         cmpIndex++;
         soleFrameOfPreviousFootstep = currentFootstep.getSoleReferenceFrame();

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

   private void computeEntryCMPForSupportFoot(FramePoint entryCMPToPack, RobotSide robotSide, ReferenceFrame soleFrameOfPreviousSupportFoot, YoFramePoint previousLateCMP)
   {
      ReferenceFrame soleFrame = soleFrames.get(robotSide);
      tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygons.get(robotSide));
      tempSupportPolygon.changeFrame(soleFrame);

      computeEntryCMP(entryCMPToPack, robotSide, soleFrame, soleFrameOfPreviousSupportFoot, previousLateCMP);
   }

   private void computeEntryCMPForFootstep(FramePoint entryCMPToPack, Footstep footstep, ReferenceFrame soleFrameOfPreviousSupportFoot, YoFramePoint previousLateCMP)
   {
      ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
      List<Point2d> predictedContactPoints = footstep.getPredictedContactPoints();
      RobotSide robotSide = footstep.getRobotSide();

      if (predictedContactPoints != null)
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
      else
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

      computeEntryCMP(entryCMPToPack, robotSide, soleFrame, soleFrameOfPreviousSupportFoot, previousLateCMP);
   }

   private void computeEntryCMP(FramePoint entryCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, ReferenceFrame soleFrameOfPreviousSupportFoot, YoFramePoint previousLateCMP)
   {
      if (useTwoCMPsPerSupport)
      {
         if (soleFrameOfPreviousSupportFoot != null)
            cmp2d.setIncludingFrame(soleFrameOfPreviousSupportFoot, 0.0, 0.0);
         else
            cmp2d.setToZero(soleFrame);
         cmp2d.changeFrameAndProjectToXYPlane(soleFrame);

         if (previousLateCMP != null)
         {
            previousLateCMP.getFrameTuple2dIncludingFrame(previousExitCMP2d);
            previousExitCMP2d.changeFrameAndProjectToXYPlane(soleFrame);
            // Choose the laziest option
            if (Math.abs(previousExitCMP2d.getX()) < Math.abs(cmp2d.getX()))
               cmp2d.set(previousExitCMP2d);
         }

         constrainCMPAccordingToSupportPolygonAndUserOffsets(robotSide, entryCMPUserOffsets.get(robotSide), minForwardEntryCMPOffset.getDoubleValue(), maxForwardEntryCMPOffset.getDoubleValue());
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

   private void computeExitCMPForSupportFoot(FramePoint exitCMPToPack, RobotSide robotSide, ReferenceFrame soleFrameOfUpcomingSupportFoot)
   {
      if (useTwoCMPsPerSupport)
      {
         ReferenceFrame soleFrame = soleFrames.get(robotSide);
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygons.get(robotSide));
         tempSupportPolygon.changeFrame(soleFrame);
         
         computeExitCMP(exitCMPToPack, robotSide, soleFrame, soleFrameOfUpcomingSupportFoot);
      }
      else
      {
         exitCMPToPack.setToNaN(worldFrame);
      }
   }

   private void computeExitCMPForFootstep(FramePoint exitCMPToPack, Footstep footstep, ReferenceFrame soleFrameOfUpcomingSupportFoot)
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

         computeExitCMP(exitCMPToPack, robotSide, soleFrame, soleFrameOfUpcomingSupportFoot);
      }
      else
      {
         exitCMPToPack.setToNaN(worldFrame);
      }
   }

   private void computeExitCMP(FramePoint exitCMPToPack, RobotSide robotSide, ReferenceFrame soleFrame, ReferenceFrame soleFrameOfUpcomingSupportFoot)
   {
      if (soleFrameOfUpcomingSupportFoot != null)
         cmp2d.setIncludingFrame(soleFrameOfUpcomingSupportFoot, 0.0, 0.0);
      else
         cmp2d.setToZero(soleFrame);
      cmp2d.changeFrameAndProjectToXYPlane(soleFrame);

      if (PUT_EXIT_CMP_ON_TOES_FOR_STEPS_DOWN)
      {
         if (soleFrameOfUpcomingSupportFoot != null)
         {
            soleFrameOrigin.setToZero(soleFrameOfUpcomingSupportFoot);
            soleFrameOrigin.changeFrame(soleFrame);
            soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);
            boolean isSteppingForwardEnough = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCMPOnToes.getDoubleValue();
            soleToSoleFrameVector.changeFrame(worldFrame);
            boolean isSteppingDownEnough = soleToSoleFrameVector.getZ() < - footstepHeightThresholdToPutExitCMPOnToes.getDoubleValue();

            if (isSteppingForwardEnough && isSteppingDownEnough)
               putExitCMPOnToes(robotSide);
            else
               constrainCMPAccordingToSupportPolygonAndUserOffsets(robotSide, exitCMPUserOffsets.get(robotSide), minForwardExitCMPOffset.getDoubleValue(), maxForwardExitCMPOffset.getDoubleValue());
         }
         else
         {
            constrainCMPAccordingToSupportPolygonAndUserOffsets(robotSide, exitCMPUserOffsets.get(robotSide), minForwardExitCMPOffset.getDoubleValue(), maxForwardExitCMPOffset.getDoubleValue());
         }
      }
      else
      {
         constrainCMPAccordingToSupportPolygonAndUserOffsets(robotSide, exitCMPUserOffsets.get(robotSide), minForwardExitCMPOffset.getDoubleValue(), maxForwardExitCMPOffset.getDoubleValue());
      }

      exitCMPToPack.setXYIncludingFrame(cmp2d);
      exitCMPToPack.changeFrame(worldFrame);
   }

   private void putExitCMPOnToes(RobotSide robotSide)
   {
      // Set x to a high value to make sure it will be projected on the toes.
      cmp2d.setX(1.0);
      cmp2d.setY(tempSupportPolygon.getCentroid().getY());

      // Then constrain the computed CMP to be inside a safe support region
//      tempSupportPolygon.shrink(safeMarginDistanceForFootPolygon.getDoubleValue());
      tempSupportPolygon.orthogonalProjection(cmp2d);
   }

   private void constrainCMPAccordingToSupportPolygonAndUserOffsets(RobotSide robotSide, YoFrameVector2d cmpOffset, double minForwardCMPOffset, double maxForwardCMPOffset)
   {
      // First constrain the computed CMP to the given min/max along the x-axis.
      double cmpX = stepLengthToCMPOffsetFactor.getDoubleValue() * cmp2d.getX() + cmpOffset.getX();
      cmp2d.setX(MathTools.clipToMinMax(cmpX, minForwardCMPOffset, maxForwardCMPOffset));
      cmp2d.setY(tempSupportPolygon.getCentroid().getY() + cmpOffset.getY());
      
      // Then constrain the computed CMP to be inside a safe support region
      tempSupportPolygon.shrink(safeMarginDistanceForFootPolygon.getDoubleValue());
      tempSupportPolygon.orthogonalProjection(cmp2d);
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

   public boolean isDoneWalking()
   {
      return isDoneWalking.getBooleanValue();
   }
}
