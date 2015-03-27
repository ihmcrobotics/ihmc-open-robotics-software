package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.lists.FrameTupleArrayList;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;

public class NewInstantaneousCapturePointPlannerWithSmoother
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private boolean VISUALIZE = true;
   private double ICP_CORNER_POINT_SIZE = 0.004;
   private double ICP_CONSTANT_COP_POINT_SIZE = 0.005;
   
   private final String namePrefix = "icpPlanner";

   private final FramePoint tmpFramePoint1 = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint2 = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint3 = new FramePoint(worldFrame);
   private final FrameVector tmpFrameVector1 = new FrameVector(worldFrame);
   private final FrameVector tmpFrameVector2 = new FrameVector(worldFrame);

   private final FramePoint2d tmp2dFramePoint1 = new FramePoint2d(worldFrame);
   private final FramePoint2d tmp2dFramePoint2 = new FramePoint2d(worldFrame);
   private final FramePoint2d tmp2dFramePoint3 = new FramePoint2d(worldFrame);
   private final FrameLine2d frameLine2d;

   private final BooleanYoVariable atAStop = new BooleanYoVariable(namePrefix + "AtAStop", registry);
   private final BooleanYoVariable cancelPlan = new BooleanYoVariable(namePrefix + "CancelPlan", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable(namePrefix + "ComeToStop", registry);
   private final BooleanYoVariable wasPushedInSingleSupport = new BooleanYoVariable(namePrefix + "WasPushedInSingleSupport", registry);
   private final BooleanYoVariable wasPushedInDoubleSupport = new BooleanYoVariable(namePrefix + "WasPushedInDoubleSupport", registry);
   protected final BooleanYoVariable isDoubleSupport = new BooleanYoVariable(namePrefix + "IsDoubleSupport", registry);
   private final BooleanYoVariable hasBeenWokenUp = new BooleanYoVariable(namePrefix + "HasBeenWokenUp", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(namePrefix + "TimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable(namePrefix + "isDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(namePrefix + "DoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(namePrefix + "SingleSupportTime", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable(namePrefix + "InitialTransferDuration", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable(namePrefix + "DoubleSupportSplitFractior", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable(namePrefix + "RemainingTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable(namePrefix + "NumberFootstepsToConsider", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable(namePrefix + "NumberFootstepsToStop", registry);
   private final YoFramePoint singleSupportInitialDesiredCapturePointPosition = new YoFramePoint(namePrefix + "FinalDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector singleSupportInitialDesiredCapturePointVelocity = new YoFrameVector(namePrefix + "FinalDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFramePoint nextSingleSupportInitialCapturePointPosition = new YoFramePoint(namePrefix + "NextSingleSupportInitialCapturePointPosition", worldFrame, registry);
   private final YoFramePoint desiredCentroidalMomentumPivotPosition = new YoFramePoint(namePrefix + "DesiredCentroidalMomentumPosition", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint(namePrefix + "DesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector(namePrefix + "DesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector(namePrefix + "DesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable(namePrefix + "Omega0", registry);
   private final ArrayList<YoFramePoint> constantCentroidalMomentumPivots = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> capturePointCornerPoints = new ArrayList<YoFramePoint>();
   private final FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList();
   private final FrameTupleArrayList<FramePoint> tmpArrayListOfFramePoints = FrameTupleArrayList.createFramePointArrayList();

   private final VelocityConstrainedPositionTrajectoryGenerator doubleSupportCapturePointTrajectory;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   public NewInstantaneousCapturePointPlannerWithSmoother(CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE = false;
      }

      cancelPlan.set(false);
      this.capturePointPlannerParameters = capturePointPlannerParameters;
      atAStop.set(true);
      wasPushedInSingleSupport.set(false);
      hasBeenWokenUp.set(false);

      tmp2dFramePoint1.set(0, 0);
      tmp2dFramePoint2.set(1, 1);
      frameLine2d = new FrameLine2d(tmp2dFramePoint1, tmp2dFramePoint2);

      doubleSupportCapturePointTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix + "DoubleSupportTrajectory",
            this.capturePointPlannerParameters.getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory(), worldFrame, registry);
      singleSupportDuration.set(this.capturePointPlannerParameters.getSingleSupportDuration());
      doubleSupportDuration.set(this.capturePointPlannerParameters.getDoubleSupportDuration());
      doubleSupportInitialTransferDuration.set(this.capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      numberFootstepsToConsider.set(this.capturePointPlannerParameters.getNumberOfFootstepsToConsider());
      footstepsToStop.set(this.capturePointPlannerParameters.getNumberOfFootstepsToStop());
      isDoneTimeThreshold.set(this.capturePointPlannerParameters.getIsDoneTimeThreshold());
      doubleSupportSplitFraction.set(this.capturePointPlannerParameters.getDoubleSupportSplitFraction());
      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         YoFramePoint constantCopYoFramePoint = new YoFramePoint("icpConstantCoP" + i, worldFrame, registry);
         constantCentroidalMomentumPivots.add(constantCopYoFramePoint);
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePoint icpCornerPointYoFramePoint = new YoFramePoint("icpCornerPoints" + i, worldFrame, registry);
         capturePointCornerPoints.add(icpCornerPointYoFramePoint);
      }

      parentRegistry.addChild(registry);

      if (VISUALIZE)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("ICPComputer");
      ArtifactList artifactList = new ArtifactList("ICPPlanner");

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition("icpConstantCoP" + i, constantCentroidalMomentumPivots.get(i), ICP_CONSTANT_COP_POINT_SIZE, YoAppearance.Green(), GraphicType.SOLID_BALL);
         yoGraphicsList.add(constantFootCenterCentersOfPressureViz);
         artifactList.add(constantFootCenterCentersOfPressureViz.createArtifact());
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoGraphicPosition icpCornerPointsViz = new YoGraphicPosition("icpCornerPoints" + i, capturePointCornerPoints.get(i), ICP_CORNER_POINT_SIZE, YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpCornerPointsViz);
         artifactList.add(icpCornerPointsViz.createArtifact());
      }

      YoGraphicPosition singleSupportInitialICP = new YoGraphicPosition("singleSupportInitialICP", singleSupportInitialDesiredCapturePointPosition, 0.004, YoAppearance.Chocolate(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition nextSingleSupportInitialICP = new YoGraphicPosition("nextSingleSupportInitialICP", nextSingleSupportInitialCapturePointPosition, 0.004, YoAppearance.Chocolate(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(singleSupportInitialICP);
      yoGraphicsList.add(nextSingleSupportInitialICP);
      artifactList.add(singleSupportInitialICP.createArtifact());
      artifactList.add(nextSingleSupportInitialICP.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void initializeDoubleSupport(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity, double initialTime,
         FrameTupleArrayList<FramePoint> footstepList)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      isDoubleSupport.set(true);

      if (cancelPlan.getBooleanValue())
      {
         cancelPlan(initialTime, footstepList);
         return;
      }

      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      this.initialTime.set(initialTime);

      desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);

      computeConstantCMPs(this.footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      if (comeToStop.getBooleanValue())
      {
         singleSupportInitialDesiredCapturePointPosition.set(constantCentroidalMomentumPivots.get(0));
         singleSupportInitialDesiredCapturePointVelocity.set(0.0, 0.0, 0.0);
      }
      else
      {
         computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(1), constantCentroidalMomentumPivots.get(1),
               singleSupportInitialDesiredCapturePointPosition, doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());

         computeInitialVelocityOfUpcomingSingleSupportPhase();
      }

      DoubleYoVariable doubleSupportTimeToUse = atAStop.getBooleanValue() ? doubleSupportInitialTransferDuration : doubleSupportDuration;
      initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity, singleSupportInitialDesiredCapturePointPosition,
            singleSupportInitialDesiredCapturePointVelocity, doubleSupportTimeToUse);
   }

   public void initializeSingleSupport(double initialTime, FrameTupleArrayList<FramePoint> footstepList)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      atAStop.set(false);
      isDoubleSupport.set(false);
      this.initialTime.set(initialTime);
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());

      computeConstantCMPs(this.footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(0), constantCentroidalMomentumPivots.get(0),
            singleSupportInitialDesiredCapturePointPosition, doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());

      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(1), constantCentroidalMomentumPivots.get(1),
            nextSingleSupportInitialCapturePointPosition, doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());
   }

   private void computeUpcomingSingleSupportInitialDesiredCapturePointPosition(YoFramePoint initialCapturePointPosition,
         YoFramePoint initialCenterOfPressurePosition, YoFramePoint upcomingCapturePointPositionToPack, double time)
   {
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), time, initialCapturePointPosition, initialCenterOfPressurePosition,
            upcomingCapturePointPositionToPack);
   }

   private void computeInitialVelocityOfUpcomingSingleSupportPhase()
   {
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), 0.0, singleSupportInitialDesiredCapturePointPosition,
            constantCentroidalMomentumPivots.get(1), singleSupportInitialDesiredCapturePointVelocity);
   }

   protected void computeConstantCMPs(FrameTupleArrayList<FramePoint> footstepList)
   {
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());

      int indexOfLastFootstepToConsider = Math.min(footstepList.size(), numberFootstepsToConsider.getIntegerValue()) - 1;

      if (wasPushedInSingleSupport.getBooleanValue())
      {
         // Replan but skip the first footstep.
         CapturePointTools.computeConstantCMPsOnFeet(constantCentroidalMomentumPivots, footstepList, 1, indexOfLastFootstepToConsider);
      }
      else
      {
         CapturePointTools.computeConstantCMPs(constantCentroidalMomentumPivots, footstepList, 0, indexOfLastFootstepToConsider, atAStop.getBooleanValue(), comeToStop.getBooleanValue());
      }
   }

   protected void initializeDoubleSupportCapturePointTrajectory(YoFramePoint initialCapturePointPosition, YoFrameVector initialCapturePointVelocity,
         YoFramePoint finalDesiredCapturePointPosition, YoFrameVector finalDesiredCapturePointVelocity, DoubleYoVariable doubleSupportDuration)
   {
      initialCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      finalDesiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint2);
      initialCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);
      finalDesiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector2);

      doubleSupportCapturePointTrajectory.setTrajectoryParameters(doubleSupportDuration.getDoubleValue(), tmpFramePoint1, tmpFrameVector1, tmpFramePoint2,
            tmpFrameVector2);
      doubleSupportCapturePointTrajectory.initialize();
   }

   protected void computeCapturePointCornerPoints(double steppingDuration)
   {
      boolean skipFirstCornerPoint = wasPushedInSingleSupport.getBooleanValue() || wasPushedInDoubleSupport.getBooleanValue();
      CapturePointTools.computeDesiredCornerPoints(capturePointCornerPoints, constantCentroidalMomentumPivots, skipFirstCornerPoint, steppingDuration, omega0.getDoubleValue());
   }

   protected void computeDesiredCapturePointPosition(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         singleSupportInitialDesiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
         constantCentroidalMomentumPivots.get(0).getFrameTupleIncludingFrame(tmpFramePoint2);

         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), time, tmpFramePoint1, tmpFramePoint2, tmpFramePoint3);
         desiredCapturePointPosition.set(tmpFramePoint3);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.get(tmpFramePoint1);
         desiredCapturePointPosition.set(tmpFramePoint1);
      }
   }

   protected void computeDesiredCapturePointVelocity(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         singleSupportInitialDesiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
         constantCentroidalMomentumPivots.get(0).getFrameTupleIncludingFrame(tmpFramePoint2);

         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), time, tmpFramePoint1, tmpFramePoint2, tmpFrameVector1);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.packVelocity(tmpFrameVector1);
      }
      desiredCapturePointVelocity.set(tmpFrameVector1);
   }

   protected void computeDesiredCapturePointAcceleration(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         singleSupportInitialDesiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
         constantCentroidalMomentumPivots.get(0).getFrameTupleIncludingFrame(tmpFramePoint2);

         CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), time, tmpFramePoint1, tmpFramePoint2, tmpFrameVector1);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.packAcceleration(tmpFrameVector1);
      }
      desiredCapturePointAcceleration.set(tmpFrameVector1);
   }

   protected void computeDesiredCentroidalMomentumPivot()
   {
      if (isDoubleSupport.getBooleanValue())
      {
         desiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
         desiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);

         CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpFramePoint1, tmpFrameVector1, omega0.getDoubleValue(),
               desiredCentroidalMomentumPivotPosition);
      }
      else
      {
         desiredCentroidalMomentumPivotPosition.set(constantCentroidalMomentumPivots.get(0));
      }
   }

   public void packDesiredCapturePointPositionVelocityAndAcceleration(FramePoint desiredCapturePointPositionToPack,
         FrameVector desiredCapturePointVelocityToPack, FrameVector desiredCapturePointAccelerationToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePointPosition(timeInCurrentState.getDoubleValue());
      computeDesiredCapturePointVelocity(timeInCurrentState.getDoubleValue());
      computeDesiredCapturePointAcceleration(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);
      desiredCapturePointAcceleration.getFrameTupleIncludingFrame(tmpFrameVector2);

      desiredCapturePointPositionToPack.set(tmpFramePoint1);
      desiredCapturePointVelocityToPack.set(tmpFrameVector1);
      desiredCapturePointAccelerationToPack.set(tmpFrameVector2);
   }

   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack,
         double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePointPosition(timeInCurrentState.getDoubleValue());
      computeDesiredCapturePointVelocity(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);

      desiredCapturePointPositionToPack.set(tmpFramePoint1);
      desiredCapturePointVelocityToPack.set(tmpFrameVector1);
   }

   protected void packDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack, YoFrameVector desiredCapturePointVelocityToPack,
         double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePointPosition(timeInCurrentState.getDoubleValue());
      computeDesiredCapturePointVelocity(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);

      desiredCapturePointPositionToPack.set(desiredCapturePointPosition);
      desiredCapturePointVelocityToPack.set(desiredCapturePointVelocity);
   }

   public void packDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      computeDesiredCentroidalMomentumPivot();

      desiredCentroidalMomentumPivotPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   public void updatePlanForDoubleSupportPush(ArrayList<FramePoint> footstepList, FramePoint actualCapturePointPosition, double time)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      initialTime.set(time);
      wasPushedInDoubleSupport.set(true);
      isDoubleSupport.set(false);
      atAStop.set(false);
      comeToStop.set(true);

      computeConstantCMPs(this.footstepList);
      double stepDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      computeCapturePointCornerPoints(stepDuration);

      capturePointCornerPoints.get(1).getFrameTuple2dIncludingFrame(tmp2dFramePoint1);
      tmp2dFramePoint2.setByProjectionOntoXYPlaneIncludingFrame(footstepList.get(0));

      frameLine2d.set(tmp2dFramePoint1, tmp2dFramePoint2);

      CapturePointTools.computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(actualCapturePointPosition, frameLine2d, tmp2dFramePoint3);

      capturePointCornerPoints.get(0).setXY(tmp2dFramePoint3);

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0), capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), omega0.getDoubleValue(), stepDuration);

      desiredCapturePointPosition.set(capturePointCornerPoints.get(0));
      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);

      wasPushedInDoubleSupport.set(false);
   }

   public void updatePlanForDoubleSupportPush(FrameTupleArrayList<FramePoint> footstepList, FramePoint actualCapturePointPosition, double time)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      initialTime.set(time);
      wasPushedInDoubleSupport.set(true);
      isDoubleSupport.set(false);
      atAStop.set(false);
      comeToStop.set(true);

      computeConstantCMPs(this.footstepList);
      double stepDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      computeCapturePointCornerPoints(stepDuration);

      capturePointCornerPoints.get(1).getFrameTuple2dIncludingFrame(tmp2dFramePoint1);
      tmp2dFramePoint2.setByProjectionOntoXYPlaneIncludingFrame(footstepList.get(0));

      frameLine2d.set(tmp2dFramePoint1, tmp2dFramePoint2);

      CapturePointTools.computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(actualCapturePointPosition, frameLine2d, tmp2dFramePoint3);

      capturePointCornerPoints.get(0).setXY(tmp2dFramePoint3);

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0), capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), omega0.getDoubleValue(), stepDuration);

      desiredCapturePointPosition.set(capturePointCornerPoints.get(0));
      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);

      wasPushedInDoubleSupport.set(false);
   }

   public void updatePlanForSingleSupportPush(ArrayList<FramePoint> footstepList, FramePoint actualCapturePointPosition, double time)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      computeTimeInCurrentState(time);
      double timeRemaining = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
      wasPushedInSingleSupport.set(true);

      computeConstantCMPs(this.footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      tmp2dFramePoint1.setIncludingFrame(capturePointCornerPoints.get(1).getReferenceFrame(), capturePointCornerPoints.get(1).getX(), capturePointCornerPoints
            .get(1).getY());

      tmp2dFramePoint2.setIncludingFrame(footstepList.get(0).getReferenceFrame(), footstepList.get(0).getX(), footstepList.get(0).getY());

      frameLine2d.set(tmp2dFramePoint1, tmp2dFramePoint2);

      CapturePointTools.computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(actualCapturePointPosition, frameLine2d, tmp2dFramePoint3);

      capturePointCornerPoints.get(0).set(tmp2dFramePoint3.getX(), tmp2dFramePoint3.getY(), 0);
      desiredCapturePointPosition.set(tmp2dFramePoint3.getX(), tmp2dFramePoint3.getY(), 0.0);

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0), capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), omega0.getDoubleValue(), timeRemaining + doubleSupportDuration.getDoubleValue());

      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);

      initialTime.set(time);

      wasPushedInSingleSupport.set(false);
   }

   public void updatePlanForSingleSupportPush(FrameTupleArrayList<FramePoint> footstepList, FramePoint actualCapturePointPosition, double time)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      computeTimeInCurrentState(time);
      double timeRemaining = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
      wasPushedInSingleSupport.set(true);

      computeConstantCMPs(this.footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      tmp2dFramePoint1.setIncludingFrame(capturePointCornerPoints.get(1).getReferenceFrame(), capturePointCornerPoints.get(1).getX(), capturePointCornerPoints
            .get(1).getY());

      tmp2dFramePoint2.setIncludingFrame(footstepList.get(0).getReferenceFrame(), footstepList.get(0).getX(), footstepList.get(0).getY());

      frameLine2d.set(tmp2dFramePoint1, tmp2dFramePoint2);

      CapturePointTools.computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(actualCapturePointPosition, frameLine2d, tmp2dFramePoint3);

      capturePointCornerPoints.get(0).set(tmp2dFramePoint3.getX(), tmp2dFramePoint3.getY(), 0);
      desiredCapturePointPosition.set(tmp2dFramePoint3.getX(), tmp2dFramePoint3.getY(), 0.0);

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0), capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), omega0.getDoubleValue(), timeRemaining + doubleSupportDuration.getDoubleValue());

      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);

      initialTime.set(time);

      wasPushedInSingleSupport.set(false);
   }

   public void cancelPlan(double time, ArrayList<FramePoint> footstepList)
   {
      if (isDoubleSupport.getBooleanValue())
      {
         cancelPlanNow(time, footstepList);
      }
      else
      {
         cancelPlan.set(true);
      }
   }

   public void cancelPlan(double time, FrameTupleArrayList<FramePoint> footstepList)
   {
      if (isDoubleSupport.getBooleanValue())
      {
         cancelPlanNow(time, footstepList);
      }
      else
      {
         cancelPlan.set(true);
      }
   }

   private void cancelPlanNow(double time, ArrayList<FramePoint> footstepList)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      capturePointCornerPoints.get(0).set(desiredCapturePointPosition);

      singleSupportInitialDesiredCapturePointPosition.set(footstepList.get(0));
      singleSupportInitialDesiredCapturePointPosition.add(footstepList.get(1));
      singleSupportInitialDesiredCapturePointPosition.scale(0.5);

      singleSupportInitialDesiredCapturePointVelocity.set(0.0, 0.0, 0.0);

      for (int i = 1; i < constantCentroidalMomentumPivots.size(); i++)
      {
         constantCentroidalMomentumPivots.get(i).set(singleSupportInitialDesiredCapturePointPosition);
      }

      for (int i = 1; i < capturePointCornerPoints.size(); i++)
      {
         capturePointCornerPoints.get(i).set(singleSupportInitialDesiredCapturePointPosition);
      }

      initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity, singleSupportInitialDesiredCapturePointPosition,
            singleSupportInitialDesiredCapturePointVelocity, doubleSupportDuration);

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0),
            singleSupportInitialDesiredCapturePointPosition, desiredCapturePointPosition, omega0.getDoubleValue(), doubleSupportDuration.getDoubleValue());

      initialTime.set(time);

      cancelPlan.set(false);
   }

   private void cancelPlanNow(double time, FrameTupleArrayList<FramePoint> footstepList)
   {
      this.footstepList.copyFromListAndTrimSize(footstepList);

      capturePointCornerPoints.get(0).set(desiredCapturePointPosition);

      singleSupportInitialDesiredCapturePointPosition.set(footstepList.get(0));
      singleSupportInitialDesiredCapturePointPosition.add(footstepList.get(1));
      singleSupportInitialDesiredCapturePointPosition.scale(0.5);

      singleSupportInitialDesiredCapturePointVelocity.set(0.0, 0.0, 0.0);

      for (int i = 1; i < constantCentroidalMomentumPivots.size(); i++)
      {
         constantCentroidalMomentumPivots.get(i).set(singleSupportInitialDesiredCapturePointPosition);
      }

      for (int i = 1; i < capturePointCornerPoints.size(); i++)
      {
         capturePointCornerPoints.get(i).set(singleSupportInitialDesiredCapturePointPosition);
      }

      initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity, singleSupportInitialDesiredCapturePointPosition,
            singleSupportInitialDesiredCapturePointVelocity, doubleSupportDuration);

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0),
            singleSupportInitialDesiredCapturePointPosition, desiredCapturePointPosition, omega0.getDoubleValue(), doubleSupportDuration.getDoubleValue());

      initialTime.set(time);

      cancelPlan.set(false);
   }

   public void resetParametersToDefault()
   {
      singleSupportDuration.set(capturePointPlannerParameters.getSingleSupportDuration());
      doubleSupportDuration.set(capturePointPlannerParameters.getDoubleSupportDuration());
      doubleSupportInitialTransferDuration.set(capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      numberFootstepsToConsider.set(capturePointPlannerParameters.getNumberOfFootstepsToConsider());
      doubleSupportSplitFraction.set(capturePointPlannerParameters.getDoubleSupportSplitFraction());
   }

   protected void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   public double computeAndReturnTimeRemaining(double time)
   {
      computeTimeInCurrentState(time);

      if (isDoubleSupport.getBooleanValue())
      {
         if (atAStop.getBooleanValue())
         {
            remainingTime.set(doubleSupportInitialTransferDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
         }
         else
         {
            remainingTime.set(doubleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
         }
      }
      else
      {
         remainingTime.set(singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
      }

      return remainingTime.getDoubleValue();
   }

   public ArrayList<YoFramePoint> getCapturePointCornerPoints()
   {
      return capturePointCornerPoints;
   }

   public ArrayList<YoFramePoint> getConstantCentroidalMomentumPivots()
   {
      return constantCentroidalMomentumPivots;
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

   public void getSingleSupportInitialCapturePointPosition(FramePoint capturePointPositionToPack)
   {
      singleSupportInitialDesiredCapturePointPosition.getFrameTupleIncludingFrame(capturePointPositionToPack);
   }

   public double computeAndReturnTimeInCurrentState(double time)
   {
      computeTimeInCurrentState(time);
      return timeInCurrentState.getDoubleValue();
   }

   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      capturePointCornerPoints.get(1).getFrameTupleIncludingFrame(finalDesiredCapturePointPositionToPack);
   }

   public void getConstantCentroidalMomentumPivotPosition(FramePoint constantCentroidalMomentumPositionToPack)
   {
      constantCentroidalMomentumPivots.get(0).getFrameTupleIncludingFrame(constantCentroidalMomentumPositionToPack);
   }

   public void reset(double time)
   {
      comeToStop.set(true);
      atAStop.set(true);

      desiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);

      tmpArrayListOfFramePoints.copyFromListAndTrimSize(footstepList);

      initializeDoubleSupport(tmpFramePoint1, tmpFrameVector1, time, tmpArrayListOfFramePoints);
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
