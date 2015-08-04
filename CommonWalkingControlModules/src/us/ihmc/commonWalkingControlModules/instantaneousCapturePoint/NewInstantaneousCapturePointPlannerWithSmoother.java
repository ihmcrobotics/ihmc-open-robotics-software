package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

   private final BooleanYoVariable atAStop = new BooleanYoVariable(namePrefix + "AtAStop", registry);
   private final BooleanYoVariable cancelPlan = new BooleanYoVariable(namePrefix + "CancelPlan", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable(namePrefix + "ComeToStop", registry);
   protected final BooleanYoVariable isDoubleSupport = new BooleanYoVariable(namePrefix + "IsDoubleSupport", registry);
   private final BooleanYoVariable hasBeenWokenUp = new BooleanYoVariable(namePrefix + "HasBeenWokenUp", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(namePrefix + "TimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable(namePrefix + "isDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(namePrefix + "DoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(namePrefix + "SingleSupportTime", registry);
   private final DoubleYoVariable minSingleSupportDuration = new DoubleYoVariable(namePrefix + "MinSingleSupportTime", registry);
   private final DoubleYoVariable timeCorrectionFactor = new DoubleYoVariable(namePrefix + "TimeCorrectionFactor", registry);
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
      hasBeenWokenUp.set(false);

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

      minSingleSupportDuration.set(0.3); // TODO Need to be extracted
      timeCorrectionFactor.set(0.5); // TODO Need to be extracted

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

   public void updatePlanForSingleSupportDisturbances(double time, FrameTupleArrayList<FramePoint> footstepList, FramePoint actualCapturePointPosition)
   {
      initializeSingleSupport(this.initialTime.getDoubleValue(), footstepList);

      YoFramePoint constantCMP = constantCentroidalMomentumPivots.get(0);
      double actualDistanceDueToDisturbance = constantCMP.distance(actualCapturePointPosition);
      double expectedDistanceAccordingToPlan = constantCMP.distance(singleSupportInitialDesiredCapturePointPosition);

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
      CapturePointTools.computeConstantCMPs(constantCentroidalMomentumPivots, footstepList, 0, indexOfLastFootstepToConsider, atAStop.getBooleanValue(), comeToStop.getBooleanValue());
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
      CapturePointTools.computeDesiredCornerPoints(capturePointCornerPoints, constantCentroidalMomentumPivots, false, steppingDuration, omega0.getDoubleValue());
   }

   protected void computeDesiredCapturePointPosition(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         singleSupportInitialDesiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
         constantCentroidalMomentumPivots.get(0).getFrameTupleIncludingFrame(tmpFramePoint2);
         
         time = MathTools.clipToMinMax(time, 0.0, singleSupportDuration.getDoubleValue());
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

         time = MathTools.clipToMinMax(time, 0.0, singleSupportDuration.getDoubleValue());
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

         time = MathTools.clipToMinMax(time, 0.0, singleSupportDuration.getDoubleValue());
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

      tmpArrayListOfFramePoints.clear();
      tmpArrayListOfFramePoints.setOrCreate(0, footstepList.get(0));
      tmpArrayListOfFramePoints.setOrCreate(1, footstepList.get(1));

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
