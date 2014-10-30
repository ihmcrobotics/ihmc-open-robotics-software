package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.DoubleSupportPolynomialTrajectory;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Line2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicLineSegment;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLine2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLine2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class NewInstantaneousCapturePointPlannerWithSmoother
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private boolean VISUALIZE = true;
   private double ICP_CORNER_POINT_SIZE = 0.004;
   private double ICP_CONSTANT_COP_POINT_SIZE = 0.005;

   private final FramePoint tmpFramePoint1 = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint2 = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint3 = new FramePoint(worldFrame);
   private final FrameVector tmpFrameVector1 = new FrameVector(worldFrame);
   private final FrameVector tmpFrameVector2 = new FrameVector(worldFrame);

   private final FramePoint2d tmp2dFramePoint1 = new FramePoint2d(worldFrame);
   private final FramePoint2d tmp2dFramePoint2 = new FramePoint2d(worldFrame);
   private final FramePoint2d tmp2dFramePoint3 = new FramePoint2d(worldFrame);
   private final FrameLine2d frameLine2d;

   private final BooleanYoVariable atAStop = new BooleanYoVariable("icpPlannerAtAStop", registry);
   private final BooleanYoVariable cancelPlan = new BooleanYoVariable("icpPlannerCancelPlan", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable("icpPlannerComeToStop", registry);
   private final BooleanYoVariable wasPushedInSingleSupport = new BooleanYoVariable("icpPlannerWasPushedInSingleSupport", registry);
   private final BooleanYoVariable wasPushedInDoubleSupport = new BooleanYoVariable("icpPlannerWasPushedInDoubleSupport", registry);
   protected final BooleanYoVariable isDoubleSupport = new BooleanYoVariable("icpPlannerIsDoubleSupport", registry);
   private final BooleanYoVariable hasBeenWokenUp = new BooleanYoVariable("icpPlannerHasBeenWokenUp", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("icpPlannerTimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable("icpPlannerisDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("icpPlannerDoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("icpPlannerSingleSupportTime", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable("icpPlannerInitialTransferDuration", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("icpPlannerDoubleSupportSplitFractior", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("icpPlannerInitialTime", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable("icpPlannerRemainingTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable("", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable("icpPlannerNumberFootstepsToStop", registry);
   private final YoFramePoint singleSupportInitialDesiredCapturePointPosition = new YoFramePoint("icpFinalDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector singleSupportInitialDesiredCapturePointVelocity = new YoFrameVector("icpFinalDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFramePoint nextSingleSupportInitialCapturePointPosition = new YoFramePoint("icpPlannerNextSingleSupportInitialCapturePointPosition",
         worldFrame, registry);
   private final YoFramePoint desiredCentroidalMomentumPivotPosition = new YoFramePoint("icpPlannerDesiredCentroidalMomentumPosition", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint("icpPlannerDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector("icpPlannerDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector("icpPlannerDesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);
   private final ArrayList<YoFramePoint> constantCentroidalMomentumPivots = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> capturePointCornerPoints = new ArrayList<YoFramePoint>();
   private final ArrayList<FramePoint> footstepList = new ArrayList<FramePoint>();
   private final ArrayList<FramePoint> tmpArrayListOfFramePoints = new ArrayList<FramePoint>();

   private final DoubleSupportPolynomialTrajectory doubleSupportCapturePointTrajectory;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   public NewInstantaneousCapturePointPlannerWithSmoother(CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE = false;
      }

      this.cancelPlan.set(false);
      this.capturePointPlannerParameters = capturePointPlannerParameters;
      this.atAStop.set(true);
      this.wasPushedInSingleSupport.set(false);
      this.hasBeenWokenUp.set(false);
      
      tmp2dFramePoint1.set(0,0);
      tmp2dFramePoint2.set(1,1);
      frameLine2d = new FrameLine2d(tmp2dFramePoint1, tmp2dFramePoint2);

      this.doubleSupportCapturePointTrajectory = new DoubleSupportPolynomialTrajectory("icpPlannerDoubleSupportTrajectory",
            this.capturePointPlannerParameters.getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory(), worldFrame, registry);
      this.singleSupportDuration.set(this.capturePointPlannerParameters.getSingleSupportDuration());
      this.doubleSupportDuration.set(this.capturePointPlannerParameters.getDoubleSupportDuration());
      this.doubleSupportInitialTransferDuration.set(this.capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      this.numberFootstepsToConsider.set(this.capturePointPlannerParameters.getNumberOfFootstepsToConsider());
      this.footstepsToStop.set(this.capturePointPlannerParameters.getNumberOfFootstepsToStop());
      this.isDoneTimeThreshold.set(this.capturePointPlannerParameters.getIsDoneTimeThreshold());
      this.doubleSupportSplitFraction.set(this.capturePointPlannerParameters.getDoubleSupportSplitFraction());
      // Initialize omega0 to NaN to force the user to explicitly set it.
      this.omega0.set(Double.NaN);

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

      parentRegistry.addChild(this.registry);

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
         YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition("icpConstantCoP" + i, constantCentroidalMomentumPivots.get(i),
               ICP_CONSTANT_COP_POINT_SIZE, YoAppearance.Green(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(constantFootCenterCentersOfPressureViz);
         artifactList.add(constantFootCenterCentersOfPressureViz.createArtifact());
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoGraphicPosition icpCornerPointsViz = new YoGraphicPosition("icpCornerPoints" + i, capturePointCornerPoints.get(i), ICP_CORNER_POINT_SIZE,
               YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpCornerPointsViz);
         artifactList.add(icpCornerPointsViz.createArtifact());
      }

      YoGraphicPosition singleSupportInitialICP = new YoGraphicPosition("singleSupportInitialICP", singleSupportInitialDesiredCapturePointPosition, 0.004,
            YoAppearance.Chocolate(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition nextSingleSupportInitialICP = new YoGraphicPosition("nextSingleSupportInitialICP", nextSingleSupportInitialCapturePointPosition, 0.004,
            YoAppearance.Chocolate(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(singleSupportInitialICP);
      yoGraphicsList.add(nextSingleSupportInitialICP);
      artifactList.add(singleSupportInitialICP.createArtifact());
      artifactList.add(nextSingleSupportInitialICP.createArtifact());
      
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void initializeDoubleSupport(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity, double initialTime,
         ArrayList<FramePoint> footstepList)
   {
      this.footstepList.clear();
      for(int i = 0; i<footstepList.size(); i++)
      {
         this.footstepList.add(footstepList.get(i));
      }
      
      this.isDoubleSupport.set(true);

      if (cancelPlan.getBooleanValue())
      {
         cancelPlan(initialTime, footstepList);
         return;
      }

      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      this.initialTime.set(initialTime);

      this.desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      this.desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());

      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(1), constantCentroidalMomentumPivots.get(1),
            singleSupportInitialDesiredCapturePointPosition, doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());

      if (comeToStop.getBooleanValue())
      {
         singleSupportInitialDesiredCapturePointVelocity.set(0.0, 0.0, 0.0);
      }
      else
      {
         computeInitialVelocityOfUpcomingSingleSupportPhase();
      }

      if (atAStop.getBooleanValue())
      {
         initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity,
               singleSupportInitialDesiredCapturePointPosition, singleSupportInitialDesiredCapturePointVelocity, doubleSupportInitialTransferDuration);
      }
      else
      {
         initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity,
               singleSupportInitialDesiredCapturePointPosition, singleSupportInitialDesiredCapturePointVelocity, doubleSupportDuration);
      }
   }

   public void initializeSingleSupport(double initialTime, ArrayList<FramePoint> footstepList)
   {
      this.footstepList.clear();
      for(int i = 0; i<footstepList.size(); i++)
      {
         this.footstepList.add(footstepList.get(i));
      }
      
      atAStop.set(false);
      this.isDoubleSupport.set(false);
      this.initialTime.set(initialTime);
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());

      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(0), constantCentroidalMomentumPivots.get(0),
            singleSupportInitialDesiredCapturePointPosition, doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());

      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(1), constantCentroidalMomentumPivots.get(1),
            nextSingleSupportInitialCapturePointPosition, doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());
   }

   private void computeUpcomingSingleSupportInitialDesiredCapturePointPosition(YoFramePoint initialCapturePointPosition,
         YoFramePoint initialCenterOfPressurePosition, YoFramePoint upcomingCapturePointPositionToPack, double time)
   {
      CapturePointTools.computeDesiredCapturePointPosition(this.omega0.getDoubleValue(), time, initialCapturePointPosition, initialCenterOfPressurePosition,
            upcomingCapturePointPositionToPack);
   }

   private void computeInitialVelocityOfUpcomingSingleSupportPhase()
   {
      CapturePointTools.computeDesiredCapturePointVelocity(this.omega0.getDoubleValue(), 0.0, singleSupportInitialDesiredCapturePointPosition,
            constantCentroidalMomentumPivots.get(1), singleSupportInitialDesiredCapturePointVelocity);
   }

   protected void computeConstantCentersOfPressure(ArrayList<FramePoint> footstepList)
   {
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());

      int numberOfCentersOfPressureToPlan = numberFootstepsToConsider.getIntegerValue();

      if (atAStop.getBooleanValue())
      {
         if (!comeToStop.getBooleanValue())
         {
            CapturePointTools.computeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet(constantCentroidalMomentumPivots, footstepList,
                  numberOfCentersOfPressureToPlan);
         }
         else
         {
            CapturePointTools.computeConstantCentersOfPressuresWithBeginningAndEndBetweenFeetRestOnFeet(constantCentroidalMomentumPivots, footstepList,
                  numberOfCentersOfPressureToPlan);
         }
      }
      else
      {
         if (comeToStop.getBooleanValue())
         {
            CapturePointTools.computeConstantCentersOfPressuresOnFeetWithEndBetweenFeet(constantCentroidalMomentumPivots, footstepList,
                  numberOfCentersOfPressureToPlan);
         }
         else
         {
            CapturePointTools.computeConstantCentersOfPressuresOnFeet(constantCentroidalMomentumPivots, footstepList, numberOfCentersOfPressureToPlan);
         }
      }
   }

   protected void initializeDoubleSupportCapturePointTrajectory(YoFramePoint initialCapturePointPosition, YoFrameVector initialCapturePointVelocity,
         YoFramePoint finalDesiredCapturePointPosition, YoFrameVector finalDesiredCapturePointVelocity, DoubleYoVariable doubleSupportDuration)
   {
      initialCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      finalDesiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint2);
      initialCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);
      finalDesiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector2);

      doubleSupportCapturePointTrajectory.initialize(doubleSupportDuration.getDoubleValue(), tmpFramePoint1, tmpFrameVector1, tmpFramePoint2, tmpFrameVector2);
   }

   protected void computeCapturePointCornerPoints(double steppingDuration)
   {
      if (!wasPushedInSingleSupport.getBooleanValue() && !wasPushedInDoubleSupport.getBooleanValue())
      {
         CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentroidalMomentumPivots, capturePointCornerPoints, steppingDuration,
               omega0.getDoubleValue());
      }
      else
      {
         CapturePointTools.computeDesiredEndOfStepCapturePointLocationsWithFirstLeftUnset(constantCentroidalMomentumPivots, capturePointCornerPoints,
               steppingDuration, omega0.getDoubleValue());
      }
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
         doubleSupportCapturePointTrajectory.getPosition(tmpFramePoint1);
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
         doubleSupportCapturePointTrajectory.getVelocity(tmpFrameVector1);
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
         doubleSupportCapturePointTrajectory.getAcceleration(tmpFrameVector1);
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

   public void updatePlanForDoubleSupportPush(ArrayList<FramePoint> footstepList, FramePoint actualCapturePointPosition,
         double time)
   {
      this.footstepList.clear();
      for(int i = 0; i<footstepList.size(); i++)
      {
         this.footstepList.add(footstepList.get(i));
      }
      
      this.initialTime.set(time);
      this.wasPushedInDoubleSupport.set(true);
      this.isDoubleSupport.set(false);
      this.atAStop.set(false);
      this.comeToStop.set(true);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());

      tmp2dFramePoint1.setIncludingFrame(capturePointCornerPoints.get(1).getReferenceFrame(), capturePointCornerPoints.get(1).getX(), capturePointCornerPoints
            .get(1).getY());
      tmp2dFramePoint2.setIncludingFrame(footstepList.get(0).getReferenceFrame(), footstepList.get(0).getX(), footstepList.get(0).getY());

      frameLine2d.set(tmp2dFramePoint1, tmp2dFramePoint2);
      
      CapturePointTools.computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(actualCapturePointPosition, frameLine2d, tmp2dFramePoint3);

      capturePointCornerPoints.get(0).set(tmp2dFramePoint3.getX(), tmp2dFramePoint3.getY(), 0);

      CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), constantCentroidalMomentumPivots.get(0), this.omega0.getDoubleValue(), singleSupportDuration.getDoubleValue()
                  + doubleSupportDuration.getDoubleValue());
      
      desiredCapturePointPosition.set(capturePointCornerPoints.get(0));
      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);
      
      this.wasPushedInDoubleSupport.set(false);
   }

   public void updatePlanForSingleSupportPush(ArrayList<FramePoint> footstepList, FramePoint actualCapturePointPosition, double time)
   {
      this.footstepList.clear();
      for(int i = 0; i<footstepList.size(); i++)
      {
         this.footstepList.add(footstepList.get(i));
      }
      
      computeTimeInCurrentState(time);
      double timeRemaining = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
      this.wasPushedInSingleSupport.set(true);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      tmp2dFramePoint1.setIncludingFrame(capturePointCornerPoints.get(1).getReferenceFrame(), capturePointCornerPoints.get(1).getX(), capturePointCornerPoints
            .get(1).getY());
      
      tmp2dFramePoint2.setIncludingFrame(footstepList.get(0).getReferenceFrame(), footstepList.get(0).getX(), footstepList.get(0).getY());
      
      frameLine2d.set(tmp2dFramePoint1, tmp2dFramePoint2);
      
      CapturePointTools.computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(actualCapturePointPosition, frameLine2d, tmp2dFramePoint3);

      capturePointCornerPoints.get(0).set(tmp2dFramePoint3.getX(), tmp2dFramePoint3.getY(), 0);
      desiredCapturePointPosition.set(tmp2dFramePoint3.getX(), tmp2dFramePoint3.getY(),0.0);
      
      CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), constantCentroidalMomentumPivots.get(0), this.omega0.getDoubleValue(),
            timeRemaining + doubleSupportDuration.getDoubleValue());

      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);

      initialTime.set(time);

      this.wasPushedInSingleSupport.set(false);
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

   private void cancelPlanNow(double time, ArrayList<FramePoint> footstepList)
   {
      this.footstepList.clear();
      for(int i = 0; i<footstepList.size(); i++)
      {
         this.footstepList.add(footstepList.get(i));
      }

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

      CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(singleSupportInitialDesiredCapturePointPosition,
            desiredCapturePointPosition, constantCentroidalMomentumPivots.get(0), omega0.getDoubleValue(), doubleSupportDuration.getDoubleValue());

      initialTime.set(time);

      cancelPlan.set(false);
   }

   public void resetParametersToDefault()
   {
      this.singleSupportDuration.set(this.capturePointPlannerParameters.getSingleSupportDuration());
      this.doubleSupportDuration.set(this.capturePointPlannerParameters.getDoubleSupportDuration());
      this.doubleSupportInitialTransferDuration.set(this.capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      this.numberFootstepsToConsider.set(this.capturePointPlannerParameters.getNumberOfFootstepsToConsider());
      this.doubleSupportSplitFraction.set(this.capturePointPlannerParameters.getDoubleSupportSplitFraction());
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

   public void setDoubleSupportTime(double time)
   {
      this.doubleSupportDuration.set(time);
   }

   public void setSingleSupportTime(double time)
   {
      this.singleSupportDuration.set(time);
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
      this.computeTimeInCurrentState(time);
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
      
      for(int i = 0; i<footstepList.size(); i++)
      {
         tmpArrayListOfFramePoints.add(footstepList.get(i));
      }
      
      this.initializeDoubleSupport(tmpFramePoint1, tmpFrameVector1, time, tmpArrayListOfFramePoints);
   }

   public boolean getHasBeenWokenUp()
   {
      return hasBeenWokenUp.getBooleanValue();
   }

   public void wakeUp()
   {
      this.hasBeenWokenUp.set(true);
   }

   public boolean isDone(double time)
   {
      double timeRemaining = computeAndReturnTimeRemaining(time);
      return (timeRemaining <= isDoneTimeThreshold.getDoubleValue());
   }
}
