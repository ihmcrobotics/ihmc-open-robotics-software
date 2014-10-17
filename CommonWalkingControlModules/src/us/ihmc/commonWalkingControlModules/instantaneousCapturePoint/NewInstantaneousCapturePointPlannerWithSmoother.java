package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.DoubleSupportPolynomialTrajectory;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class NewInstantaneousCapturePointPlannerWithSmoother
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private BooleanYoVariable VISUALIZE = new BooleanYoVariable("icpPlannerVISUALIZE", registry);
   private double ICP_CORNER_POINT_SIZE = 0.004;
   private double ICP_CONSTANT_COP_POINT_SIZE = 0.005;
   
   private final FramePoint tmpFramePoint1 = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint2 = new FramePoint(worldFrame);
   private final FrameVector tmpFrameVector1 = new FrameVector(worldFrame);
   private final FrameVector tmpFrameVector2 = new FrameVector(worldFrame);
   
   private final BooleanYoVariable atAStop = new BooleanYoVariable("icpPlannerAtAStop", registry);
   private final BooleanYoVariable cancelPlan = new BooleanYoVariable("icpPlannerCancelPlan", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable("icpPlannerComeToStop", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable("icpPlannerIsInitialTransfer", registry);
   private final BooleanYoVariable wasPushedInSingleSupport = new BooleanYoVariable("icpPlannerWasPushedInSingleSupport", registry);
   protected final BooleanYoVariable isDoubleSupport = new BooleanYoVariable("icpPlannerIsDoubleSupport", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("icpPlannerTimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable("icpPlannerisDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("icpPlannerDoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("icpPlannerSingleSupportTime", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable("icpPlannerInitialTransferDuration", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("icpPlannerDoubleSupportSplitFractior", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("icpPlannerInitialTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable("", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable("icpPlannerNumberFootstepsToStop", registry);
   private final YoFramePoint singleSupportInitialDesiredCapturePointPosition = new YoFramePoint("icpFinalDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector singleSupportInitialDesiredCapturePointVelocity = new YoFrameVector("icpFinalDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFramePoint desiredCentroidalMomentumPivotPosition = new YoFramePoint("icpPlannerDesiredCentroidalMomentumPosition", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint("icpPlannerDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector("icpPlannerDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector("icpPlannerDesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);
   private final ArrayList<YoFramePoint> constantCentroidalMomentumPivots = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> capturePointCornerPoints = new ArrayList<YoFramePoint>();

   private final DoubleSupportPolynomialTrajectory doubleSupportCapturePointTrajectory;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   public NewInstantaneousCapturePointPlannerWithSmoother(CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE.set(false);
      }
      
      cancelPlan.set(false);

      this.capturePointPlannerParameters = capturePointPlannerParameters;
      this.atAStop.set(true);
      this.wasPushedInSingleSupport.set(false);

      this.doubleSupportCapturePointTrajectory = new DoubleSupportPolynomialTrajectory("icpPlannerDoubleSupportTrajectory",
            this.capturePointPlannerParameters.getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory(), worldFrame, registry);
      this.singleSupportDuration.set(this.capturePointPlannerParameters.getSingleSupportDuration());
      this.doubleSupportDuration.set(this.capturePointPlannerParameters.getDoubleSupportDuration());
      this.doubleSupportInitialTransferDuration.set(this.capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      this.numberFootstepsToConsider.set(this.capturePointPlannerParameters.getNumberOfFootstepsToConsider());
      this.footstepsToStop.set(this.capturePointPlannerParameters.getNumberOfFootstepsToStop());
      this.isDoneTimeThreshold.set(this.capturePointPlannerParameters.getIsDoneTimeThreshold());
      this.doubleSupportSplitFraction.set(this.capturePointPlannerParameters.getDoubleSupportSplitFraction());
      //Initialize omega0 to NaN to force the user to explicitly set it.
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

      if (VISUALIZE.getBooleanValue())
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
         if (VISUALIZE.getBooleanValue())
         {
            YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition("icpConstantCoP" + i, constantCentroidalMomentumPivots.get(i),
                  ICP_CONSTANT_COP_POINT_SIZE, YoAppearance.Green(), GraphicType.SOLID_BALL);

            yoGraphicsList.add(constantFootCenterCentersOfPressureViz);
            artifactList.add(constantFootCenterCentersOfPressureViz.createArtifact());
         }
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         if (VISUALIZE.getBooleanValue())
         {
            YoGraphicPosition icpCornerPointsViz = new YoGraphicPosition("icpCornerPoints" + i, capturePointCornerPoints.get(i), ICP_CORNER_POINT_SIZE,
                  YoAppearance.Green(), GraphicType.SOLID_BALL);

            yoGraphicsList.add(icpCornerPointsViz);
            artifactList.add(icpCornerPointsViz.createArtifact());
         }
      }

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void initializeDoubleSupport(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity,
         double initialTime, ArrayList<FramePoint> footstepList)
   {
	   this.isDoubleSupport.set(true);
	   
	   if(cancelPlan.getBooleanValue())
	   {
		   cancelPlan(initialTime, footstepList);
		   atAStop.set(true);
		   return;
	   }
	   
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      this.initialTime.set(initialTime);

      this.desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      this.desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());

      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(1),constantCentroidalMomentumPivots.get(1), doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());
      
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

      if (wasPushedInSingleSupport.getBooleanValue())
      {
         wasPushedInSingleSupport.set(false);
      }
   }

   public void initializeSingleSupport(double initialTime, ArrayList<FramePoint> footstepList)
   {
      atAStop.set(false);
      this.isDoubleSupport.set(false);
      this.initialTime.set(initialTime);
      this.isInitialTransfer.set(false);
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());
      
      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(0), constantCentroidalMomentumPivots.get(0), doubleSupportSplitFraction.getDoubleValue()*doubleSupportDuration.getDoubleValue());
   }
   
   private void computeUpcomingSingleSupportInitialDesiredCapturePointPosition(YoFramePoint initialCapturePointPosition, YoFramePoint initialCenterOfPressurePosition, double time)
   {
      CapturePointTools.computeDesiredCapturePointPosition(this.omega0.getDoubleValue(), time, initialCapturePointPosition, initialCenterOfPressurePosition, singleSupportInitialDesiredCapturePointPosition);
   }

   private void computeInitialVelocityOfUpcomingSingleSupportPhase()
   {
      CapturePointTools.computeDesiredCapturePointVelocity(this.omega0.getDoubleValue(), 0.0, singleSupportInitialDesiredCapturePointPosition,
            constantCentroidalMomentumPivots.get(1), singleSupportInitialDesiredCapturePointVelocity);
   }

   protected void computeConstantCentersOfPressure(ArrayList<FramePoint> footstepList)
   {
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());

      int numberOfCentersOfPressureToPlan = (this.numberFootstepsToConsider.getIntegerValue() > footstepList.size()) ? footstepList.size()
            : this.numberFootstepsToConsider.getIntegerValue();

      if (!wasPushedInSingleSupport.getBooleanValue())
      {
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
      else
      {
         CapturePointTools.computeConstantCentersOfPressuresExceptFirstOnFeet(constantCentroidalMomentumPivots, footstepList,
               numberFootstepsToConsider.getIntegerValue());
      }
   }
   
   protected void initializeDoubleSupportCapturePointTrajectory(YoFramePoint initialCapturePointPosition, YoFrameVector initialCapturePointVelocity,
         YoFramePoint finalDesiredCapturePointPosition, YoFrameVector finalDesiredCapturePointVelocity, DoubleYoVariable doubleSupportDuration)
   {
      initialCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      finalDesiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint2);
      initialCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);
      finalDesiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector2);
      
      doubleSupportCapturePointTrajectory.initialize(doubleSupportDuration.getDoubleValue(), tmpFramePoint1, tmpFrameVector1,
            tmpFramePoint2, tmpFrameVector2);
   }

   protected void computeCapturePointCornerPoints(double steppingDuration)
   {
      if (!wasPushedInSingleSupport.getBooleanValue())
      {
         CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentroidalMomentumPivots, capturePointCornerPoints, steppingDuration,
               omega0.getDoubleValue());
      }
      else
      {
         CapturePointTools.computeDesiredEndOfStepCapturePointLocationsWithFirstLeftUnset(constantCentroidalMomentumPivots, capturePointCornerPoints,
               this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue(), omega0.getDoubleValue());
      }
   }

   protected void computeDesiredCapturePointPosition(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         YoFramePoint initialCapturePoint = singleSupportInitialDesiredCapturePointPosition;
         YoFramePoint initialCenterOfPressure = constantCentroidalMomentumPivots.get(0);

         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure,
               desiredCapturePointPosition);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.getPosition(desiredCapturePointPosition);
      }
   }

   protected void computeDesiredCapturePointVelocity(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         YoFramePoint initialCapturePoint = singleSupportInitialDesiredCapturePointPosition;
         YoFramePoint initialCenterOfPressure = constantCentroidalMomentumPivots.get(0);

         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure,
               desiredCapturePointVelocity);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.getVelocity(desiredCapturePointVelocity);
      }
   }

   protected void computeDesiredCapturePointAcceleration(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         YoFramePoint initialCapturePoint = singleSupportInitialDesiredCapturePointPosition;
         YoFramePoint initialCenterOfPressure = constantCentroidalMomentumPivots.get(0);

         CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure,
               desiredCapturePointAcceleration);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.getAcceleration(desiredCapturePointAcceleration);
      }
   }
   
   protected void computeDesiredCentroidalMomentumPivot()
   {
      if(isDoubleSupport.getBooleanValue())
      {
         CapturePointTools.computeDesiredCentroidalMomentumPivot(singleSupportInitialDesiredCapturePointPosition, desiredCapturePointVelocity, omega0.getDoubleValue(), desiredCentroidalMomentumPivotPosition);
      }
      else
      {
         desiredCentroidalMomentumPivotPosition.set(constantCentroidalMomentumPivots.get(0));
      }
   }

   protected void computeDesiredCornerPoints(ArrayList<YoFramePoint> constantCentersOfPressure, double stepTime, double omega0)
   {
      CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentersOfPressure, capturePointCornerPoints, stepTime, omega0);
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
   
   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack,
         FrameVector desiredCapturePointVelocityToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePointPosition(timeInCurrentState.getDoubleValue());
      computeDesiredCapturePointVelocity(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(tmpFramePoint1);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(tmpFrameVector1);
      
      desiredCapturePointPositionToPack.set(tmpFramePoint1);
      desiredCapturePointVelocityToPack.set(tmpFrameVector1);
   }
   
   protected void packDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack,
         YoFrameVector desiredCapturePointVelocityToPack, double time)
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

   public void updateForSingleSupportPush(ArrayList<FramePoint> footstepList, double time)
   {
      computeTimeInCurrentState(time);
      double timeRemaining = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();

      this.wasPushedInSingleSupport.set(true);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      capturePointCornerPoints.get(0).set(desiredCapturePointPosition);

      CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), constantCentroidalMomentumPivots.get(0), this.omega0.getDoubleValue(),
            timeRemaining + doubleSupportDuration.getDoubleValue());

      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);
      
      initialTime.set(time);
   }
   
   public void cancelPlan(double time, ArrayList<FramePoint> footstepList)
   {
	   if(isDoubleSupport.getBooleanValue())
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

      initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity, singleSupportInitialDesiredCapturePointPosition, singleSupportInitialDesiredCapturePointVelocity, doubleSupportDuration);

      CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(singleSupportInitialDesiredCapturePointPosition, desiredCapturePointPosition,
            constantCentroidalMomentumPivots.get(0), omega0.getDoubleValue(), doubleSupportDuration.getDoubleValue());

      initialTime.set(time);
      
      cancelPlan.set(false);
   }

   public void resetParametersToDefault()
   {
      this.singleSupportDuration.set(this.capturePointPlannerParameters.getSingleSupportDuration());
      this.doubleSupportDuration.set(this.capturePointPlannerParameters.getDoubleSupportDuration());
      this.doubleSupportInitialTransferDuration.set(this.capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      this.numberFootstepsToConsider.set(this.capturePointPlannerParameters.getNumberOfFootstepsToConsider());
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
            return (doubleSupportInitialTransferDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
         }
         else
         {
            return (doubleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
         }
      }
      else
      {
         return (singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
      }
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

   public boolean isDone(double time)
   {
      double timeRemaining = computeAndReturnTimeRemaining(time);
      return (timeRemaining <= isDoneTimeThreshold.getDoubleValue());
   }
}
