package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

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
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class NewInstantaneousCapturePointPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private BooleanYoVariable VISUALIZE = new BooleanYoVariable("icpPlannerVISUALIZE", registry);
   private double ICP_CORNER_POINT_SIZE = 0.004;
   private double ICP_CONSTANT_COP_POINT_SIZE = 0.005;

   ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FramePoint tmpFramePoint1 = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint2 = new FramePoint(worldFrame);
   private final FrameVector tmpFrameVector1 = new FrameVector(worldFrame);
   private final FrameVector tmpFrameVector2 = new FrameVector(worldFrame);
   private final BooleanYoVariable atAStop = new BooleanYoVariable("icpPlannerAtAStop", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable("icpPlannerComeToStop", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable("icpPlannerIsInitialTransfer", registry);
   private final BooleanYoVariable wasPushedInSingleSupport = new BooleanYoVariable("icpPlannerWasPushedInSingleSupport", registry);
   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable("icpPlannerIsDoubleSupport", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("icpPlannerTimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable("icpPlannerisDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("icpPlannerDoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("icpPlannerSingleSupportTime", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable("icpPlannerInitialTransferDuration", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("icpPlannerInitialTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable("", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable("icpPlannerNumberFootstepsToStop", registry);
   private final YoFramePoint finalDesiredCapturePointPosition = new YoFramePoint("icpFinalDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector finalDesiredCapturePointVelocity = new YoFrameVector("icpFinalDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint("icpPlannerDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector("icpPlannerDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector("icpPlannerDesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);
   private final ArrayList<YoFramePoint> constantCentroidalMomentumPivots = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> capturePointCornerPoints = new ArrayList<YoFramePoint>();

   private final DoubleSupportPolynomialTrajectory doubleSupportCapturePointTrajectory;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   public NewInstantaneousCapturePointPlanner(int maxNumberFootstepsToConsider, CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE.set(false);
      }

      this.capturePointPlannerParameters = capturePointPlannerParameters;
      this.numberFootstepsToConsider.set(maxNumberFootstepsToConsider);
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

   public void initializeDoubleSupport(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity, double initialTime,
         ArrayList<FramePoint> footstepList)
   {
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      this.isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      this.desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      this.desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());

      finalDesiredCapturePointPosition.set(capturePointCornerPoints.get(1));
      finalDesiredCapturePointVelocity.set(0.0, 0.0, 0.0);

      if (atAStop.getBooleanValue())
      {
         initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity, finalDesiredCapturePointPosition,
               finalDesiredCapturePointVelocity, doubleSupportInitialTransferDuration);
         atAStop.set(false);
      }
      else
      {
         initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity, finalDesiredCapturePointPosition,
               finalDesiredCapturePointVelocity, doubleSupportDuration);
      }

      if (wasPushedInSingleSupport.getBooleanValue())
      {
         wasPushedInSingleSupport.set(false);
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

   public void initializeSingleSupport(double initialTime, ArrayList<FramePoint> footstepList)
   {
      this.isDoubleSupport.set(false);
      this.initialTime.set(initialTime);
      this.isInitialTransfer.set(false);
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      atAStop.set(false);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());
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
         YoFramePoint initialCapturePoint = capturePointCornerPoints.get(0);
         YoFramePoint initialCenterOfPressure = constantCentroidalMomentumPivots.get(0);

         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure,
               desiredCapturePointPosition);
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
         YoFramePoint initialCapturePoint = capturePointCornerPoints.get(0);
         YoFramePoint initialCenterOfPressure = constantCentroidalMomentumPivots.get(0);

         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure,
               desiredCapturePointVelocity);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.getVelocity(tmpFrameVector1);
         desiredCapturePointVelocity.set(tmpFrameVector1);
      }
   }

   protected void computeDesiredCapturePointAcceleration(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         YoFramePoint initialCapturePoint = capturePointCornerPoints.get(0);
         YoFramePoint initialCenterOfPressure = constantCentroidalMomentumPivots.get(0);

         CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure,
               desiredCapturePointAcceleration);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.getAcceleration(tmpFrameVector1);
         desiredCapturePointAcceleration.set(tmpFrameVector1);
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

      initialTime.set(time);
   }

   protected void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   public double computeTimeRemaining(double time)
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

   public void resetParametersToDefault()
   {
      this.singleSupportDuration.set(this.capturePointPlannerParameters.getSingleSupportDuration());
      this.doubleSupportDuration.set(this.capturePointPlannerParameters.getDoubleSupportDuration());
      this.doubleSupportInitialTransferDuration.set(this.capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      this.numberFootstepsToConsider.set(this.capturePointPlannerParameters.getNumberOfFootstepsToConsider());
   }

   public void cancelPlan(double time, ArrayList<FramePoint> footstepList)
   {
      capturePointCornerPoints.get(0).set(desiredCapturePointPosition);

      finalDesiredCapturePointPosition.set(footstepList.get(0));
      finalDesiredCapturePointPosition.add(footstepList.get(1));
      finalDesiredCapturePointPosition.scale(0.5);

      finalDesiredCapturePointVelocity.set(0.0, 0.0, 0.0);

      for (int i = 1; i < constantCentroidalMomentumPivots.size(); i++)
      {
         constantCentroidalMomentumPivots.get(i).set(finalDesiredCapturePointPosition);
      }

      for (int i = 1; i < capturePointCornerPoints.size(); i++)
      {
         capturePointCornerPoints.get(i).set(finalDesiredCapturePointPosition);
      }

      initializeDoubleSupportCapturePointTrajectory(desiredCapturePointPosition, desiredCapturePointVelocity, finalDesiredCapturePointPosition,
            finalDesiredCapturePointVelocity, doubleSupportDuration);

      CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(finalDesiredCapturePointPosition, desiredCapturePointPosition,
            constantCentroidalMomentumPivots.get(0), omega0.getDoubleValue(), doubleSupportDuration.getDoubleValue());

      initialTime.set(time);
      
      atAStop.set(true);
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

   public boolean isDone(double time)
   {
      double timeRemaining = computeTimeRemaining(time);
      return (timeRemaining <= isDoneTimeThreshold.getDoubleValue());
   }
}
