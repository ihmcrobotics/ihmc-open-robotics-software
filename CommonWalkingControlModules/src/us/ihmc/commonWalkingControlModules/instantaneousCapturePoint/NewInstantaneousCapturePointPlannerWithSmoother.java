package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.DoubleSupportPolynomialTrajectory;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("icpPlannerDoubleSupportSplitFractior", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("icpPlannerInitialTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable("", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable("icpPlannerNumberFootstepsToStop", registry);
   private final YoFramePoint singleSupportInitialDesiredCapturePointPosition = new YoFramePoint("icpFinalDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector singleSupportInitialDesiredCapturePointVelocity = new YoFrameVector("icpFinalDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint("icpPlannerDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector("icpPlannerDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector("icpPlannerDesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);
   private final ArrayList<YoFramePoint> constantCentersOfPressure = new ArrayList<YoFramePoint>();
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

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         YoFramePoint constantCopYoFramePoint = new YoFramePoint("icpConstantCoP" + i, worldFrame, registry);
         constantCentersOfPressure.add(constantCopYoFramePoint);
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
            YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition("icpConstantCoP" + i, constantCentersOfPressure.get(i),
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

   public void initializeDoubleSupport(YoFramePoint currentDesiredCapturePointPosition, YoFrameVector currentDesiredCapturePointVelocity, double omega0,
         double initialTime, ArrayList<YoFramePoint> footstepList)
   {
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      this.isDoubleSupport.set(true);
      this.omega0.set(omega0);
      this.initialTime.set(initialTime);

      this.desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      this.desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());

      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(1),constantCentersOfPressure.get(1), doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue());
      
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
         initializeDoubleSupportCapturePointTrajectory(currentDesiredCapturePointPosition, currentDesiredCapturePointVelocity,
               singleSupportInitialDesiredCapturePointPosition, singleSupportInitialDesiredCapturePointVelocity, doubleSupportInitialTransferDuration);
         atAStop.set(false);
      }
      else
      {
         initializeDoubleSupportCapturePointTrajectory(currentDesiredCapturePointPosition, currentDesiredCapturePointVelocity,
               singleSupportInitialDesiredCapturePointPosition, singleSupportInitialDesiredCapturePointVelocity, doubleSupportDuration);
      }

      if (wasPushedInSingleSupport.getBooleanValue())
      {
         wasPushedInSingleSupport.set(false);
      }
   }

   public void initializeSingleSupport(double omega0, double initialTime, ArrayList<YoFramePoint> footstepList)
   {
      this.isDoubleSupport.set(false);
      this.omega0.set(omega0);
      this.initialTime.set(initialTime);
      this.isInitialTransfer.set(false);
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      atAStop.set(false);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue());
      
      computeUpcomingSingleSupportInitialDesiredCapturePointPosition(capturePointCornerPoints.get(0), constantCentersOfPressure.get(0), doubleSupportSplitFraction.getDoubleValue()*doubleSupportDuration.getDoubleValue());
   }
   
   private void computeUpcomingSingleSupportInitialDesiredCapturePointPosition(YoFramePoint initialCapturePointPosition, YoFramePoint initialCenterOfPressurePosition, double time)
   {
      CapturePointTools.computeDesiredCapturePointPosition(this.omega0.getDoubleValue(), time, initialCapturePointPosition, initialCenterOfPressurePosition, singleSupportInitialDesiredCapturePointPosition);
   }

   private void computeInitialVelocityOfUpcomingSingleSupportPhase()
   {
      CapturePointTools.computeDesiredCapturePointVelocity(this.omega0.getDoubleValue(), 0.0, singleSupportInitialDesiredCapturePointPosition,
            constantCentersOfPressure.get(1), singleSupportInitialDesiredCapturePointVelocity);
   }

   protected void computeConstantCentersOfPressure(ArrayList<YoFramePoint> footstepList)
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
               CapturePointTools.computeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet(constantCentersOfPressure, footstepList,
                     numberOfCentersOfPressureToPlan);
            }
            else
            {
               CapturePointTools.computeConstantCentersOfPressuresWithBeginningAndEndBetweenFeetRestOnFeet(constantCentersOfPressure, footstepList,
                     numberOfCentersOfPressureToPlan);
            }
         }
         else
         {
            if (comeToStop.getBooleanValue())
            {
               CapturePointTools.computeConstantCentersOfPressuresOnFeetWithEndBetweenFeet(constantCentersOfPressure, footstepList,
                     numberOfCentersOfPressureToPlan);
            }
            else
            {
               CapturePointTools.computeConstantCentersOfPressuresOnFeet(constantCentersOfPressure, footstepList, numberOfCentersOfPressureToPlan);
            }
         }
      }
      else
      {
         CapturePointTools.computeConstantCentersOfPressuresExceptFirstOnFeet(constantCentersOfPressure, footstepList,
               numberFootstepsToConsider.getIntegerValue());
      }
   }
   
   protected void initializeDoubleSupportCapturePointTrajectory(YoFramePoint initialCapturePointPosition, YoFrameVector initialCapturePointVelocity,
         YoFramePoint finalDesiredCapturePointPosition, YoFrameVector finalDesiredCapturePointVelocity, DoubleYoVariable doubleSupportDuration)
   {
      doubleSupportCapturePointTrajectory.initialize(doubleSupportDuration.getDoubleValue(), initialCapturePointPosition, initialCapturePointVelocity,
            finalDesiredCapturePointPosition, finalDesiredCapturePointVelocity);
   }

   protected void computeCapturePointCornerPoints(double steppingDuration)
   {
      if (!wasPushedInSingleSupport.getBooleanValue())
      {
         CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentersOfPressure, capturePointCornerPoints, steppingDuration,
               omega0.getDoubleValue());
      }
      else
      {
         CapturePointTools.computeDesiredEndOfStepCapturePointLocationsWithFirstLeftUnset(constantCentersOfPressure, capturePointCornerPoints,
               this.doubleSupportDuration.getDoubleValue() + this.singleSupportDuration.getDoubleValue(), omega0.getDoubleValue());
      }
   }

   protected void computeDesiredCapturePointPosition(double time)
   {
      if (!isDoubleSupport.getBooleanValue())
      {
         YoFramePoint initialCapturePoint = singleSupportInitialDesiredCapturePointPosition;
         YoFramePoint initialCenterOfPressure = constantCentersOfPressure.get(0);

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
         YoFramePoint initialCenterOfPressure = constantCentersOfPressure.get(0);

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
         YoFramePoint initialCenterOfPressure = constantCentersOfPressure.get(0);

         CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure,
               desiredCapturePointAcceleration);
      }
      else
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.getAcceleration(desiredCapturePointAcceleration);
      }
   }

   protected void computeDesiredCornerPoints(ArrayList<YoFramePoint> constantCentersOfPressure, double stepTime, double omega0)
   {
      CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentersOfPressure, capturePointCornerPoints, stepTime, omega0);
   }

   public void packDesiredCapturePointPositionVelocityAndAcceleration(YoFramePoint desiredCapturePointPositionToPack,
         YoFrameVector desiredCapturePointVelocityToPack, YoFrameVector desiredCapturePointAccelerationToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePointPosition(timeInCurrentState.getDoubleValue());
      computeDesiredCapturePointVelocity(timeInCurrentState.getDoubleValue());
      computeDesiredCapturePointAcceleration(timeInCurrentState.getDoubleValue());

      desiredCapturePointPositionToPack.set(this.desiredCapturePointPosition);
      desiredCapturePointVelocityToPack.set(this.desiredCapturePointVelocity);
      desiredCapturePointAccelerationToPack.set(this.desiredCapturePointAcceleration);
   }

   public void updateForSingleSupportPush(ArrayList<YoFramePoint> footstepList, double time)
   {
      computeTimeInCurrentState(time);
      double timeRemaining = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();

      this.wasPushedInSingleSupport.set(true);

      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      capturePointCornerPoints.get(0).set(desiredCapturePointPosition);

      CapturePointTools.computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(capturePointCornerPoints.get(1),
            capturePointCornerPoints.get(0), constantCentersOfPressure.get(0), this.omega0.getDoubleValue(),
            timeRemaining + doubleSupportDuration.getDoubleValue());

      singleSupportInitialDesiredCapturePointPosition.set(desiredCapturePointPosition);
      
      initialTime.set(time);
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

   public YoFramePoint getDesiredCapturePointPosition()
   {
      return desiredCapturePointPosition;
   }

   public ArrayList<YoFramePoint> getConstantCentersOfPressure()
   {
      return constantCentersOfPressure;
   }

   public ArrayList<YoFramePoint> getCapturePointCornerPoints()
   {
      return capturePointCornerPoints;
   }

   public void packDesiredCapturePointIntoPoint3d(Point3d pointToPack)
   {
      desiredCapturePointPosition.get(pointToPack);
   }

   public YoFrameVector getDesiredCapturePointVelocity()
   {
      return desiredCapturePointVelocity;
   }

   public void packDesiredCapturePointVelocityIntoVector3d(Vector3d vectorToPack)
   {
      desiredCapturePointVelocity.get(vectorToPack);
   }

   public YoFrameVector getDesiredCapturePointAcceleration()
   {
      return desiredCapturePointAcceleration;
   }

   public void packDesiredCapturePointAccelerationIntoVector3d(Vector3d vectorToPack)
   {
      desiredCapturePointAcceleration.get(vectorToPack);
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
