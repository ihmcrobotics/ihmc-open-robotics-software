package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;

public class NewInstantaneousCapturePointPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double ICP_CORNER_POINT_SIZE = 0.004;
   private static final double ICP_CONSTANT_COP_POINT_SIZE = 0.005;

   private final String namePrefix = "icpPlanner";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable visualize = new BooleanYoVariable(namePrefix + "Visualize", registry);

   private final FramePoint tmpFramePoint1 = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint2 = new FramePoint(worldFrame);
   private final FrameVector tmpFrameVector1 = new FrameVector(worldFrame);
   private final FrameVector tmpFrameVector2 = new FrameVector(worldFrame);

   private final BooleanYoVariable atAStop = new BooleanYoVariable(namePrefix + "AtAStop", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable(namePrefix + "ComeToStop", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable(namePrefix + "IsInitialTransfer", registry);
   private final BooleanYoVariable wasPushedInSingleSupport = new BooleanYoVariable(namePrefix + "WasPushedInSingleSupport", registry);
   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable(namePrefix + "IsDoubleSupport", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(namePrefix + "TimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable(namePrefix + "isDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(namePrefix + "DoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(namePrefix + "SingleSupportTime", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable(namePrefix + "InitialTransferDuration", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable(namePrefix + "NumberOfFootstepsToConsider", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable(namePrefix + "NumberOfFootstepsToStop", registry);
   private final YoFramePoint finalDesiredCapturePointPosition = new YoFramePoint("icpFinalDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector finalDesiredCapturePointVelocity = new YoFrameVector("icpFinalDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint(namePrefix + "DesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector(namePrefix + "DesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector(namePrefix + "DesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable(namePrefix + "Omega0", registry);
   private final ArrayList<YoFramePoint> constantCentroidalMomentumPivots = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> capturePointCornerPoints = new ArrayList<YoFramePoint>();
   private final FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList();

   private final VelocityConstrainedPositionTrajectoryGenerator doubleSupportCapturePointTrajectory;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   public NewInstantaneousCapturePointPlanner(int maxNumberFootstepsToConsider, CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
      {
         visualize.set(false);
      }

      this.capturePointPlannerParameters = capturePointPlannerParameters;
      numberFootstepsToConsider.set(maxNumberFootstepsToConsider);
      atAStop.set(true);
      wasPushedInSingleSupport.set(false);

      int numberOfCoefficientsForDoubleSupportPolynomialTrajectory = this.capturePointPlannerParameters
            .getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory();
      doubleSupportCapturePointTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix + "DoubleSupportTrajectory",
            numberOfCoefficientsForDoubleSupportPolynomialTrajectory, worldFrame, registry);
      singleSupportDuration.set(this.capturePointPlannerParameters.getSingleSupportDuration());
      doubleSupportDuration.set(this.capturePointPlannerParameters.getDoubleSupportDuration());
      doubleSupportInitialTransferDuration.set(this.capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      numberFootstepsToConsider.set(this.capturePointPlannerParameters.getNumberOfFootstepsToConsider());
      footstepsToStop.set(this.capturePointPlannerParameters.getNumberOfFootstepsToStop());
      isDoneTimeThreshold.set(this.capturePointPlannerParameters.getIsDoneTimeThreshold());

      omega0.set(Double.NaN);

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         YoFramePoint constantCopYoFramePoint = new YoFramePoint("icpConstantCMP" + i, worldFrame, registry);
         constantCentroidalMomentumPivots.add(constantCopYoFramePoint);
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePoint icpCornerPointYoFramePoint = new YoFramePoint("icpCornerPoints" + i, worldFrame, registry);
         capturePointCornerPoints.add(icpCornerPointYoFramePoint);
      }

      parentRegistry.addChild(registry);

      if (visualize.getBooleanValue())
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
         if (visualize.getBooleanValue())
         {
            YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition("icpConstantCoP" + i, constantCentroidalMomentumPivots.get(i),
                  ICP_CONSTANT_COP_POINT_SIZE, YoAppearance.Green(), GraphicType.SOLID_BALL);

            yoGraphicsList.add(constantFootCenterCentersOfPressureViz);
            artifactList.add(constantFootCenterCentersOfPressureViz.createArtifact());
         }
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         if (visualize.getBooleanValue())
         {
            YoGraphicPosition icpCornerPointsViz = new YoGraphicPosition("icpCornerPoints" + i, capturePointCornerPoints.get(i), ICP_CORNER_POINT_SIZE,
                  YoAppearance.Blue(), GraphicType.SOLID_BALL);

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
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);

      this.footstepList.copyFromListAndTrimSize(footstepList);

      computeConstantCMPs(this.footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

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

      doubleSupportCapturePointTrajectory.setTrajectoryParameters(doubleSupportDuration.getDoubleValue(), tmpFramePoint1, tmpFrameVector1, tmpFramePoint2,
            tmpFrameVector2);
      doubleSupportCapturePointTrajectory.initialize();
   }

   public void initializeSingleSupport(double initialTime, ArrayList<FramePoint> footstepList)
   {
      isDoubleSupport.set(false);
      this.initialTime.set(initialTime);
      isInitialTransfer.set(false);
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());
      atAStop.set(false);

      this.footstepList.copyFromListAndTrimSize(footstepList);
      computeConstantCMPs(this.footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());
   }

   protected void computeConstantCMPs(RecyclingArrayList<FramePoint> footstepList)
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

   protected void computeCapturePointCornerPoints(double steppingDuration)
   {
      boolean skipFirstCornerPoint = wasPushedInSingleSupport.getBooleanValue();
      CapturePointTools.computeDesiredCornerPoints(capturePointCornerPoints, constantCentroidalMomentumPivots, skipFirstCornerPoint, steppingDuration, omega0.getDoubleValue());
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
         doubleSupportCapturePointTrajectory.get(tmpFramePoint1);
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
         doubleSupportCapturePointTrajectory.packVelocity(tmpFrameVector1);
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
         doubleSupportCapturePointTrajectory.packAcceleration(tmpFrameVector1);
         desiredCapturePointAcceleration.set(tmpFrameVector1);
      }
   }

   protected void computeDesiredCornerPoints(ArrayList<YoFramePoint> constantCentersOfPressure, double stepTime, double omega0)
   {
      CapturePointTools.computeDesiredCornerPoints(capturePointCornerPoints, constantCentersOfPressure, false, stepTime, omega0);
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

      wasPushedInSingleSupport.set(true);

      this.footstepList.copyFromListAndTrimSize(footstepList);
      computeConstantCMPs(this.footstepList);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      capturePointCornerPoints.get(0).set(desiredCapturePointPosition);

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0),
            capturePointCornerPoints.get(1), capturePointCornerPoints.get(0), omega0.getDoubleValue(),
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
      singleSupportDuration.set(capturePointPlannerParameters.getSingleSupportDuration());
      doubleSupportDuration.set(capturePointPlannerParameters.getDoubleSupportDuration());
      doubleSupportInitialTransferDuration.set(capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      numberFootstepsToConsider.set(capturePointPlannerParameters.getNumberOfFootstepsToConsider());
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

      CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCentroidalMomentumPivots.get(0), finalDesiredCapturePointPosition,
            desiredCapturePointPosition, omega0.getDoubleValue(), doubleSupportDuration.getDoubleValue());

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
      doubleSupportDuration.set(time);
   }

   public void setSingleSupportTime(double time)
   {
      singleSupportDuration.set(time);
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
