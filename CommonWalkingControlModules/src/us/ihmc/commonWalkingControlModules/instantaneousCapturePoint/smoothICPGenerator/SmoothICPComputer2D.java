package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.InstantaneousCapturePointPlanner;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class SmoothICPComputer2D extends DoubleSupportFootCenterToToeICPComputer implements InstantaneousCapturePointPlanner
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable percentToScaleBackOnVelocity = new DoubleYoVariable("percentToScaleBackOnVelocity", registry);
   private final DoubleYoVariable deltaFootDistance = new DoubleYoVariable("deltaFootDistance", registry);

   private final YoFramePoint initialFootPlacement = new YoFramePoint("initialFootPlacement", worldFrame, registry);
   private final YoFramePoint currentFootPlacement = new YoFramePoint("currentFootPlacement", worldFrame, registry);
   private final EnumYoVariable<RobotSide> currentTransferToSide = new EnumYoVariable<RobotSide>("currentTransferToSide", registry, RobotSide.class);

   private final DoubleYoVariable percentIn = new DoubleYoVariable("percentIn", registry);

   private final FramePoint footLocationTemp = new FramePoint();
   private final CommonHumanoidReferenceFrames referenceFrames;

   private final DoubleYoVariable alphaDeltaFootPosition = new DoubleYoVariable("alphaDeltaFootPosition", registry);
   private final AlphaFilteredYoFrameVector deltaFootPosition = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("deltaFootPosition", "", registry,
         alphaDeltaFootPosition, worldFrame);
   private final Vector3d deltaVectorTemp = new Vector3d();

   public SmoothICPComputer2D(CommonHumanoidReferenceFrames referenceFrames, double dt, double doubleSupportFirstStepFraction,
         int maxNumberOfConsideredFootsteps, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(dt, doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps, parentRegistry, yoGraphicsListRegistry);

      alphaDeltaFootPosition.set(0.65);
      this.referenceFrames = referenceFrames;
   }

   private final Point3d icpPostionToPackTemp = new Point3d();
   private final Vector3d icpVelocityToPackTemp = new Vector3d();
   private final Vector3d icpAccelerationToPackTemp = new Vector3d();
   private final Point3d ecmpToPackTemp = new Point3d();

   public void getICPPositionAndVelocity(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, FramePoint2d actualICP,
         double time)
   {
      super.computeICPPositionVelocityAcceleration(icpPostionToPackTemp, icpVelocityToPackTemp, icpAccelerationToPackTemp, ecmpToPackTemp, time);

      RobotSide transferToSide = currentTransferToSide.getEnumValue();

      if ((transferToSide != null) && isPerformingICPDoubleSupport())
      {
         getFootPlacement(transferToSide, currentFootPlacement);

         double deltaX = currentFootPlacement.getX() - initialFootPlacement.getX();
         double deltaY = currentFootPlacement.getY() - initialFootPlacement.getY();
         deltaFootPosition.update(deltaX, deltaY, 0.0);
         deltaFootDistance.set(deltaFootPosition.length());

         //TODO: Do the delta proportionally to how long remaining better...

         double timeInState = super.getTimeInState(time);
         double timeLeft = super.getEstimatedTimeRemainingForState(time);

         double percentIn = timeInState / (timeInState + timeLeft);

         percentIn = MathTools.clipToMinMax(percentIn, 0.0, 1.0);
         this.percentIn.set(percentIn);
         deltaFootPosition.scale(percentIn);

         deltaVectorTemp.setX(deltaFootPosition.getX());
         deltaVectorTemp.setY(deltaFootPosition.getY());
         icpPostionToPackTemp.add(deltaVectorTemp);

         // Scale back on the velocity if the foot slipped a lot.
         // And when coming to the end of the trajectory.
         // This is very hackish. We should make the trajectories better so we don't have
         // to do this...
         percentToScaleBackOnVelocity.set(1.0 - deltaFootDistance.getDoubleValue() / 0.04);
         percentToScaleBackOnVelocity.set(MathTools.clipToMinMax(percentToScaleBackOnVelocity.getDoubleValue(), 0.0, 1.0));

         // At 95% in, should be at zero velocity.
         double percentToScaleDownAtEnd = 1.0 - (percentIn) / 0.95;
         percentToScaleDownAtEnd = MathTools.clipToMinMax(percentToScaleDownAtEnd, 0.0, 1.0);
         percentToScaleBackOnVelocity.set(percentToScaleBackOnVelocity.getDoubleValue() * percentToScaleDownAtEnd);
         icpVelocityToPackTemp.scale(percentToScaleBackOnVelocity.getDoubleValue());
      }

      icpPostionToPack.checkReferenceFrameMatch(worldFrame);
      icpVelocityToPack.checkReferenceFrameMatch(worldFrame);
      ecmpToPack.checkReferenceFrameMatch(worldFrame);

      icpPostionToPack.set(icpPostionToPackTemp.getX(), icpPostionToPackTemp.getY());
      icpVelocityToPack.set(icpVelocityToPackTemp.getX(), icpVelocityToPackTemp.getY());
      ecmpToPack.set(ecmpToPackTemp.getX(), ecmpToPackTemp.getY());
   }

   private final Point3d initialICPPositionTemp = new Point3d();

   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point2d initialICPPosition,
         double initialTime)
   {
      deltaFootPosition.reset();

      initialICPPositionTemp.set(initialICPPosition.getX(), initialICPPosition.getY(), 0.0);
      initializeDoubleSupportInitialTransfer(transferToAndNextFootstepsData, initialICPPositionTemp, initialTime);
   }

   public FramePoint2d getFinalDesiredICP()
   {
      Point3d upcomingCornerPoint = super.getUpcomingCornerPoint();

      //TODO: Need to use Frames throughout here, or don't assume this!
      FramePoint2d ret = new FramePoint2d(ReferenceFrame.getWorldFrame(), upcomingCornerPoint.getX(), upcomingCornerPoint.getY());
      return ret;
   }

   public FramePoint2d getConstantCenterOfPressure()
   {
      // Returns first element in list of constant centers of pressure from ICP planner.
      return super.getConstantCentersOfPressure().get(0).getFramePoint2dCopy();
   }

   @Override
   public void initializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      deltaFootPosition.reset();

      super.initializeSingleSupport(transferToAndNextFootstepsData, initialTime);
   }

   @Override
   public void reInitializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double currentTime)
   {
      deltaFootPosition.reset();

      super.reInitializeSingleSupport(transferToAndNextFootstepsData, currentTime);
   }

   @Override
   public void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      deltaFootPosition.reset();

      RobotSide transferToSide = transferToAndNextFootstepsData.getTransferToSide();

      currentTransferToSide.set(transferToSide);
      getFootPlacement(transferToSide, initialFootPlacement);

      super.initializeDoubleSupport(transferToAndNextFootstepsData, initialTime);
   }

   private void getFootPlacement(RobotSide transferToSide, YoFramePoint footPlacementToPack)
   {
      if (transferToSide != null)
      {
         footLocationTemp.setToZero(referenceFrames.getSoleFrame(transferToSide));
         footLocationTemp.changeFrame(ReferenceFrame.getWorldFrame());

         footPlacementToPack.set(footLocationTemp);
      }
   }

   @Override
   public void reset(double time)
   {
      deltaFootPosition.reset();
      super.reset(time);
   }

   @Override
   public boolean isDone(double time)
   {
      return super.isDone(time);
   }

   @Override
   public double getEstimatedTimeRemainingForState(double time)
   {
      return super.getEstimatedTimeRemainingForState(time);
   }

   @Override
   public boolean isPerformingICPDoubleSupport()
   {
      return super.isPerformingICPDoubleSupport();
   }

   @Override
   public double getTimeInState(double time)
   {
      return super.getTimeInState(time);
   }

   @Override
   public void updatePlanForSingleSupportPush(TransferToAndNextFootstepsData transferToAndNextFootstepsData, FramePoint actualCapturePointPosition,double time)
   {
      throw new RuntimeException("Not implemented.");      
   }

   @Override
   public void updatePlanForDoubleSupportPush(TransferToAndNextFootstepsData transferToAndNextFootstepsData, FramePoint actualCapturePointPosition, double time)
   {
      throw new RuntimeException("Not implemented.");
   }
}
