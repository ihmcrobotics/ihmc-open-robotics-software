package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.InstantaneousCapturePointPlanner;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SmoothICPComputer2D extends DoubleSupportFootCenterToToeICPComputer implements InstantaneousCapturePointPlanner
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoFramePoint initialFootPlacement = new YoFramePoint("initialFootPlacement", worldFrame, registry);
   private final YoFramePoint currentFootPlacement = new YoFramePoint("currentFootPlacement", worldFrame, registry);
   private final EnumYoVariable<RobotSide> currentTransferToSide = new EnumYoVariable<RobotSide>("currentTransferToSide", registry, RobotSide.class);
   
   private final DoubleYoVariable percentIn = new DoubleYoVariable("percentIn", registry);
   
   private final FramePoint footLocationTemp = new FramePoint();
   private final CommonWalkingReferenceFrames referenceFrames;
   
   private final DoubleYoVariable alphaDeltaFootPosition = new DoubleYoVariable("alphaDeltaFootPosition", registry);
   private final AlphaFilteredYoFrameVector deltaFootPosition = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("deltaFootPosition", "", registry, alphaDeltaFootPosition, worldFrame);
   private final Vector3d deltaVectorTemp = new Vector3d();
   
   public SmoothICPComputer2D(CommonWalkingReferenceFrames referenceFrames, double dt, double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps, YoVariableRegistry parentRegistry,
                              DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(dt, doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps, parentRegistry, dynamicGraphicObjectsListRegistry);
      
      alphaDeltaFootPosition.set(0.65);
      this.referenceFrames = referenceFrames;
   }

   private final Point3d icpPostionToPackTemp = new Point3d();
   private final Vector3d icpVelocityToPackTemp = new Vector3d();
   private final Vector3d icpAccelerationToPackTemp = new Vector3d();
   private final Point3d ecmpToPackTemp = new Point3d();

   public void getICPPositionAndVelocity(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, FramePoint2d actualICP, double time)
   {
      super.computeICPPositionVelocityAcceleration(icpPostionToPackTemp, icpVelocityToPackTemp, icpAccelerationToPackTemp, ecmpToPackTemp, time);

      RobotSide transferToSide = currentTransferToSide.getEnumValue();

      if ((transferToSide != null) && isPerformingICPDoubleSupport())
      {
         getFootPlacement(transferToSide, currentFootPlacement);
         
         double deltaX = currentFootPlacement.getX() - initialFootPlacement.getX();
         double deltaY = currentFootPlacement.getY() - initialFootPlacement.getY();
         deltaFootPosition.update(deltaX, deltaY, 0.0);
         
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
      initializeDoubleSupportInitialTransfer(transferToAndNextFootstepsData, initialICPPositionTemp,
            initialTime);
   }
   

   public FramePoint2d getFinalDesiredICP()
   {
      Point3d upcomingCornerPoint = super.getUpcomingCornerPoint();
      
      //TODO: Need to use Frames throughout here, or don't assume this!
      FramePoint2d ret = new FramePoint2d(ReferenceFrame.getWorldFrame(), upcomingCornerPoint.getX(), upcomingCornerPoint.getY());
      return ret;
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
         footLocationTemp.setToZero(referenceFrames.getAnkleZUpFrame(transferToSide));
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


}
