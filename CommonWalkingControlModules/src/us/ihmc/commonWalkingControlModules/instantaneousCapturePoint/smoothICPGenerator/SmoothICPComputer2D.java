package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.InstantaneousCapturePointPlanner;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

//public class SmoothICPComputer2D extends SmoothICPComputer implements InstantaneousCapturePointPlanner
public class SmoothICPComputer2D extends DoubleSupportFootCenterToToeICPComputer implements InstantaneousCapturePointPlanner
{
   public SmoothICPComputer2D(double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps, YoVariableRegistry parentRegistry,
                              DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps, parentRegistry, dynamicGraphicObjectsListRegistry);
   }

   private Point3d icpPostionToPackTemp = new Point3d();
   private Vector3d icpVelocityToPackTemp = new Vector3d();
   private Vector3d icpAccelerationToPackTemp = new Vector3d();
   private Point3d ecmpToPackTemp = new Point3d();

   public void getICPPositionAndVelocity(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, FramePoint2d actualICP, double time)
   {
      super.computeICPPositionVelocityAcceleration(icpPostionToPackTemp, icpVelocityToPackTemp, icpAccelerationToPackTemp, ecmpToPackTemp, time);

      icpPostionToPack.set(icpPostionToPackTemp.getX(), icpPostionToPackTemp.getY());
      icpVelocityToPack.set(icpVelocityToPackTemp.getX(), icpVelocityToPackTemp.getY());
      ecmpToPack.set(ecmpToPackTemp.getX(), ecmpToPackTemp.getY());
   }
   
   private final Point3d initialICPPositionTemp = new Point3d();
   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point2d initialICPPosition,
         double initialTime)
   {
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


}
