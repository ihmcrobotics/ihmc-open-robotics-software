package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.InstantaneousCapturePointPlanner;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class SmoothICPComputer2D extends SmoothICPComputer implements InstantaneousCapturePointPlanner
{
   public SmoothICPComputer2D(double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps, YoVariableRegistry parentRegistry,
                              DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(doubleSupportFirstStepFraction, maxNumberOfConsideredFootsteps, parentRegistry, dynamicGraphicObjectsListRegistry);
   }

   private Point3d icpPostionToPackTemp = new Point3d();
   private Vector3d icpVelocityToPackTemp = new Vector3d();
   private Point3d ecmpToPackTemp = new Point3d();

   public void getICPPositionAndVelocity(FramePoint2d icpPostionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, double time)
   {
      super.getICPPositionAndVelocity(icpPostionToPackTemp, icpVelocityToPackTemp, ecmpToPackTemp, time);

      icpPostionToPack.set(icpPostionToPackTemp.getX(), icpPostionToPackTemp.getY());
      icpVelocityToPack.set(icpVelocityToPackTemp.getX(), icpVelocityToPackTemp.getY());
      ecmpToPack.set(ecmpToPackTemp.getX(), ecmpToPackTemp.getY());
   }
}
