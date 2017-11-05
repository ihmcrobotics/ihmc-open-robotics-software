package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class ICPControlPlane
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble controlPlaneHeight;
   private final ReferenceFrame centerOfMassFrame;

   public ICPControlPlane(YoDouble omega0, ReferenceFrame centerOfMassFrame, double gravityZ, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      controlPlaneHeight = new YoDouble("controlPlaneHeight", registry);
      parentRegistry.addChild(registry);

      omega0.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double heightOfPlane = -gravityZ / Math.pow(omega0.getDoubleValue(), 2.0);
            controlPlaneHeight.set(heightOfPlane);
         }
      });
   }

   public double getControlPlaneHeight()
   {
      return controlPlaneHeight.getDoubleValue();
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePoint2D tempPoint2D = new FramePoint2D();

   public void projectPointOntoControlPlane(ReferenceFrame desiredReferenceFrame, FramePoint3D pointToProject, FramePoint3D projectionToPack)
   {
      tempPoint.setIncludingFrame(pointToProject);
      tempPoint.changeFrame(centerOfMassFrame);

      projectPointOntoControlPlane(tempPoint, projectionToPack, controlPlaneHeight.getDoubleValue());

      pointToProject.changeFrame(desiredReferenceFrame);
      projectionToPack.changeFrame(desiredReferenceFrame);
   }


   public void projectPointFromPlaneOntoSurface(ReferenceFrame desiredReferenceFrame, FramePoint2D pointToProject, FramePoint3D projectionToPack, double surfaceHeightInWorld)
   {
      tempPoint2D.set(pointToProject);
      tempPoint2D.changeFrame(ReferenceFrame.getWorldFrame());

      tempPoint.setIncludingFrame(tempPoint2D, surfaceHeightInWorld);
      tempPoint.changeFrame(centerOfMassFrame);

      double surfaceHeightInCoMFrame = tempPoint.getZ();
      projectPointFromControlPlaneOntoSurface(tempPoint, projectionToPack, controlPlaneHeight.getDoubleValue(), surfaceHeightInCoMFrame);

      projectionToPack.changeFrame(desiredReferenceFrame);
      pointToProject.changeFrame(desiredReferenceFrame);
   }

   private static void projectPointOntoControlPlane(FramePoint3D pointToProject, FramePoint3D projectionToPack, double planeHeight)
   {
      double unprojectedHeight = pointToProject.getZ();
      projectPoint(pointToProject, projectionToPack, planeHeight, unprojectedHeight);
   }

   private static void projectPointFromControlPlaneOntoSurface(FramePoint3D pointToProject, FramePoint3D projectionToPack, double planeHeight, double surfaceHeight)
   {
      projectPoint(pointToProject, projectionToPack, surfaceHeight, planeHeight);
   }

   private static void projectPoint(FramePoint3D pointToProject, FramePoint3D projectionToPack, double projectedHeight, double unprojectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.scale(projectedHeight / unprojectedHeight);
      projectionToPack.setZ(projectedHeight);
   }
}
