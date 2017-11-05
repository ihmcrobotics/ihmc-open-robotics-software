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

   public void projectPointOntoControlPlane(FramePoint3D pointToProject, FramePoint3D projectionToPack)
   {
      ReferenceFrame referenceFrame = pointToProject.getReferenceFrame();
      pointToProject.changeFrame(centerOfMassFrame);

      projectPointOntoControlPlane(pointToProject, projectionToPack, controlPlaneHeight.getDoubleValue());

      pointToProject.changeFrame(referenceFrame);
      projectionToPack.changeFrame(referenceFrame);
   }

   public void projectPointFromPlaneOntoSurface(FramePoint3D pointToProject, FramePoint3D projectionToPack, double surfaceHeight)
   {
      ReferenceFrame referenceFrame = pointToProject.getReferenceFrame();
      pointToProject.setZ(surfaceHeight);
      pointToProject.changeFrame(centerOfMassFrame);

      double surfaceHeightInCoMFrame = pointToProject.getZ();
      projectPointFromControlPlaneOntoSurface(pointToProject, projectionToPack, controlPlaneHeight.getDoubleValue(), surfaceHeightInCoMFrame);

      pointToProject.changeFrame(referenceFrame);
      projectionToPack.changeFrame(referenceFrame);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   public void projectPointFromPlaneOntoSurface(FramePoint2D pointToProject, FramePoint2D projectionToPack, double surfaceHeightInWorld)
   {
      ReferenceFrame referenceFrame = pointToProject.getReferenceFrame();
      pointToProject.changeFrame(ReferenceFrame.getWorldFrame());

      tempPoint.setIncludingFrame(pointToProject, surfaceHeightInWorld);
      tempPoint.changeFrame(centerOfMassFrame);

      double surfaceHeightInCoMFrame = tempPoint.getZ();
      projectPointFromControlPlaneOntoSurface(tempPoint, projectionToPack, controlPlaneHeight.getDoubleValue(), surfaceHeightInCoMFrame);

      projectionToPack.changeFrame(referenceFrame);
      pointToProject.changeFrame(referenceFrame);
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

   private static void projectPointFromControlPlaneOntoSurface(FramePoint3D pointToProject, FramePoint2D projectionToPack, double planeHeight, double surfaceHeight)
   {
      projectPoint(pointToProject, projectionToPack, surfaceHeight, planeHeight);
   }

   private static void projectPointFromControlPlaneOntoSurface(FramePoint2D pointToProject, FramePoint2D projectionToPack, double planeHeight, double surfaceHeight)
   {
      projectPoint(pointToProject, projectionToPack, surfaceHeight, planeHeight);
   }

   private static void projectPoint(FramePoint3D pointToProject, FramePoint3D projectionToPack, double projectedHeight, double unprojectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.scale(projectedHeight / unprojectedHeight);
      projectionToPack.setZ(projectedHeight);
   }

   private static void projectPoint(FramePoint2D pointToProject, FramePoint2D projectionToPack, double projectedHeight, double unprojectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.scale(projectedHeight / unprojectedHeight);
   }

   private static void projectPoint(FramePoint3D pointToProject, FramePoint2D projectionToPack, double projectedHeight, double unprojectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.scale(projectedHeight / unprojectedHeight);
   }
}
