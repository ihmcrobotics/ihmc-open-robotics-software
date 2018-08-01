package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class YoGroundPlaneEstimator extends GroundPlaneEstimator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint3D yoGroundPlanePoint = new YoFramePoint3D("groundPlanePoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoGroundPlaneNormal = new YoFrameVector3D("groundPlaneNormal", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll yoGroundPlaneOrientation = new YoFrameYawPitchRoll("groundPlaneOrientation", ReferenceFrame.getWorldFrame(), registry);

   public YoGroundPlaneEstimator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(parentRegistry, graphicsListRegistry, YoAppearance.Glass());
   }

   public YoGroundPlaneEstimator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, AppearanceDefinition groundPlaneAppearance)
   {
      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (graphicsListRegistry != null)
      {
         Graphics3DObject groundPlaneGraphic = new Graphics3DObject();
         groundPlaneGraphic.addCylinder(0.005, 0.5, groundPlaneAppearance);
         YoGraphicShape yoGroundPlaneGraphic = new YoGraphicShape("groundPlaneEstimate", groundPlaneGraphic, yoGroundPlanePoint, yoGroundPlaneOrientation, 1.0);
         graphicsListRegistry.registerYoGraphic("groundPlaneEstimate", yoGroundPlaneGraphic);
      }
   }

   @Override
   public void compute()
   {
      super.compute();

      yoGroundPlaneNormal.set(groundPlaneFrameNormal);

      yoGroundPlanePoint.set(groundPlaneFramePoint);
      yoGroundPlaneOrientation.setYawPitchRoll(0.0, getPitch(), getRoll());
   }
}
