package us.ihmc.robotics.geometry;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class YoGroundPlaneEstimator extends GroundPlaneEstimator
{
   private final YoFramePoint3D yoGroundPlanePoint;
   private final YoFrameVector3D yoGroundPlaneNormal;
   private final YoFrameYawPitchRoll yoGroundPlaneOrientation;

   public YoGroundPlaneEstimator(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this("", parentRegistry, graphicsListRegistry, YoAppearance.Glass());
   }

   public YoGroundPlaneEstimator(String prefix, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, AppearanceDefinition groundPlaneAppearance)
   {
      YoRegistry registry = new YoRegistry(prefix + getClass().getSimpleName());

      yoGroundPlanePoint = new YoFramePoint3D(prefix + "GroundPlanePointInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoGroundPlaneNormal = new YoFrameVector3D(prefix + "GroundPlaneNormalInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoGroundPlaneOrientation = new YoFrameYawPitchRoll(prefix + "GroundPlaneOrientationInWorld", ReferenceFrame.getWorldFrame(), registry);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (graphicsListRegistry != null)
      {
         Graphics3DObject groundPlaneGraphic = new Graphics3DObject();
         groundPlaneGraphic.addCylinder(0.005, 0.5, groundPlaneAppearance);
         YoGraphicShape yoGroundPlaneGraphic = new YoGraphicShape(prefix + "GroundPlaneEstimate", groundPlaneGraphic, yoGroundPlanePoint, yoGroundPlaneOrientation, 1.0);
         graphicsListRegistry.registerYoGraphic(prefix + "GroundPlaneEstimate", yoGroundPlaneGraphic);
      }
   }

   /**
    * Estimate the ground plane given the current list of ground contact points.
    */
   @Override
   public void compute()
   {
      super.compute();

      yoGroundPlaneNormal.set(getPlaneNormal());
      yoGroundPlanePoint.set(getPlanePoint());

      yoGroundPlaneOrientation.setYawPitchRoll(0.0, getPitch(), getRoll());
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : list of ground contact points
    */
   @Override
   public void compute(List<? extends FramePoint3DBasics> contactPoints)
   {
      super.compute(contactPoints);

      yoGroundPlaneNormal.set(getPlaneNormal());
      yoGroundPlanePoint.set(getPlanePoint());

      yoGroundPlaneOrientation.setYawPitchRoll(0.0, getPitch(), getRoll());
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : quadrant dependent list of contact points
    */
   @Override
   public void compute(QuadrantDependentList<? extends FramePoint3DBasics> contactPoints)
   {
      super.compute(contactPoints);

      yoGroundPlaneNormal.set(getPlaneNormal());
      yoGroundPlanePoint.set(getPlanePoint());

      yoGroundPlaneOrientation.setYawPitchRoll(0.0, getPitch(), getRoll());
   }
}
