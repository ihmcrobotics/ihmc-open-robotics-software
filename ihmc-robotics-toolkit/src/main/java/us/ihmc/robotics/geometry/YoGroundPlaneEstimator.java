package us.ihmc.robotics.geometry;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
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
   private  YoGraphicPolygon groundPlaneVisualizer;

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
         YoFrameConvexPolygon2D yoGroundPlaneEstimate = new YoFrameConvexPolygon2D("groundPlaneEstimate", "", ReferenceFrame.getWorldFrame(), 4, parentRegistry);
         yoGroundPlaneEstimate.addVertex(2.0, 2.0);
         yoGroundPlaneEstimate.addVertex(2.0, -2.0);
         yoGroundPlaneEstimate.addVertex(-2.0, -2.0);
         yoGroundPlaneEstimate.addVertex(-2.0, 2.0);

         groundPlaneVisualizer = new YoGraphicPolygon("groundPlaneEstimateVisualizer", yoGroundPlaneEstimate,
                                                      yoGroundPlanePoint, yoGroundPlaneOrientation,
                                                      1, YoAppearance.Glass());
         graphicsListRegistry.registerYoGraphic(prefix + "GroundPlaneEstimate", groundPlaneVisualizer);
      }

      hideGraphics();
   }

   /**
    * Estimate the ground plane given the current list of ground contact points.
    */
   @Override
   public void compute()
   {
      compute(0.0);
   }

   /**
    * Estimate the ground plane given the current list of ground contact points, accounting for yaw.
    * @param yaw : yaw of ground plane relative to world
    */
   @Override
   public void compute(double yaw)
   {
      super.compute(yaw);

      yoGroundPlaneNormal.set(getPlaneNormal());
      yoGroundPlanePoint.set(getPlanePoint());

      yoGroundPlaneOrientation.setYawPitchRoll(yaw, getPitch(yaw), getRoll(yaw));

      groundPlaneVisualizer.update();
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : list of ground contact points
    */
   @Override
   public void compute(List<? extends FramePoint3DReadOnly> contactPoints)
   {
      compute(contactPoints, 0.0);
   }

   /**
    * Set the list of ground contact points and compute the ground plane, accounting for yaw.
    * @param contactPoints : list of ground contact points
    * @param yaw : yaw of ground plane relative to world
    */
   @Override
   public void compute(List<? extends FramePoint3DReadOnly> contactPoints, double yaw)
   {
      super.compute(contactPoints, yaw);

      yoGroundPlaneNormal.set(getPlaneNormal());
      yoGroundPlanePoint.set(getPlanePoint());

      yoGroundPlaneOrientation.setYawPitchRoll(yaw, getPitch(yaw), getRoll(yaw));

      groundPlaneVisualizer.update();
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : quadrant dependent list of contact points
    */
   @Override
   public void compute(QuadrantDependentList<? extends FramePoint3DReadOnly> contactPoints)
   {
      compute(contactPoints, 0.0);
   }

   /**
    * Set the list of ground contact points and compute the ground plane, accounting for yaw.
    * @param contactPoints : quadrant dependent list of contact points
    * @param yaw : yaw of ground plane relative to world
    */
   @Override
   public void compute(QuadrantDependentList<? extends FramePoint3DReadOnly> contactPoints, double yaw)
   {
      super.compute(contactPoints, yaw);

      yoGroundPlaneNormal.set(getPlaneNormal());
      yoGroundPlanePoint.set(getPlanePoint());

      yoGroundPlaneOrientation.setYawPitchRoll(yaw, getPitch(yaw), getRoll(yaw));

      groundPlaneVisualizer.update();
   }

   public void hideGraphics()
   {
      yoGroundPlaneNormal.setToNaN();
      yoGroundPlanePoint.setToNaN();
   }
}
