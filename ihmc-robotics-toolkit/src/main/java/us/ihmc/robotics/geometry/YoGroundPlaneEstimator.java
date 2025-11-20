package us.ihmc.robotics.geometry;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPolygonExtruded3DDefinition;

public class YoGroundPlaneEstimator extends GroundPlaneEstimator implements SCS2YoGraphicHolder
{
   private final YoFramePoint3D yoGroundPlanePoint;
   private final YoFrameVector3D yoGroundPlaneNormal;
   private final YoFrameQuaternion yoGroundPlaneOrientation;

   public YoGroundPlaneEstimator(YoRegistry parentRegistry)
   {
      this("", parentRegistry);
   }

   public YoGroundPlaneEstimator(String prefix, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(prefix + getClass().getSimpleName());

      yoGroundPlanePoint = new YoFramePoint3D(prefix + "GroundPlanePointInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoGroundPlaneNormal = new YoFrameVector3D(prefix + "GroundPlaneNormalInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoGroundPlaneOrientation = new YoFrameQuaternion(prefix + "GroundPlaneOrientationInWorld", ReferenceFrame.getWorldFrame(), registry);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
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
   }

   public void hideGraphics()
   {
      yoGroundPlaneNormal.setToNaN();
      yoGroundPlanePoint.setToNaN();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition graphicGroupDefinition = new YoGraphicGroupDefinition(getClass().getSimpleName());
      List<Point2D> vertices = new ArrayList<>();
      vertices.add(new Point2D(2.0, 2.0));
      vertices.add(new Point2D(2.0, -2.0));
      vertices.add(new Point2D(-2.0, -2.0));
      vertices.add(new Point2D(-2.0, 2.0));
      ColorDefinition color = ColorDefinitions.SkyBlue();
      color.setAlpha(0.5);
      graphicGroupDefinition.addChild(newYoGraphicPolygonExtruded3DDefinition("groundPlaneEstimateVisualizer",
                                                                              new YoFramePose3D(yoGroundPlanePoint, yoGroundPlaneOrientation),
                                                                              vertices,
                                                                              0.01, color));
      return null;
   }
}
