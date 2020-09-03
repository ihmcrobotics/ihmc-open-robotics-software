package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PolygonSnapperVisualizer
{
   private final SimulationConstructionSet scs;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoFrameConvexPolygon2D polygonToSnap, snappedPolygon;
   private final YoFramePoseUsingYawPitchRoll polygonToSnapPose, snappedPolygonPose;
   private final YoGraphicPolygon polygonToSnapViz, snappedPolygonViz;

   public PolygonSnapperVisualizer(ConvexPolygon2D snappingPolygonShape)
   {
      Robot robot = new Robot("Robot");
      scs = new SimulationConstructionSet(robot);
      scs.setDT(0.1, 1);

      polygonToSnap = new YoFrameConvexPolygon2D("polygonToSnap", ReferenceFrame.getWorldFrame(), 4, registry);
      snappedPolygon = new YoFrameConvexPolygon2D("snappedPolygon", ReferenceFrame.getWorldFrame(), 4, registry);

      polygonToSnap.set(snappingPolygonShape);
      snappedPolygon.set(snappingPolygonShape);

      polygonToSnapPose = new YoFramePoseUsingYawPitchRoll("polygonToSnapPose", ReferenceFrame.getWorldFrame(), registry);
      snappedPolygonPose = new YoFramePoseUsingYawPitchRoll("snappedPolygonPose", ReferenceFrame.getWorldFrame(), registry);

      polygonToSnapPose.setToNaN();
      snappedPolygonPose.setToNaN();

      polygonToSnapViz = new YoGraphicPolygon("polygonToSnapViz", polygonToSnap, polygonToSnapPose, 1.0, YoAppearance.Green());
      snappedPolygonViz = new YoGraphicPolygon("snappedPolygonViz", polygonToSnap, snappedPolygonPose, 1.0, YoAppearance.Red());

      polygonToSnapViz.update();
      snappedPolygonViz.update();

      scs.addYoGraphic(polygonToSnapViz);
      scs.addYoGraphic(snappedPolygonViz);

      scs.addYoRegistry(registry);
      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   public void addPlanarRegionsList(PlanarRegionsList planarRegions, AppearanceDefinition... appearances)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, planarRegions, appearances);
      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   public void addPolygon(RigidBodyTransform transform, ConvexPolygon2D polygon, AppearanceDefinition appearance)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.transform(transform);
      graphics3DObject.addPolygon(polygon, appearance);
      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   public void setSnappedPolygon(RigidBodyTransform nonSnappedTransform, RigidBodyTransform snapTransform)
   {
      setSnappedPolygon(nonSnappedTransform, snapTransform, null);
   }

   public void setSnappedPolygon(RigidBodyTransform nonSnappedTransform, RigidBodyTransform snapTransform, ConvexPolygon2D partialFootholdPolygon)
   {
      Point3D nonSnappedPosition = new Point3D();
      Quaternion nonSnappedOrientation = new Quaternion();
      nonSnappedTransform.get(nonSnappedOrientation, nonSnappedPosition);

      Point3D snappedPosition = new Point3D();
      Quaternion snappedOrientation = new Quaternion();

      if (snapTransform != null)
      {
         RigidBodyTransform combinedTransform = new RigidBodyTransform(snapTransform);
         combinedTransform.multiply(nonSnappedTransform);
         combinedTransform.get(snappedOrientation, snappedPosition);

         nonSnappedPosition.setZ(snappedPosition.getZ() + 0.15);
      }

      polygonToSnapPose.setPosition(nonSnappedPosition);
      polygonToSnapPose.setOrientation(nonSnappedOrientation);
      polygonToSnapViz.update();

      if (snapTransform == null)
      {
         snappedPolygonPose.setToNaN();
         snappedPolygonViz.update();
         return;
      }

      snappedPolygonPose.setPosition(snappedPosition);
      snappedPolygonPose.setOrientation(snappedOrientation);
      snappedPolygonViz.update();

      if(partialFootholdPolygon != null)
      {
         snappedPolygonViz.updateConvexPolygon2d(partialFootholdPolygon);
      }
      else
      {
         snappedPolygonViz.updateConvexPolygon2d(snappedPolygon);
      }

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   public void cropBuffer()
   {
      scs.cropBuffer();
   }

}
