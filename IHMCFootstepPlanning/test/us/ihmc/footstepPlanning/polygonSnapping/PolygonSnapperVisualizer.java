package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PolygonSnapperVisualizer
{
   private final SimulationConstructionSet scs;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameConvexPolygon2d polygonToSnap, snappedPolygon;
   private final YoFramePose polygonToSnapPose, snappedPolygonPose;
   private final YoGraphicPolygon polygonToSnapViz, snappedPolygonViz;

   public PolygonSnapperVisualizer(ConvexPolygon2d snappingPolygonShape)
   {
      Robot robot = new Robot("Robot");
      scs = new SimulationConstructionSet(robot);
      scs.setDT(0.1, 1);

      polygonToSnap = new YoFrameConvexPolygon2d("polygonToSnap", ReferenceFrame.getWorldFrame(), 4, registry);
      snappedPolygon = new YoFrameConvexPolygon2d("snappedPolygon", ReferenceFrame.getWorldFrame(), 4, registry);

      polygonToSnap.setConvexPolygon2d(snappingPolygonShape);
      snappedPolygon.setConvexPolygon2d(snappingPolygonShape);

      polygonToSnapPose = new YoFramePose("polygonToSnapPose", ReferenceFrame.getWorldFrame(), registry);
      snappedPolygonPose = new YoFramePose("snappedPolygonPose", ReferenceFrame.getWorldFrame(), registry);

      polygonToSnapPose.setToNaN();
      snappedPolygonPose.setToNaN();

      polygonToSnapViz = new YoGraphicPolygon("polygonToSnapViz", polygonToSnap, polygonToSnapPose, 1.0, YoAppearance.Green());
      snappedPolygonViz = new YoGraphicPolygon("snappedPolygonViz", polygonToSnap, snappedPolygonPose, 1.0, YoAppearance.Red());

      polygonToSnapViz.update();
      snappedPolygonViz.update();

      scs.addYoGraphic(polygonToSnapViz);
      scs.addYoGraphic(snappedPolygonViz);

      scs.addYoVariableRegistry(registry);
      scs.startOnAThread();
   }

   public void addPlanarRegionsList(PlanarRegionsList planarRegions, AppearanceDefinition... appearances)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addPlanarRegionsList(planarRegions, appearances);
      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   public void addPolygon(RigidBodyTransform transform, ConvexPolygon2d polygon, AppearanceDefinition appearance)
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

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   public void cropBuffer()
   {
      scs.cropBuffer();
   }

}
