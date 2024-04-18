package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

public class DoorNode extends SceneNode
{
   private OpeningMechanismType openingMechanismType;
   private final Point3D openingMechanismPoint3D = new Point3D();
   private final Pose3D openingMechanismPose3D = new Pose3D();

   private final PlanarRegion doorPlanarRegion = new PlanarRegion();
   private long doorPlanarRegionUpdateTimeMillis;

   public DoorNode(long id, String name)
   {
      super(id, name);
   }

   public OpeningMechanismType getOpeningMechanismType()
   {
      return openingMechanismType;
   }

   public void setOpeningMechanismType(OpeningMechanismType openingMechanismType)
   {
      this.openingMechanismType = openingMechanismType;
   }

   public void setOpeningMechanismTypeFromYoloClass(YOLOv8DetectionClass yoloClass)
   {
      switch (yoloClass)
      {
         case DOOR_LEVER -> setOpeningMechanismType(OpeningMechanismType.LEVER_HANDLE);
         case DOOR_KNOB -> setOpeningMechanismType(OpeningMechanismType.KNOB);
         case DOOR_PULL_HANDLE -> setOpeningMechanismType(OpeningMechanismType.PULL_HANDLE);
         case DOOR_PUSH_BAR -> setOpeningMechanismType(OpeningMechanismType.PUSH_BAR);
      }
   }

   public Point3D getOpeningMechanismPoint3D()
   {
      return openingMechanismPoint3D;
   }

   public void setOpeningMechanismPoint3D(Point3D point3D)
   {
      this.openingMechanismPoint3D.set(point3D);
   }

   public Pose3D getOpeningMechanismPose3D()
   {
      return openingMechanismPose3D;
   }

   public void setOpeningMechanismPose3D(Pose3D pose3D)
   {
      this.openingMechanismPose3D.set(pose3D);
   }

   public PlanarRegion getDoorPlanarRegion()
   {
      return doorPlanarRegion;
   }

   public void setDoorPlanarRegion(PlanarRegion planarRegion)
   {
      this.doorPlanarRegion.set(planarRegion);
   }

   public long getDoorPlanarRegionUpdateTime()
   {
      return doorPlanarRegionUpdateTimeMillis;
   }

   public void setDoorPlanarRegionUpdateTime(long doorPlanarRegionUpdateTimeMillis)
   {
      this.doorPlanarRegionUpdateTimeMillis = doorPlanarRegionUpdateTimeMillis;
   }

   public void filterAndSetDoorPlanarRegionFromPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      // Check if the current door planar region is old
      if (System.currentTimeMillis() - doorPlanarRegionUpdateTimeMillis > 2000)
      {
         setDoorPlanarRegion(new PlanarRegion());
      }

      if (!planarRegionsList.isEmpty())
      {
         float epsilon = 0.75f;

         // TODO: fixme doesn't work
         //            PlanarRegion doorPlanarRegion = planarRegionsList.findClosestPlanarRegionToPointByProjectionOntoXYPlane(doorLeverPointInWorld.getX(),
         //                                                                                                                    doorLeverPointInWorld.getY());

         PlanarRegion doorPlanarRegion = null;

         for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         {
            Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(planarRegion);

            if (planarRegionCentroidInWorld.distance(openingMechanismPoint3D) > epsilon)
               continue;

            // If the planar region is less than 1/5th the area of a door
            if (planarRegion.getArea() < ((DoorModelParameters.DOOR_PANEL_HEIGHT * DoorModelParameters.DOOR_PANEL_WIDTH) / 5))
               continue;

            if (doorPlanarRegion == null)
            {
               doorPlanarRegion = planarRegion;
               continue;
            }

            if (planarRegion.getArea() > doorPlanarRegion.getArea())
               doorPlanarRegion = planarRegion;
         }

         if (doorPlanarRegion != null)
         {
            setDoorPlanarRegion(doorPlanarRegion);
            setDoorPlanarRegionUpdateTime(System.currentTimeMillis());
         }
      }

      // Calculate yaw, pitch, roll of opening mechanism pose
      Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(doorPlanarRegion);
      Line2D doorLineNormal = new Line2D(planarRegionCentroidInWorld.getX(),
                                         planarRegionCentroidInWorld.getY(),
                                         doorPlanarRegion.getNormalX(),
                                         doorPlanarRegion.getNormalY());
      Point2D openingMechanismPointInWorld2D = new Point2D(openingMechanismPoint3D);
      RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(openingMechanismPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;
      double yaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());
      double pitch = 0.0;
      double roll = 0.0;
      if (openingMechanismType == OpeningMechanismType.LEVER_HANDLE)
         roll = doorSide == RobotSide.LEFT ? Math.PI : 0.0;
      openingMechanismPose3D.getTranslation().set(openingMechanismPoint3D);
      openingMechanismPose3D.getRotation().setYawPitchRoll(yaw, pitch, roll);
   }
}
