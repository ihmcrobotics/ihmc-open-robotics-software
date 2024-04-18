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
   private final Pose3D openingMechanismPose = new Pose3D();

   private final PlanarRegion doorPlanarRegion = new PlanarRegion();
   private long doorPlanarRegionUpdateTimeMillis;

   public DoorNode(long id, String name)
   {
      this(id, name, OpeningMechanismType.UNKNOWN, new Pose3D(), new PlanarRegion());
   }

   public DoorNode(long id, String name, OpeningMechanismType openingMechanismType, Pose3D objectPose, PlanarRegion doorPlanarRegion)
   {
      super(id, name);
      this.openingMechanismType = openingMechanismType;
      this.openingMechanismPose.set(objectPose);
      this.doorPlanarRegion.set(doorPlanarRegion);
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

   public Pose3D getOpeningMechanismPose()
   {
      return openingMechanismPose;
   }

   public void setOpeningMechanismPose(Pose3D pose)
   {
      this.openingMechanismPose.set(pose);
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

      Point3D openingMechanismCentroidInWorld = new Point3D(getOpeningMechanismPose().getTranslation());

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

            if (planarRegionCentroidInWorld.distance(openingMechanismCentroidInWorld) > epsilon)
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
      Point2D openingMechanismPointInWorld2D = new Point2D(getOpeningMechanismPose().getTranslation());
      RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(openingMechanismPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;
      double yaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());
      double pitch = 0.0;
      double roll = 0.0;
      if (openingMechanismType == OpeningMechanismType.LEVER_HANDLE)
         roll = doorSide == RobotSide.LEFT ? Math.PI : 0.0;
      openingMechanismPose.getRotation().setYawPitchRoll(yaw, pitch, roll);
   }
}
