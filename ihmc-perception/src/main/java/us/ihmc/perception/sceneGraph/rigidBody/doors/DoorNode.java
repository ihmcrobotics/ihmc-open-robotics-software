package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class DoorNode extends SceneNode
{
   private OpeningMechanismType openingMechanismType;
   private final Pose3D doorHardwarePose = new Pose3D();
   private final RigidBodyTransform doorHardwareVisualTransformToObjectPose = new RigidBodyTransform();

   private final PlanarRegion doorPlanarRegion = new PlanarRegion();

   public DoorNode(long id, String name)
   {
      this(id, name, OpeningMechanismType.UNKNOWN, new Pose3D(), new RigidBodyTransform(), new PlanarRegion());
   }

   public DoorNode(long id,
                   String name,
                   OpeningMechanismType openingMechanismType,
                   Pose3D objectPose,
                   RigidBodyTransformBasics visualTransformToObjectPose,
                   PlanarRegion doorPlanarRegion)
   {
      super(id, name);
      this.openingMechanismType = openingMechanismType;
      this.doorHardwarePose.set(objectPose);
      this.doorHardwareVisualTransformToObjectPose.set(visualTransformToObjectPose);
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

   public Pose3D getDoorHardwarePose()
   {
      return doorHardwarePose;
   }

   public void setDoorHardwarePose(Pose3D pose)
   {
      this.doorHardwarePose.set(pose);
   }

   public RigidBodyTransform getDoorHardwareVisualTransformToObjectPose()
   {
      return doorHardwareVisualTransformToObjectPose;
   }

   public void setDoorHardwareVisualTransformToObjectPose(RigidBodyTransformBasics transform)
   {
      doorHardwareVisualTransformToObjectPose.set(transform);
   }

   public PlanarRegion getDoorPlanarRegion()
   {
      return doorPlanarRegion;
   }

   public void setDoorPlanarRegion(PlanarRegion planarRegion)
   {
      this.doorPlanarRegion.set(planarRegion);
   }

   public void filterAndSetDoorPlanarRegionFromPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      Point3D doorHardwareCentroidInWorld = new Point3D(getDoorHardwarePose().getTranslation());

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

            if (planarRegionCentroidInWorld.distance(doorHardwareCentroidInWorld) > epsilon)
               continue;

            // If the planar region is less than 1/4th the area of a door
            if (planarRegion.getArea() < ((DoorModelParameters.DOOR_PANEL_HEIGHT * DoorModelParameters.DOOR_PANEL_WIDTH) / 4))
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
            setDoorPlanarRegion(doorPlanarRegion);
      }
   }
}
