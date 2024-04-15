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
   private DoorHardwareType doorHardwareType;
   private final PlanarRegion doorPlanarRegion = new PlanarRegion();
   private final Pose3D objectPose = new Pose3D();
   private final RigidBodyTransform visualTransformToObjectPose = new RigidBodyTransform();

   public DoorNode(long id, String name)
   {
      this(id,
           name,
           DoorHardwareType.UNKNOWN,
           new PlanarRegion(),
           new Pose3D(),
           new RigidBodyTransform());
   }

   public DoorNode(long id,
                   String name,
                   DoorHardwareType doorHardwareType,
                   PlanarRegion doorPlanarRegion,
                   Pose3D objectPose,
                   RigidBodyTransformBasics visualTransformToObjectPose)
   {
      super(id, name);
      this.doorHardwareType = doorHardwareType;
      this.doorPlanarRegion.set(doorPlanarRegion);
      this.objectPose.set(objectPose);
      this.visualTransformToObjectPose.set(visualTransformToObjectPose);
   }

   public DoorHardwareType getDoorHardwareType()
   {
      return doorHardwareType;
   }

   public void setDoorHardwareType(DoorHardwareType doorHardwareType)
   {
      this.doorHardwareType = doorHardwareType;
   }

   public void setDoorHardwareTypeFromYoloClass(YOLOv8DetectionClass yoloClass)
   {
      switch (yoloClass)
      {
         case DOOR_LEVER -> setDoorHardwareType(DoorHardwareType.LEVER_HANDLE);
         case DOOR_KNOB -> setDoorHardwareType(DoorHardwareType.KNOB);
         case DOOR_PULL_HANDLE -> setDoorHardwareType(DoorHardwareType.PULL_HANDLE);
         case DOOR_PUSH_BAR -> setDoorHardwareType(DoorHardwareType.PUSH_BAR);
      }
   }

   public PlanarRegion getDoorPlanarRegion()
   {
      return doorPlanarRegion;
   }

   public void setDoorPlanarRegion(PlanarRegion planarRegion)
   {
      this.doorPlanarRegion.set(planarRegion);
   }

   public Pose3D getObjectPose()
   {
      return objectPose;
   }

   public void setObjectPose(Pose3D objectPose)
   {
      this.objectPose.set(objectPose);
   }

   public RigidBodyTransform getVisualTransformToObjectPose()
   {
      return visualTransformToObjectPose;
   }

   public void setVisualTransformToObjectPose(RigidBodyTransformBasics transform)
   {
      visualTransformToObjectPose.set(transform);
   }

   public void updatePlanarRegions(PlanarRegionsList planarRegionsList)
   {
      Point3D objectCentroidInWorld = new Point3D(getObjectPose().getTranslation());

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

            if (planarRegionCentroidInWorld.distance(objectCentroidInWorld) > epsilon)
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
            this.doorPlanarRegion.set(doorPlanarRegion);
      }
   }
}
