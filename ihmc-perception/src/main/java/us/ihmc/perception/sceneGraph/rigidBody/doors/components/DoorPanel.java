package us.ihmc.perception.sceneGraph.rigidBody.doors.components;

import perception_msgs.msg.dds.DoorPanelMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorModelParameters;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class DoorPanel
{
   private transient final DoorNode doorNode;

   private final PlanarRegion planarRegion = new PlanarRegion();
   private long planarRegionLastUpdateTimeMillis;

   public DoorPanel(DoorNode doorNode)
   {
      this.doorNode = doorNode;
   }

   public PlanarRegion getPlanarRegion()
   {
      return planarRegion;
   }

   public void setPlanarRegion(PlanarRegion planarRegion)
   {
      this.planarRegion.set(planarRegion);
   }

   public long getPlanarRegionLastUpdateTimeMillis()
   {
      return planarRegionLastUpdateTimeMillis;
   }

   public void setPlanarRegionLastUpdateTimeMillis(long planarRegionLastUpdateTimeMillis)
   {
      this.planarRegionLastUpdateTimeMillis = planarRegionLastUpdateTimeMillis;
   }

   public void toMessage(DoorPanelMessage message)
   {
      message.getPlanarRegion().set(PlanarRegionMessageConverter.convertToPlanarRegionMessage(planarRegion));
      message.setPlanarRegionLastUpdateTimeMillis(planarRegionLastUpdateTimeMillis);
   }

   public void fromMessage(DoorPanelMessage message)
   {
      planarRegion.set(PlanarRegionMessageConverter.convertToPlanarRegion(message.getPlanarRegion()));
      planarRegionLastUpdateTimeMillis = message.getPlanarRegionLastUpdateTimeMillis();
   }

   public void filterAndSetPlanarRegionFromPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      // Check if the current door planar region is old
      if (System.currentTimeMillis() - planarRegionLastUpdateTimeMillis > 1000)
      {
         setPlanarRegion(new PlanarRegion());
      }

      DoorOpeningMechanism doorOpeningMechanism = doorNode.getLatestUpdatedOpeningMechanism();
      if (doorOpeningMechanism == null)
         return; // Early return
      Point3D doorPointInWorld = new Point3D(doorOpeningMechanism.getGraspPose().getTranslation());

      if (!planarRegionsList.isEmpty())
      {
         float epsilon = 0.75f;

         // TODO: fixme doesn't work
         //            PlanarRegion doorPlanarRegion = planarRegionsList.findClosestPlanarRegionToPointByProjectionOntoXYPlane(doorLeverPointInWorld.getX(),
         //                                                                                                                    doorLeverPointInWorld.getY());

         PlanarRegion candidatePlanarRegion = null;

         for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         {
            Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(planarRegion);
            double distance = planarRegionCentroidInWorld.distance(doorPointInWorld);

            System.out.println(doorPointInWorld);
            System.out.println(distance);

            if (distance > epsilon)
               continue;

            // If the planar region is less than 1/5th the area of a door
            if (planarRegion.getArea() < ((DoorModelParameters.DOOR_PANEL_HEIGHT * DoorModelParameters.DOOR_PANEL_WIDTH) / 5))
               continue;

            if (candidatePlanarRegion == null)
            {
               candidatePlanarRegion = planarRegion;
               continue;
            }

            if (planarRegion.getArea() > candidatePlanarRegion.getArea())
               candidatePlanarRegion = planarRegion;
         }

         if (candidatePlanarRegion != null)
         {
            setPlanarRegion(candidatePlanarRegion);
            setPlanarRegionLastUpdateTimeMillis(System.currentTimeMillis());
         }
      }
   }
}
