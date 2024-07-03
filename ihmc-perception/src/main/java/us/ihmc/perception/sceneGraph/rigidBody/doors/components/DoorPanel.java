package us.ihmc.perception.sceneGraph.rigidBody.doors.components;

import perception_msgs.msg.dds.DoorPanelMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorModelParameters;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNodeTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Collection;
import java.util.UUID;

public class DoorPanel
{
   private transient final DoorNode doorNode;

   private PersistentDetection doorPanelDetection = null;
   private UUID detectionID;

   private final PlanarRegion planarRegion = new PlanarRegion();
   private long planarRegionLastUpdateTimeMillis;

   public DoorPanel(DoorNode doorNode)
   {
      this.doorNode = doorNode;
   }

   /**
    * Accepts a door panel detection if it matches the acceptance criteria
    * @param newPanelDetection {@link PersistentDetection} of the door panel.
    * @return true if the detection is accepted, false otherwise.
    */
   public boolean acceptDetection(PersistentDetection newPanelDetection, Collection<DoorOpeningMechanism> presentOpeningMechanisms)
   {
      if (this.doorPanelDetection == null && DoorNodeTools.detectionIsDoorPanel(newPanelDetection))
      {
         if (presentOpeningMechanisms.isEmpty()
             || presentOpeningMechanisms.stream()
                                        .anyMatch(openingMechanism -> openingMechanism.getDetection()
                                                                                      .getMostRecentPosition()
                                                                                      .distanceSquared(newPanelDetection.getMostRecentPosition())
                                                                      < DoorNode.DOOR_COMPONENT_DISTANCE_THRESHOLD))
         {
            this.doorPanelDetection = newPanelDetection;
            detectionID = doorPanelDetection.getID();
            return true;
         }
      }

      return false;
   }

   public PersistentDetection getDoorPanelDetection()
   {
      return doorPanelDetection;
   }

   public InstantDetection getLatestDetection()
   {
      if (doorPanelDetection == null)
         return null;

      return doorPanelDetection.getMostRecentDetection();
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
      MessageTools.toMessage(detectionID != null ? detectionID : PersistentDetection.NULL_DETECTION_ID, message.getPersistentDetectionId());
   }

   public void fromMessage(DoorPanelMessage message)
   {
      planarRegion.set(PlanarRegionMessageConverter.convertToPlanarRegion(message.getPlanarRegion()));
      planarRegionLastUpdateTimeMillis = message.getPlanarRegionLastUpdateTimeMillis();
      detectionID = MessageTools.toUUID(message.getPersistentDetectionId());
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
      Point3D doorPointInWorld = new Point3D(doorOpeningMechanism.getMechanismPose().getTranslation());

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
