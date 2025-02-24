package us.ihmc.perception.detections.doors;

import perception_msgs.msg.dds.DetectedDoorMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorModelParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.time.Instant;
import java.util.Comparator;
import java.util.UUID;

import static us.ihmc.perception.detections.doors.DoorDetectionManager.DOOR_STRING;
import static us.ihmc.perception.detections.doors.DoorDetectionManager.PANEL_STRING;

public class DetectedDoor
{
   private final UUID detectionUUID;

   // Opening Mechanism
   private final DoorOpeningMechanism openingMechanism;

   // Panel
   private final Pose3D panelPose;
   private final PlanarRegion panelPlanarRegion;

   // Detection stuff
   private Instant lastDetectionTime = Instant.MIN;

   // Parameters
   private double maxPlanarRegionToOpeningMechanismDistance = 0.75;
   private double minPlanarRegionArea = 0.2 * (DoorModelParameters.DOOR_PANEL_WIDTH * DoorModelParameters.DOOR_PANEL_HEIGHT); // 1/5th of panel area

   public DetectedDoor()
   {
      detectionUUID = UUID.randomUUID();

      openingMechanism = new DoorOpeningMechanism();

      panelPose = new Pose3D();
      panelPose.setToNaN();

      panelPlanarRegion = new PlanarRegion();
   }

   public DoorOpeningMechanism getOpeningMechanism()
   {
      return openingMechanism;
   }

   public Pose3DReadOnly getPanelPose()
   {
      return panelPose;
   }

   public boolean hasPanelPose()
   {
      return !panelPose.containsNaN();
   }

   public PlanarRegion getPanelPlanarRegion()
   {
      return panelPlanarRegion;
   }

   public boolean hasPanelPlanarRegion()
   {
      return panelPlanarRegion.getArea() > 0.0;
   }

   public Instant getLastDetectedTime()
   {
      return lastDetectionTime;
   }

   /* package-private */ double distanceSquared(InstantDetection detection)
   {
      String detectedClass = detection.getDetectedObjectClass().toLowerCase();

      if (detectedClass.contains(PANEL_STRING))
         return detection.getPose().getPosition().distanceSquared(getPanelPose().getPosition());
      else if (detectedClass.startsWith(DOOR_STRING))
         return detection.getPose().getPosition().distanceSquared(openingMechanism.getPosition());

      return Double.POSITIVE_INFINITY;
   }

   /* package-private */ void updateDetection(InstantDetection newDetection)
   {
      String detectedClass = newDetection.getDetectedObjectClass().toLowerCase();

      if (detectedClass.contains(PANEL_STRING))
      {
         panelPose.getPosition().set(newDetection.getPose().getPosition());
      }
      else if (detectedClass.startsWith(DOOR_STRING))
      {
         openingMechanism.setName(detectedClass);
         openingMechanism.setPosition(newDetection.getPose().getPosition());
      }
      else
      {
         return;
      }

      if (newDetection.getDetectionTime().isAfter(lastDetectionTime))
         lastDetectionTime = newDetection.getDetectionTime();
   }

   /* package-private */ void updatePlanarRegion(PlanarRegionsList planarRegions)
   {
      if (!openingMechanism.isPositionKnown() || planarRegions.isEmpty())
         return;

      PlanarRegion bestFitRegion = planarRegions.getPlanarRegionsAsList()                                   // Get planar regions as List<PlanarRegion>
                                                .stream()                                                   // Turn it into a stream
                                                .filter(this::planarRegionAreaFilter)                       // Filter out tiny regions
                                                .filter(this::planarRegionDistanceToOpeningMechanismFilter) // Filter out regions that are too far away
                                                .max(Comparator.comparingDouble(PlanarRegion::getArea))     // Find the largest region of the remaining
                                                .orElse(null);

      if (bestFitRegion == null)
         return;

      panelPlanarRegion.set(bestFitRegion);
      Point3DReadOnly regionCentroid = PlanarRegionTools.getCentroid3DInWorld(panelPlanarRegion);
      Line2D doorNormalLine = new Line2D(regionCentroid.getX(),
                                         regionCentroid.getY(),
                                         panelPlanarRegion.getNormalX(),
                                         panelPlanarRegion.getNormalY());

      double planarRegionYaw = TupleTools.angle(Axis2D.X, doorNormalLine.getDirection());

      openingMechanism.setOrientation(new Quaternion(planarRegionYaw, 0.0, 0.0));
      panelPose.getRotation().setYawPitchRoll(planarRegionYaw, 0.0, 0.0);
   }

   private boolean planarRegionDistanceToOpeningMechanismFilter(PlanarRegion planarRegion)
   {
      Point3DReadOnly planarRegionCentroid = PlanarRegionTools.getCentroid3DInWorld(planarRegion);
      double distanceToOpeningMechanism = planarRegionCentroid.distance(openingMechanism.getPosition());
      return distanceToOpeningMechanism < maxPlanarRegionToOpeningMechanismDistance;
   }

   private boolean planarRegionAreaFilter(PlanarRegion planarRegion)
   {
      return planarRegion.getArea() > minPlanarRegionArea;
   }

   public void toMessage(DetectedDoorMessage messageToPack)
   {
      MessageTools.toMessage(detectionUUID, messageToPack.getDetectionUuid());
      openingMechanism.toMessage(messageToPack.getOpeningMechanism());
      messageToPack.getPanelPose().set(panelPose);
      messageToPack.getPanelPlanarRegion().set(PlanarRegionMessageConverter.convertToPlanarRegionMessage(panelPlanarRegion));
      MessageTools.toMessage(lastDetectionTime, messageToPack.getLastDetectionTime());
   }
}
