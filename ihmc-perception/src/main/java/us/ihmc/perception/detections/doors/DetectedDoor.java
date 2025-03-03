package us.ihmc.perception.detections.doors;

import perception_msgs.msg.dds.DetectedDoorMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorModelParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.Comparator;
import java.util.Objects;
import java.util.UUID;

import static us.ihmc.perception.detections.doors.DoorDetectionManager.*;

public class DetectedDoor
{
   private static final double MAX_PLANAR_REGION_TO_PANEL_DISTANCE = 0.6 * DoorModelParameters.DOOR_PANEL_WIDTH;
   private static final double MAX_PLANAR_REGION_TO_OPENING_MECHANISM_DISTANCE = 0.75;
   private static final double MIN_PLANAR_REGION_AREA = 0.2 * (DoorModelParameters.DOOR_PANEL_WIDTH * DoorModelParameters.DOOR_PANEL_HEIGHT); // 1/5th of panel area
   private static final double STABLE_DETECTIONS_PER_SECOND = 5.0;
   private static final double POSE_FILTER_ALPHA = 0.5;

   private final UUID detectionUUID;

   // Opening Mechanism
   private final DoorOpeningMechanism openingMechanism;

   // Panel
   private final MutableReferenceFrame panelFrame;
   private final PlanarRegion panelPlanarRegion;

   // Detection stuff
   private Instant lastDetectionTime = Instant.MIN;

   // Internal statistics
   private Instant openingMechanismFirstDetectionTime = null;
   private int openingMechanismDetectionCount = 0;
   private Instant panelFirstDetectionTime = null;
   private int panelDetectionCount = 0;

   public DetectedDoor()
   {
      detectionUUID = UUID.randomUUID();

      openingMechanism = new DoorOpeningMechanism();

      panelFrame = new MutableReferenceFrame();
      panelFrame.update(RigidBodyTransformBasics::setToNaN);

      panelPlanarRegion = new PlanarRegion();
   }

   public DetectedDoor(DetectedDoorMessage message)
   {
      detectionUUID = MessageTools.toUUID(message.getDetectionUuid());
      openingMechanism = new DoorOpeningMechanism(message.getOpeningMechanism());
      panelFrame = new MutableReferenceFrame();
      panelFrame.update(transformToWorld -> transformToWorld.set(message.getPanelPose()));
      panelPlanarRegion = PlanarRegionMessageConverter.convertToPlanarRegion(message.getPanelPlanarRegion());
      lastDetectionTime = MessageTools.toInstant(message.getLastDetectionTime());

      openingMechanismFirstDetectionTime = MessageTools.toInstant(message.getOpeningMechanismFirstDetectionTime());
      if (openingMechanismFirstDetectionTime.equals(Instant.EPOCH))
         openingMechanismFirstDetectionTime = null;
      openingMechanismDetectionCount = message.getOpeningMechanismDetectionCount();

      panelFirstDetectionTime = MessageTools.toInstant(message.getPanelFirstDetectionTime());
      if (panelFirstDetectionTime.equals(Instant.EPOCH))
         panelFirstDetectionTime = null;
      panelDetectionCount = message.getPanelDetectionCount();
   }

   public UUID getDetectionUUID()
   {
      return detectionUUID;
   }

   public DoorOpeningMechanism getOpeningMechanism()
   {
      return openingMechanism;
   }

   public Pose3DReadOnly getPanelPose()
   {
      return new Pose3D(panelFrame.getTransformToParent());
   }

   public ReferenceFrame getPanelFrame()
   {
      return panelFrame.getReferenceFrame();
   }

   public RigidBodyTransformReadOnly getPanelTransformToWorld()
   {
      return panelFrame.getTransformToParent();
   }

   public boolean hasPanelPosition()
   {
      return !getPanelTransformToWorld().getTranslation().containsNaN();
   }

   public boolean hasPanelOrientation()
   {
      return !getPanelTransformToWorld().getRotation().containsNaN();
   }

   public boolean hasPanelPose()
   {
      return !getPanelTransformToWorld().containsNaN();
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

   public boolean isDetectionStable()
   {
      return isOpeningMechanismDetectionStable() || isPanelDetectionStable();
   }

   public boolean isOpeningMechanismDetectionStable()
   {
      if (openingMechanismFirstDetectionTime == null)
         return false;

      Instant now = Instant.now();
      boolean isOldEnough = now.isAfter(openingMechanismFirstDetectionTime.plusSeconds((long) DETECTION_EXPIRATION_SECONDS));
      double detectionsPerSecond =
            openingMechanismDetectionCount / Conversions.millisecondsToSeconds((openingMechanismFirstDetectionTime.until(now, ChronoUnit.MILLIS)));
      return isOldEnough && detectionsPerSecond > STABLE_DETECTIONS_PER_SECOND;
   }

   public boolean isPanelDetectionStable()
   {
      if (panelFirstDetectionTime == null)
         return false;

      Instant now = Instant.now();
      boolean isOldEnough = now.isAfter(panelFirstDetectionTime.plusSeconds((long) DETECTION_EXPIRATION_SECONDS));
      double detectionsPerSecond =
            panelDetectionCount / Conversions.millisecondsToSeconds((panelFirstDetectionTime.until(now, ChronoUnit.MILLIS)));
      return isOldEnough && detectionsPerSecond > STABLE_DETECTIONS_PER_SECOND;
   }

   /* package-private */ double distanceSquared(InstantDetection detection)
   {
      String detectedClass = detection.getDetectedObjectClass().toLowerCase();

      if (detectedClass.contains(PANEL_STRING))
      {
         if (hasPanelPosition())
            return detection.getPose().getPosition().distanceSquared(getPanelPose().getPosition());
         else if (openingMechanism.isPositionKnown())
            return detection.getPose().getPosition().distanceSquared(openingMechanism.getPosition());
      }
      else if (detectedClass.startsWith(DOOR_STRING))
      {
         if (openingMechanism.isPositionKnown())
            return detection.getPose().getPosition().distanceSquared(openingMechanism.getPosition());
         else if (hasPanelPosition())
            return detection.getPose().getPosition().distanceSquared(getPanelPose().getPosition());
      }

      return Double.POSITIVE_INFINITY;
   }

   /* package-private */ void updateDetection(InstantDetection newDetection)
   {
      String detectedClass = newDetection.getDetectedObjectClass().toLowerCase();

      if (detectedClass.contains(PANEL_STRING))
      {
         if (panelFirstDetectionTime == null)
            panelFirstDetectionTime = newDetection.getDetectionTime();
         panelDetectionCount++;
         updatePanelPosition(newDetection.getPose().getPosition());
      }
      else if (detectedClass.startsWith(DOOR_STRING))
      {
         if (openingMechanismFirstDetectionTime == null)
            openingMechanismFirstDetectionTime = newDetection.getDetectionTime();
         openingMechanismDetectionCount++;
         openingMechanism.setName(detectedClass);
         openingMechanism.updatePosition(newDetection.getPose().getPosition(), POSE_FILTER_ALPHA);
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

      RotationMatrix orientation = new RotationMatrix(planarRegionYaw, 0.0, 0.0);
      openingMechanism.updateOrientation(orientation, POSE_FILTER_ALPHA);
      updatePanelOrientation(orientation);
   }

   private boolean planarRegionDistanceToOpeningMechanismFilter(PlanarRegion planarRegion)
   {
      Point3DReadOnly planarRegionCentroid = PlanarRegionTools.getCentroid3DInWorld(planarRegion);
      if (hasPanelPosition())
      {
         double distanceToPanel = planarRegionCentroid.distance(getPanelPose().getPosition());
         return distanceToPanel < MAX_PLANAR_REGION_TO_PANEL_DISTANCE;
      }
      else if (openingMechanism.isPositionKnown())
      {
         double distanceToOpeningMechanism = planarRegionCentroid.distance(openingMechanism.getPosition());
         return distanceToOpeningMechanism < MAX_PLANAR_REGION_TO_OPENING_MECHANISM_DISTANCE;
      }

      return false;
   }

   private boolean planarRegionAreaFilter(PlanarRegion planarRegion)
   {
      return planarRegion.getArea() > MIN_PLANAR_REGION_AREA;
   }

   private void updatePanelPosition(Point3DReadOnly newPanelPosition)
   {
      panelFrame.update(transformToWorld ->
      {
         if (hasPanelPosition())
            transformToWorld.getTranslation().interpolate(newPanelPosition, POSE_FILTER_ALPHA);
         else
            transformToWorld.getTranslation().set(newPanelPosition);
      });
   }

   private void updatePanelOrientation(RotationMatrixReadOnly newPanelOrientation)
   {
      panelFrame.update(transformToWorld ->
      {
         if (hasPanelOrientation())
            transformToWorld.getRotation().interpolate(newPanelOrientation, POSE_FILTER_ALPHA);
         else
            transformToWorld.getRotation().set(newPanelOrientation);
      });
   }

   public void toMessage(DetectedDoorMessage messageToPack)
   {
      MessageTools.toMessage(detectionUUID, messageToPack.getDetectionUuid());
      openingMechanism.toMessage(messageToPack.getOpeningMechanism());
      messageToPack.getPanelPose().set(getPanelPose());
      messageToPack.getPanelPlanarRegion().set(PlanarRegionMessageConverter.convertToPlanarRegionMessage(panelPlanarRegion));
      MessageTools.toMessage(lastDetectionTime, messageToPack.getLastDetectionTime());
      MessageTools.toMessage(Objects.requireNonNullElse(openingMechanismFirstDetectionTime, Instant.EPOCH), messageToPack.getOpeningMechanismFirstDetectionTime());
      messageToPack.setOpeningMechanismDetectionCount(openingMechanismDetectionCount);
      MessageTools.toMessage(Objects.requireNonNullElse(panelFirstDetectionTime, Instant.EPOCH), messageToPack.getPanelFirstDetectionTime());
      messageToPack.setPanelDetectionCount(panelDetectionCount);
   }
}
