package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism.DoorOpeningMechanismType;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorPanel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.stream.Collectors;

/**
 * A node that represents a door.
 * This includes a frame, hinged swinging panel that swings one way,
 * and opening mechanisms on either side of the panel.
 *
 * The X forward direction of this node points straight through the frame,
 * towards the side that the door panel swings open to.
 * The origin of the frame is positioned at the bottom of the panel hinge.
 */
public class DoorNode extends DetectableSceneNode
{
   // Maximum distance one door component can be from another (squared meters)
   public static final double DOOR_COMPONENT_DISTANCE_THRESHOLD = MathTools.square(1.0);
   private static final Pose3D NAN_POSE = new Pose3D();
   static
   {
      NAN_POSE.setToNaN();
   }

   private final MutableReferenceFrame doorCornerFrame; // To know which way the door opens. X points in the direction that the door swings.
   private final DoorPanel doorPanel = new DoorPanel(this);
   private final Map<UUID, DoorOpeningMechanism> openingMechanisms = new HashMap<>();
   private final Map<DoorOpeningMechanism, SceneNode> openingMechanismToHelperNodeMap = new HashMap<>();

   // translation from opening mechanism to door hinge corner when looking towards push side (invert y is looking towards pull side)
   private final Vector3D openingMechanismToHingeCornerTranslation = new Vector3D(NAN_POSE.getTranslation());

   private boolean lockDoorFramePose = false;

   public DoorNode(long id, CRDTInfo crdtInfo)
   {
      this(id, null, crdtInfo);
   }

   public DoorNode(long id, PersistentDetection initialDetection, CRDTInfo crdtInfo)
   {
      super(id, "Door" + id, crdtInfo);

      doorCornerFrame = new MutableReferenceFrame("doorCorner_" + super.getID(), ReferenceFrame.getWorldFrame(), NAN_POSE);

      if (initialDetection != null && !acceptDetection(initialDetection))
         throw new IllegalArgumentException("Oops, something went super wrong. FIXME PLEASE");
   }

   /**
    * Accepts a detection of a door component (e.g. door handle, door panel, etc).
    * @param doorComponentDetection {@link PersistentDetection} of a door component
    * @return true if the detection is accepted, false otherwise.
    */
   public boolean acceptDetection(PersistentDetection doorComponentDetection)
   {
      if (doorComponentDetection == null)
         return false;

      if (doorPanel.acceptDetection(doorComponentDetection))
         return true;
      else
         return acceptOpeningMechanismDetection(doorComponentDetection);
   }

   private boolean acceptOpeningMechanismDetection(PersistentDetection doorOpeningMechanismDetection)
   {
      // Detection must be of an opening mechanism
      if (!DoorNodeTools.detectionIsDoorOpeningMechanism(doorOpeningMechanismDetection))
         return false;

      // Detection must be close enough to the door panel detection (if it has been detected)
      PersistentDetection panelDetection = doorPanel.getDoorPanelDetection();
      if (panelDetection != null
          && panelDetection.getMostRecentPosition().distanceSquared(doorOpeningMechanismDetection.getMostRecentPosition())
             > DOOR_COMPONENT_DISTANCE_THRESHOLD)
         return false;

      // Detection must be close enough to existing opening mechanisms
      boolean closeToAllMechanisms = openingMechanisms.isEmpty() ||
         openingMechanisms.values()
                          .stream()
                          .allMatch(openingMechanism ->
                                    openingMechanism.getDetection()
                                                    .getMostRecentPosition()
                                                    .distanceSquared(doorOpeningMechanismDetection.getMostRecentPosition()) < DOOR_COMPONENT_DISTANCE_THRESHOLD);
      if (!closeToAllMechanisms)
         return false;

      /*
       * Figure out which side the door mechanism is on, assuming this is the first detection
       * of a door component for this node.
       * ALSO ASSUMES ONR DEMO COURSE. THIS WILL NOT WORK UNIVERSALLY.
       */
      DoorSide detectionSide;
      /*
       * ASSUMPTION: Robot will detect DoorLever and DoorKnob when entering main room.
       * The doors will be oriented such that they are push doors.
       * At this moment there will be no opening mechanisms in the list (first detection)
       */
      if (doorOpeningMechanismDetection.getDetectedObjectClass().contains("YOLODoorLever")
          || doorOpeningMechanismDetection.getDetectedObjectClass().contains("YOLODoorKnob"))
      {
         detectionSide = DoorSide.PUSH;

         // Not first detection; reverse the sides since our assumption was wrong.
         if (!openingMechanisms.isEmpty())
            detectionSide = DoorSide.getOppositeSide(detectionSide);
      }
      else if (doorOpeningMechanismDetection.getDetectedObjectClass().contains("YOLOPushBar"))
      {
         detectionSide = DoorSide.PUSH;
      }
      else // YOLOPullHandle
      {
         detectionSide = DoorSide.PULL;
      }

      // ASSUMPTION: Only one opening mechanism per door side
      if (!getOpeningMechanisms(detectionSide).isEmpty())
         return false;

      // Assign hinge translation if first opening mechanism
      if (openingMechanisms.isEmpty())
      {
         double openerToHingeCornerY = DoorModelParameters.DOOR_PANEL_WIDTH - DoorModelParameters.DOOR_OPENER_INSET;
         // Centroid of push bar is further right compared to other opening mechanisms
         if (doorOpeningMechanismDetection.getDetectedObjectName().contains("YOLOPushBar"))
            openerToHingeCornerY -= 0.25;


         /*
          * ASSUMPTIONS: In the ONR demo course the door's hing locations will be ties to the opening mechanisms
          * Push Bar    -> right
          * Door Knob   -> left
          * Door Handle -> right
          * (looking at the push side of the door)
          */
         if (!doorOpeningMechanismDetection.getDetectedObjectName().contains("YOLODoorKnob"))
            openerToHingeCornerY *= -1.0;

         double openerToHingeCornerZ = -1.0 * DoorModelParameters.DOOR_OPENER_FROM_BOTTOM_OF_PANEL;
         // Centroid of pull handle is higher up compared to other opening mechanisms
         if (doorOpeningMechanismDetection.getDetectedObjectName().contains("YOLOPullHandle"))
            openerToHingeCornerZ -= 0.18;

         openingMechanismToHingeCornerTranslation.set(0.0, openerToHingeCornerY, openerToHingeCornerZ);
      }

      // Must be new opening mechanism of this door; add to list
      DoorOpeningMechanism openingMechanism = new DoorOpeningMechanism(detectionSide,
                                                                       YOLOv8DetectionClass.fromName(doorOpeningMechanismDetection.getDetectedObjectName()),
                                                                       doorOpeningMechanismDetection.getID());
      openingMechanism.setDetection(doorOpeningMechanismDetection);
      openingMechanisms.put(doorOpeningMechanismDetection.getID(), openingMechanism);

      return true;
   }

   @Override
   public void update(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      super.update(sceneGraph, modificationQueue);

      // Calculate yaw, pitch, roll of opening mechanism pose based on door panel
      updateOpeningMechanismPoses();

      boolean openingMechanismDetected = openingMechanisms.values().stream().anyMatch(mechanism -> mechanism.getDetection().isStable());
      setCurrentlyDetected(doorPanel.isDetected() || openingMechanismDetected);

      for (DoorOpeningMechanism openingMechanism : openingMechanisms.values())
      {
         if (!openingMechanismToHelperNodeMap.containsKey(openingMechanism))
         {
            SceneNode helperNode = DoorNodeTools.addOpeningMechanismHelperNode(this, openingMechanism, sceneGraph, modificationQueue);
            openingMechanismToHelperNodeMap.put(openingMechanism, helperNode);
         }
      }
   }

   private void updateOpeningMechanismPoses()
   {
      for (DoorOpeningMechanism openingMechanism : openingMechanisms.values())
         openingMechanism.update();

      PlanarRegion planarRegion = doorPanel.getPlanarRegion();

      if (planarRegion.getArea() > 0)
      {
         Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(planarRegion);
         Line2D doorLineNormal = new Line2D(planarRegionCentroidInWorld.getX(),
                                            planarRegionCentroidInWorld.getY(),
                                            planarRegion.getNormalX(),
                                            planarRegion.getNormalY());

         double planarRegionYaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());

         // If doorFramePose is NaN, this means we are just now perceiving the door.
         // Attempt to set the orientation of the door pose according to the side of perceived opening mechanisms
         // If there are no opening mechanisms, or the opening mechanisms are on different sides, assume push door.
         if (doorCornerFrame.getTransformToParent().containsNaN() || !lockDoorFramePose)
         {
            RigidBodyTransform doorCornerTransformToWorld = new RigidBodyTransform();

            // Check if all opening mechanisms are on the same side.
            double doorFrameYaw = planarRegionYaw;
            DoorSide mechanismsSide = allMechanismsAreSameSide(openingMechanisms.values());
            if (mechanismsSide != null) // If so, set yaw to match the side
               doorFrameYaw = mechanismsSide == DoorSide.PUSH ? doorFrameYaw + Math.PI : doorFrameYaw;
            else // otherwise assume push door
               doorFrameYaw = doorFrameYaw + Math.PI;

            doorCornerTransformToWorld.getRotation().setYawPitchRoll(doorFrameYaw, 0.0, 0.0);

            DoorOpeningMechanism detectedOpeningMechanism = getLatestUpdatedOpeningMechanism();
            if (detectedOpeningMechanism != null)
            {
               doorCornerTransformToWorld.getTranslation().set(detectedOpeningMechanism.getMechanismFrame().getTransformToWorldFrame().getTranslation());
               doorCornerTransformToWorld.appendTranslation(openingMechanismToHingeCornerTranslation); // TODO: Invert y translation if looking at door from opposite side
            }
            else
               doorCornerTransformToWorld.getTranslation().set(planarRegionCentroidInWorld);

            doorCornerFrame.update(transformToWorld -> transformToWorld.set(doorCornerTransformToWorld));
         }

         // Update the opening mechanism poses with the planar region orientation,
         // special case for the LEVER_HANDLE
         for (DoorOpeningMechanism openingMechanism : openingMechanisms.values())
         {
            RigidBodyTransform mechanismTransformToWorld = new RigidBodyTransform(openingMechanism.getMechanismFrame().getTransformToWorldFrame());
            Point2D openingMechanismPointInWorld2D = new Point2D(mechanismTransformToWorld.getTranslation());
            RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(openingMechanismPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;
            double pitch = 0.0;
            double roll = 0.0;
            if (openingMechanism.getType() == DoorOpeningMechanismType.LEVER_HANDLE)
               roll += doorSide == RobotSide.LEFT ? Math.PI : 0.0;
            mechanismTransformToWorld.getRotation().setYawPitchRoll(planarRegionYaw, pitch, roll);
            openingMechanism.updateMechanismFrame(mechanismTransformToWorld);
         }

         // Update the scene node reference frame
         setNodeToParentFrameTransformAndUpdate(getLatestUpdatedOpeningMechanism().getMechanismFrame().getTransformToWorldFrame());
      }
   }

   public void setDoorFramePoseLock(boolean lockPose)
   {
      lockDoorFramePose = lockPose;
   }

   public boolean isDoorFramePoseLocked()
   {
      return lockDoorFramePose;
   }

   public void updateDoorCornerFrame(RigidBodyTransformReadOnly newTransformToWorld)
   {
      doorCornerFrame.update(transformToWorld -> transformToWorld.set(newTransformToWorld));
   }

   public ReferenceFrame getDoorCornerFrame()
   {
      return doorCornerFrame.getReferenceFrame();
   }

   public DoorPanel getDoorPanel()
   {
      return doorPanel;
   }

   public Map<UUID, DoorOpeningMechanism> getOpeningMechanisms()
   {
      return openingMechanisms;
   }

   public DoorOpeningMechanism getLatestUpdatedOpeningMechanism()
   {
      DoorOpeningMechanism candidate = null;

      for (DoorOpeningMechanism openingMechanism : openingMechanisms.values())
      {
         if (candidate == null)
         {
            candidate = openingMechanism;
         }
         else
         {
            if (openingMechanism.getLastDetection().getDetectionTime().isAfter(candidate.getLastDetection().getDetectionTime()))
            {
               candidate = openingMechanism;
            }
         }
      }

      return candidate;
   }

   public Set<DoorOpeningMechanism> getOpeningMechanisms(DoorSide doorSide)
   {
      return openingMechanisms.values().stream().filter(openingMechanism -> openingMechanism.getDoorSide() == doorSide).collect(Collectors.toSet());
   }

   public DoorSide getDoorSideRelativeTo(Point3D position)
   {
      // TODO: DOORNODES
      // Needed for determining whether we're looking at the push or pull handle

      return null;
   }

   /**
    * Checks whether all passed in mechanisms are on the same door side.
    * @return If all opening mechanisms are on the same side, returns that side.
    * {@code null} if openingMechanisms is empty or contains opening mechanisms of different sides.
    */
   private DoorSide allMechanismsAreSameSide(Collection<DoorOpeningMechanism> openingMechanisms)
   {
      DoorSide side = null;
      for (DoorOpeningMechanism openingMechanism : openingMechanisms)
      {
         if (side == null)
            side = openingMechanism.getDoorSide();
         else if (side != openingMechanism.getDoorSide())
            return null;
      }

      return side;
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);

      for (DoorOpeningMechanism openingMechanism : openingMechanisms.values())
         openingMechanism.destroy();

      doorPanel.destroy();
   }


   public enum DoorSide
   {
      PUSH(true), PULL(false);

      private final boolean booleanValue;

      DoorSide(boolean booleanValue)
      {
         this.booleanValue = booleanValue;
      }

      public static DoorSide getOppositeSide(DoorSide side)
      {
         return fromBoolean(!side.getBooleanValue());
      }

      public boolean getBooleanValue()
      {
         return booleanValue;
      }

      public static DoorSide fromBoolean(boolean doorSide)
      {
         return doorSide ? PUSH : PULL;
      }
   }
}
