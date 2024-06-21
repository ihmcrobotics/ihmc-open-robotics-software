package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism.DoorOpeningMechanismType;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorPanel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class DoorNode extends SceneNode
{
   private static final Pose3D ZERO_POSE = new Pose3D();

   private final Pose3D doorFramePose = new Pose3D(); // To know which way the door opens. X points in the direction that the door swings.
   private final DoorPanel doorPanel = new DoorPanel(this);
   private final List<DoorOpeningMechanism> openingMechanisms = new ArrayList<>();

   public DoorNode(long id, CRDTInfo crdtInfo)
   {
      super(id, "Door" + id, crdtInfo);
   }

   public Pose3D getDoorFramePose()
   {
      return doorFramePose;
   }

   public DoorPanel getDoorPanel()
   {
      return doorPanel;
   }

   public List<DoorOpeningMechanism> getOpeningMechanisms()
   {
      return openingMechanisms;
   }

   public void update()
   {
      // Calculate yaw, pitch, roll of opening mechanism pose
      PlanarRegion planarRegion = doorPanel.getPlanarRegion();

      if (planarRegion.getArea() > 0)
      {
         // Update the scene node reference frame
         getNodeToParentFrameTransform().set(planarRegion.getTransformToWorld());
         getNodeFrame().update();

         Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(planarRegion);
         Line2D doorLineNormal = new Line2D(planarRegionCentroidInWorld.getX(),
                                            planarRegionCentroidInWorld.getY(),
                                            planarRegion.getNormalX(),
                                            planarRegion.getNormalY());

         double yaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());

         // If doorFramePose is zero, this means we are just now perceiving the door.
         // We assume all doors are "push doors" when we first see them.
         // TODO: remove assumption door is a "push door" when we first see it
         if (doorFramePose.epsilonEquals(ZERO_POSE, 0.0))
         {
            doorFramePose.getTranslation().set(planarRegionCentroidInWorld);
            doorFramePose.getRotation().setYawPitchRoll(Math.PI * yaw, 0.0, 0.0);
         }

         // Update the opening mechanism poses with the planar region orientation,
         // special case for the LEVER_HANDLE
         for (DoorOpeningMechanism openingMechanism : openingMechanisms)
         {
            Pose3D openingMechanismPose = openingMechanism.getGraspPose();
            Point2D openingMechanismPointInWorld2D = new Point2D(openingMechanismPose.getTranslation());
            RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(openingMechanismPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;
            double pitch = 0.0;
            double roll = 0.0;
            if (openingMechanism.getType() == DoorOpeningMechanismType.LEVER_HANDLE)
               roll += doorSide == RobotSide.LEFT ? Math.PI : 0.0;
            openingMechanismPose.getRotation().setYawPitchRoll(yaw, pitch, roll);
         }
      }
   }
}
