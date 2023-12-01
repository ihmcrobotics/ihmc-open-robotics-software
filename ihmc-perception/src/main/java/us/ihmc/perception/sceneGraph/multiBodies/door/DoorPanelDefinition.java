package us.ihmc.perception.sceneGraph.multiBodies.door;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import static us.ihmc.perception.sceneGraph.multiBodies.door.DoorModelParameters.*;

/**
 * This is a door panel with optional ArUco markers.
 * ArUco marker ID 0 on the pull side and ID 1 on the push side.
 */
public class DoorPanelDefinition extends RigidBodyDefinition
{
   private boolean addArUcoMarkers = false;

   public DoorPanelDefinition()
   {
      super("panelBody");
   }

   public void setAddArUcoMarkers(boolean addFiducials)
   {
      this.addArUcoMarkers = addFiducials;
   }

   public void build()
   {
      double sizeX = DOOR_PANEL_THICKNESS; // centered on X
      double physicalPanelSizeY = DOOR_PANEL_WIDTH;
      double sizeZ = DOOR_PANEL_HEIGHT;
      Point3D centerOfMassOffset = new Point3D(physicalPanelSizeY / 2.0, physicalPanelSizeY / 2.0, sizeZ / 2.0);

      double hingeAvoidanceOffset = 0.05;
      double panelCollisionSizeY = physicalPanelSizeY - hingeAvoidanceOffset; // prevent door hinge self collision

      double mass = 70.0;
      setMass(mass);
      double radiusOfGyrationPercent = 0.8;
      setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(getMass(),
                                                                           radiusOfGyrationPercent * sizeX,
                                                                           radiusOfGyrationPercent * physicalPanelSizeY,
                                                                           radiusOfGyrationPercent * sizeZ));

      getInertiaPose().getTranslation().set(centerOfMassOffset);
      getInertiaPose().getRotation().setToZero();

      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition(DoorSceneNodeDefinitions.DOOR_PANEL_VISUAL_MODEL_FILE_PATH);
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      modelVisualDefinition.getOriginPose().getTranslation().setX(DOOR_PANEL_THICKNESS / 2.0);
      addVisualDefinition(modelVisualDefinition);

      if (addArUcoMarkers)
      {
         VisualDefinition fiducialModelVisualDefinition = new VisualDefinition();
         ModelFileGeometryDefinition fiducialGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/door/doorPanel/DoorPanelFiducials.g3dj");
         fiducialModelVisualDefinition.setGeometryDefinition(fiducialGeometryDefinition);
         fiducialModelVisualDefinition.getOriginPose().getTranslation().setX(DOOR_PANEL_THICKNESS / 2.0);
         addVisualDefinition(fiducialModelVisualDefinition);
      }

      double bottomPartZ = DOOR_OPENER_FROM_BOTTOM_OF_PANEL - DOOR_BOLT_HOLE_HEIGHT / 2.0;
      Point3D collisionShapeOffset = new Point3D(DOOR_PANEL_THICKNESS / 2.0, panelCollisionSizeY / 2.0 + hingeAvoidanceOffset, bottomPartZ / 2.0);
      addCollisionShape(sizeX, panelCollisionSizeY, bottomPartZ, collisionShapeOffset);
      double topPartZ = sizeZ / 2.0 - DOOR_BOLT_HOLE_HEIGHT / 2.0;
      collisionShapeOffset.add(0.0, 0.0, bottomPartZ / 2.0 + DOOR_BOLT_HOLE_HEIGHT + topPartZ / 2.0);
      addCollisionShape(sizeX, panelCollisionSizeY, topPartZ, collisionShapeOffset);
   }

   private void addCollisionShape(double sizeX, double sizeY, double sizeZ, Point3D offset)
   {
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      Box3DDefinition boxCollisionDefinition = new Box3DDefinition();
      boxCollisionDefinition.setSize(sizeX, sizeY, sizeZ);
      collisionShapeDefinition.setGeometryDefinition(boxCollisionDefinition);
      collisionShapeDefinition.getOriginPose().set(new YawPitchRoll(), offset);
      addCollisionShapeDefinition(collisionShapeDefinition);
   }
}
