package us.ihmc.perception.sceneGraph.multiBodies.door;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class DoorPanelDefinition extends RigidBodyDefinition
{
   private boolean addFiducials = false;

   public DoorPanelDefinition()
   {
      super("panelBody");
   }

   public void setAddFiducials(boolean addFiducials)
   {
      this.addFiducials = addFiducials;
   }

   public void build()
   {
      double sizeX = 0.0508; // centered on X
      double sizeY = 0.9144;
      double sizeZ = 2.0447;
      Point3D centerOfMassOffset = new Point3D(0.0, sizeY / 2.0, sizeZ / 2.0);

      double simulationY = sizeY - 0.05; // prevent door hinge self collision

      double mass = 70.0;
      setMass(mass);
      double radiusOfGyrationPercent = 0.8;
      setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(getMass(),
                                                                           radiusOfGyrationPercent * sizeX,
                                                                           radiusOfGyrationPercent * sizeY,
                                                                           radiusOfGyrationPercent * sizeZ));

      getInertiaPose().getTranslation().set(centerOfMassOffset);
      getInertiaPose().getRotation().setToZero();

      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition(DoorSceneNodeDefinitions.DOOR_PANEL_VISUAL_MODEL_FILE_PATH);
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      addVisualDefinition(modelVisualDefinition);

      if (addFiducials)
      {
         VisualDefinition fiducialModelVisualDefinition = new VisualDefinition();
         ModelFileGeometryDefinition fiducialGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/door/doorPanel/DoorPanelFiducials.g3dj");
         fiducialModelVisualDefinition.setGeometryDefinition(fiducialGeometryDefinition);
         addVisualDefinition(fiducialModelVisualDefinition);
      }

      Point3D collisionShapeOffset = new Point3D(0.0, sizeY / 2.0 + 0.025, sizeZ / 2.0);
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      Box3DDefinition boxCollisionDefinition = new Box3DDefinition(sizeX, simulationY, sizeZ);
      collisionShapeDefinition.setGeometryDefinition(boxCollisionDefinition);
      collisionShapeDefinition.getOriginPose().set(new YawPitchRoll(), collisionShapeOffset);
      addCollisionShapeDefinition(collisionShapeDefinition);
   }
}
