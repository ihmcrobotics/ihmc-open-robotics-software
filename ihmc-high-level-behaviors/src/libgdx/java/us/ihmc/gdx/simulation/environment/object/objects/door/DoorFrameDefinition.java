package us.ihmc.gdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class DoorFrameDefinition extends RigidBodyDefinition
{
   public DoorFrameDefinition()
   {
      super("frameBody");

      double sizeX = 0.0889;
      double sizeY = 0.0889;
      double sizeZ = 2.159;

      setMass(100.0f);
      double radiusOfGyrationPercent = 0.8;
      setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(getMass(),
                                                                           radiusOfGyrationPercent * sizeX,
                                                                           radiusOfGyrationPercent * sizeY,
                                                                           radiusOfGyrationPercent * sizeZ));

      Vector3D centerOfMassInModelFrame = new Vector3D(0.0, (0.939887 / 2.0) - 0.006046, 0.1);
      getInertiaPose().getTranslation().set(centerOfMassInModelFrame);
      getInertiaPose().getRotation().setToZero();

      VisualDefinition frameModelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition("environmentObjects/door/doorFrame/DoorFrame.g3dj");
      frameModelVisualDefinition.setGeometryDefinition(geometryDefinition);
      addVisualDefinition(frameModelVisualDefinition);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      Box3DDefinition boxCollisionDefinition = new Box3DDefinition();
      boxCollisionDefinition.setSize(sizeX, sizeY, sizeZ);
      collisionShapeDefinition.setGeometryDefinition(boxCollisionDefinition);
      addCollisionShapeDefinition(collisionShapeDefinition);
   }
}
