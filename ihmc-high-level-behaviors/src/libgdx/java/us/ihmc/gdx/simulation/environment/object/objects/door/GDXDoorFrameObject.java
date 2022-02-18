package us.ihmc.gdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;

public class GDXDoorFrameObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Frame";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorFrameObject.class);

   public GDXDoorFrameObject()
   {
      super(NAME, FACTORY);
      setRealisticModel(GDXModelLoader.loadG3DModel("environmentObjects/door/doorFrame/DoorFrame.g3dj"));
      double sizeX = 0.0889;
      double sizeY = 0.0889;
      double sizeZ = 2.159;
      getCenterOfMassInModelFrame().set(0.0, (0.939887 / 2.0) - 0.006046, 0.1);
      setMass(100.0f);
      getRealisticModelOffset().getTranslation().sub(getCenterOfMassInModelFrame());
      getCollisionShapeOffset().getTranslation().sub(getCenterOfMassInModelFrame());
      getCollisionShapeOffset().getTranslation().add(0.0, -0.006064, 0.0);
      getCollisionShapeOffset().getTranslation().add(0.0, -0.0889 / 2.0, 2.159 / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      getBoundingSphere().setRadius(1.5);
      getBoundingSphere().getPosition().sub(getCenterOfMassInModelFrame());
      getBoundingSphere().getPosition().add(0.0, 0.0, 2.159 / 2.0);

      RigidBodyTransform nearOriginFramePostOrigin = new RigidBodyTransform();
      nearOriginFramePostOrigin.getTranslation().sub(getCenterOfMassInModelFrame());
      nearOriginFramePostOrigin.getTranslation().add(0.0, -0.006064, 0.0);
      nearOriginFramePostOrigin.getTranslation().add(0.0, -0.0889 / 2.0, 2.159 / 2.0);

      RigidBodyTransform nearOriginSupportBoardOrigin = new RigidBodyTransform();
      nearOriginSupportBoardOrigin.getTranslation().sub(getCenterOfMassInModelFrame());
      nearOriginSupportBoardOrigin.getTranslation().add(0.0, -0.006064, 0.0);
      nearOriginSupportBoardOrigin.getTranslation().add(0.0, -0.0889, 0.0);
      nearOriginSupportBoardOrigin.getTranslation().add(0.0, -0.0381 / 2.0, 0.0889 / 2.0f);

      RigidBodyTransform farFramePostOrigin = new RigidBodyTransform();
      farFramePostOrigin.getTranslation().sub(getCenterOfMassInModelFrame());
      farFramePostOrigin.getTranslation().add(0.0, 0.93415, 0.0);
      farFramePostOrigin.getTranslation().add(0.0, 0.0889 / 2.0, 2.159 / 2.0);

      RigidBodyTransform farSupportBoardOrigin = new RigidBodyTransform();
      farSupportBoardOrigin.getTranslation().sub(getCenterOfMassInModelFrame());
      farSupportBoardOrigin.getTranslation().add(0.0, 0.93415, 0.0);
      farSupportBoardOrigin.getTranslation().add(0.0, 0.0889, 0.0);
      farSupportBoardOrigin.getTranslation().add(0.0, 0.0381 / 2.0, 0.0889 / 2.0f);

      Matrix4 tempTransform = new Matrix4();
      btCompoundShape btCompoundShape = new btCompoundShape();
      GDXTools.toGDX(nearOriginFramePostOrigin, tempTransform);
      btCompoundShape.addChildShape(tempTransform, new btBoxShape(new Vector3(0.0889f / 2.0f, 0.0889f / 2.0f, 2.159f / 2.0f)));
      GDXTools.toGDX(nearOriginSupportBoardOrigin, tempTransform);
      btCompoundShape.addChildShape(tempTransform, new btBoxShape(new Vector3(1.016f / 2.0f, 0.0381f / 2.0f, 0.0889f / 2.0f)));
      GDXTools.toGDX(farFramePostOrigin, tempTransform);
      btCompoundShape.addChildShape(tempTransform, new btBoxShape(new Vector3(0.0889f / 2.0f, 0.0889f / 2.0f, 2.159f / 2.0f)));
      GDXTools.toGDX(farSupportBoardOrigin, tempTransform);
      btCompoundShape.addChildShape(tempTransform, new btBoxShape(new Vector3(1.016f / 2.0f, 0.0381f / 2.0f, 0.0889f / 2.0f)));

      setBtCollisionShape(btCompoundShape);
   }
}
