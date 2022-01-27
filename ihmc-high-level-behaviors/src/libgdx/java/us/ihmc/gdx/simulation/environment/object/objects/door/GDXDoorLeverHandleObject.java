package us.ihmc.gdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.physics.bullet.collision.*;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class GDXDoorLeverHandleObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Lever Handle";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorLeverHandleObject.class);

   public GDXDoorLeverHandleObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/door/doorLeverHandle/DoorLeverHandle.g3dj");

      double sizeX = 0.065;
      double sizeY = 0.14;
      double sizeZ = 0.065;
      setMass(0.7f);
      RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
      collisionShapeOffset.getTranslation().add(-sizeX / 2.0, -0.04, 0.0);
      Sphere3D boundingSphere = new Sphere3D(0.2);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
      }, getPascalCasedName() + "CollisionModel" + getObjectIndex());
      collisionGraphic.materials.get(0).set(new BlendingAttribute(true, 0.4f));
      RigidBodyTransform wholeThingOffset = new RigidBodyTransform();
      getBulletCollisionOffset().setIdentity();
      create(realisticModel, collisionGraphic, collisionShapeOffset, wholeThingOffset, boundingSphere, collisionBox, collisionBox::isPointInside);

      btTriangleIndexVertexArray btTriangleIndexVertexArray = new btTriangleIndexVertexArray(realisticModel.meshParts.get(0));
      btGImpactMeshShape btGImpactMeshShape = new btGImpactMeshShape(btTriangleIndexVertexArray);
      btGImpactMeshShape.updateBound();
      setBtCollisionShape(btGImpactMeshShape);
   }
}
