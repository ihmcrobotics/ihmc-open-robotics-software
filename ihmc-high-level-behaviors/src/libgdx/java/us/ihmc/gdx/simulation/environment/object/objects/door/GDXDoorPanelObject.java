package us.ihmc.gdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class GDXDoorPanelObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Panel";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorPanelObject.class);

   public GDXDoorPanelObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/door/doorPanel/DoorPanel.g3dj");

      double sizeX = 0.05; // centered on X
      double sizeY = 0.9144;
      double sizeZ = 2.0;
      setMass(70.0f);
      RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
      collisionShapeOffset.getTranslation().add(0.0, sizeY / 2.0, sizeZ / 2.0);
      Sphere3D boundingSphere = new Sphere3D(2.5);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
      }, getPascalCasedName() + "CollisionModel" + getObjectIndex());
      collisionGraphic.materials.get(0).set(new BlendingAttribute(true, 0.4f));
      RigidBodyTransform wholeThingOffset = new RigidBodyTransform();
      getBulletCollisionOffset().set(collisionShapeOffset);
      create(realisticModel, collisionGraphic, collisionShapeOffset, wholeThingOffset, boundingSphere, collisionBox, collisionBox::isPointInside);

      setBtCollisionShape(new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f)));
   }
}
