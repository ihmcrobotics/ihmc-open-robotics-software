package us.ihmc.gdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class GDXDoorPushHandleRightFiducialStaticHandleObject extends GDXEnvironmentObject
{
   public static final String NAME = "Push Handle Right Door";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorPushHandleRightFiducialStaticHandleObject.class);

   public GDXDoorPushHandleRightFiducialStaticHandleObject()
   {
      this(YoAppearance.LightSkyBlue());
   }

   public GDXDoorPushHandleRightFiducialStaticHandleObject(AppearanceDefinition collisionMeshColor)
   {
      super(NAME, FACTORY);
      Model realisticModel
            = GDXModelLoader.loadG3DModel("environmentObjects/door/doorPushHandleRightFiducialStaticHandle/DoorPushHandleRightFiducialStaticHandle.g3dj");

      double heightZ = 2.0447; // these were measured in blender
      double widthY = 0.9144;
      double lengthX = 0.0508;
      setMass(100.0f);
      RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
      collisionShapeOffset.getTranslation().set(0.0, widthY / 2.0 + 0.003, heightZ / 2.0);

      Box3D collisionBox = new Box3D(lengthX, widthY, heightZ);

      Sphere3D boundingSphere = new Sphere3D(collisionBox.getSize().length() / 2.0);
      boundingSphere.getPosition().set(collisionShapeOffset.getTranslation());

      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(collisionMeshColor);
         meshBuilder.addBox((float) lengthX + 0.001, (float) widthY + 0.001, (float) heightZ + 0.001, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
      }, getPascalCasedName() + "CollisionModel" + getObjectIndex());
      collisionGraphic.materials.get(0).set(new BlendingAttribute(true, 0.4f));

      collisionBox.getPose().getTranslation().set(collisionShapeOffset.getTranslation());

      RigidBodyTransform wholeThingOffset = new RigidBodyTransform();
      wholeThingOffset.appendYawRotation(-Math.PI / 2.0);

      create(realisticModel, collisionGraphic, collisionShapeOffset, wholeThingOffset, boundingSphere, collisionBox, collisionBox::isPointInside);
   }
}
