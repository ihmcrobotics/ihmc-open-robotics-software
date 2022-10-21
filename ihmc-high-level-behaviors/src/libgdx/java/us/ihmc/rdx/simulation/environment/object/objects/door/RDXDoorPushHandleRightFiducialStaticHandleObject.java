package us.ihmc.rdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class RDXDoorPushHandleRightFiducialStaticHandleObject extends RDXEnvironmentObject
{
   public static final String NAME = "Push Handle Right Door";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXDoorPushHandleRightFiducialStaticHandleObject.class);

   public RDXDoorPushHandleRightFiducialStaticHandleObject()
   {
      this(YoAppearance.LightSkyBlue());
   }

   public RDXDoorPushHandleRightFiducialStaticHandleObject(AppearanceDefinition collisionMeshColor)
   {
      super(NAME, FACTORY);
      Model realisticModel
            = RDXModelLoader.load("environmentObjects/door/doorPushHandleRightFiducialStaticHandle/DoorPushHandleRightFiducialStaticHandle.g3dj");
      setRealisticModel(realisticModel);

      double heightZ = 2.0447; // these were measured in blender
      double widthY = 0.9144;
      double lengthX = 0.0508;
      setMass(100.0f);
      getCollisionShapeOffset().getTranslation().set(0.0, widthY / 2.0 + 0.003, heightZ / 2.0);

      Box3D collisionBox = new Box3D(lengthX, widthY, heightZ);

      getBoundingSphere().setRadius(collisionBox.getSize().length() / 2.0);
      getBoundingSphere().getPosition().set(getCollisionShapeOffset().getTranslation());
      setCollisionModel(meshBuilder ->
      {
         Color color = LibGDXTools.toLibGDX(collisionMeshColor);
         meshBuilder.addBox((float) lengthX + 0.001, (float) widthY + 0.001, (float) heightZ + 0.001, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
      });

      collisionBox.getPose().getTranslation().set(getCollisionShapeOffset().getTranslation());
      setCollisionGeometryObject(collisionBox);

      getRealisticModelOffset().appendYawRotation(-Math.PI / 2.0);
   }
}
