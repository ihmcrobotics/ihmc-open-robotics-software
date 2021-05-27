package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

import java.util.concurrent.atomic.AtomicInteger;

public class GDXDoorOnlyObject extends GDXEnvironmentObject
{
   private static final AtomicInteger INDEX = new AtomicInteger();

   public GDXDoorOnlyObject()
   {
      Model realisticModel = GDXModelLoader.loadG3DModel("door/DoorOnly.g3dj");

      double heightZ = 2.0447; // these were measured in blender
      double widthY = 0.9144;
      double lengthX = 0.0508;
      Vector3D collisionShapeOffset = new Vector3D(lengthX / 2.0, widthY / 2.0, heightZ / 2.0);

      Box3D collisionBox = new Box3D(lengthX, widthY, heightZ);

      Sphere3D boundingSphere = new Sphere3D(collisionBox.getSize().length() / 2.0);
      boundingSphere.getPosition().set(collisionShapeOffset);

      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox((float) lengthX, (float) widthY, (float) heightZ, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
      }, "collisionModel" + INDEX.getAndIncrement());
      collisionGraphic.materials.get(0).set(new BlendingAttribute(true, 0.4f));

      collisionBox.getPose().getTranslation().set(collisionShapeOffset);

      create(realisticModel, collisionShapeOffset, boundingSphere, collisionBox, collisionBox::isPointInside, collisionGraphic);
   }

   @Override
   public GDXDoorOnlyObject duplicate()
   {
      return new GDXDoorOnlyObject();
   }
}
