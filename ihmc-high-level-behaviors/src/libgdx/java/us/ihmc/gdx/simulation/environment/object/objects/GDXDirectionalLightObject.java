package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.gdx.lighting.GDXDirectionalLight;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

import java.util.concurrent.atomic.AtomicInteger;

public class GDXDirectionalLightObject extends GDXEnvironmentObject
{
   private static final AtomicInteger INDEX = new AtomicInteger();

   private final GDXDirectionalLight light;

   public GDXDirectionalLightObject()
   {
      this.light = new GDXDirectionalLight();

      Model model = GDXModelPrimitives.buildModel(meshBuilder -> meshBuilder.addBox(0.2f, 0.2f, 0.05f, Color.YELLOW), "directionalModel");
      RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
      Box3D collisionBox = new Box3D(0.2f, 0.2f, 0.05f);

      Sphere3D boundingSphere = new Sphere3D(collisionBox.getSize().length() / 2.0);

      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox(0.21f, 0.21f, 0.06f, color);
      }, "collisionModel" + INDEX.getAndIncrement());
      collisionGraphic.materials.get(0).set(new BlendingAttribute(true, 0.4f));
      RigidBodyTransform wholeThingOffset = new RigidBodyTransform();
      create(model, collisionShapeOffset, wholeThingOffset, boundingSphere, collisionBox, collisionBox::isPointInside, collisionGraphic);
   }

   @Override
   protected void updateRenderablesPoses()
   {
      super.updateRenderablesPoses();

      Tuple3DReadOnly position = this.getObjectTransform().getTranslation();

      Vector3D rotation = new Vector3D();
      this.getObjectTransform().getRotation().getRotationVector(rotation);

      light.getPosition().set(position.getX32(), position.getY32(), position.getZ32());
      light.getDirection().set(rotation.getX32(), rotation.getY32(), rotation.getZ32());
      light.update();
   }

   @Override
   public GDXDirectionalLightObject duplicate()
   {
      return new GDXDirectionalLightObject();
   }

   public GDXDirectionalLight getLight()
   {
      return light;
   }
}
