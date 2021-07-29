package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.gdx.lighting.GDXPointLight;
import us.ihmc.gdx.lighting.GDXShadowManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

import java.util.concurrent.atomic.AtomicInteger;

public class GDXPointLightObject extends GDXEnvironmentObject
{
   private static final AtomicInteger INDEX = new AtomicInteger();

   private final GDXShadowManager manager;
   private final GDXPointLight light;

   public GDXPointLightObject(GDXShadowManager manager)
   {
      this.manager = manager;
      this.light = new GDXPointLight(Vector3.Zero);
      manager.addLight(light);
      manager.update();

      Model model = GDXModelPrimitives.buildModel(meshBuilder -> meshBuilder.addSphere(0.1f, Color.YELLOW), "pointModel");

      RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
      Box3D collisionBox = new Box3D(0.1f, 0.1f, 0.1f);

      Sphere3D boundingSphere = new Sphere3D(collisionBox.getSize().length() / 2.0);

      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilder ->
                                                             {
                                                                Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
                                                                meshBuilder.addSphere(0.11f, color);
                                                             }, "collisionModel" + INDEX.getAndIncrement());
      collisionGraphic.materials.get(0).set(new BlendingAttribute(true, 0.4f));
      RigidBodyTransform wholeThingOffset = new RigidBodyTransform();
      create(model, collisionShapeOffset, wholeThingOffset, boundingSphere, collisionBox, collisionBox::isPointInside, collisionGraphic);
   }

   @Override
   protected void updateRenderablesPoses()
   {
      super.updateRenderablesPoses();

      Tuple3DReadOnly tuple = this.getObjectTransform().getTranslation();
      light.setPosition(new Vector3(tuple.getX32(), tuple.getY32(), tuple.getZ32()));
      light.update();
   }

   @Override
   public GDXPointLightObject duplicate()
   {
      return new GDXPointLightObject(manager);
   }
}
