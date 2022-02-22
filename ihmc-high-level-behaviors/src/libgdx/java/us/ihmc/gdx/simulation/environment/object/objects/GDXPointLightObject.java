package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.lighting.GDXPointLight;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class GDXPointLightObject extends GDXEnvironmentObject
{
   public static final String NAME = "Point Light";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXPointLightObject.class);
   private final GDXPointLight light;

   public GDXPointLightObject()
   {
      super(NAME, FACTORY);
      light = new GDXPointLight();

      Model model = GDXModelPrimitives.buildModel(meshBuilder -> meshBuilder.addSphere(0.1f, Color.YELLOW), "pointModel");
      setRealisticModel(model);

      Box3D collisionBox = new Box3D(0.1f, 0.1f, 0.1f);

      getBoundingSphere().setRadius(collisionBox.getSize().length() / 2.0);

      setCollisionModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addSphere(0.11f, color);
      });
      setCollisionGeometryObject(collisionBox);
   }

   @Override
   public void updateRenderablesPoses()
   {
      super.updateRenderablesPoses();

      light.getPosition().set(getObjectTransform().getTranslation());
      light.update();
   }

   public GDXPointLight getLight()
   {
      return light;
   }
}
