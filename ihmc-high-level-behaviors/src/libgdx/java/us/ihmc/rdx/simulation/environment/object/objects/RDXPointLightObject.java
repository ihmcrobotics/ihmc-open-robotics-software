package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.lighting.RDXPointLight;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class RDXPointLightObject extends RDXEnvironmentObject
{
   public static final String NAME = "Point Light";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXPointLightObject.class);
   private final RDXPointLight light;

   public RDXPointLightObject()
   {
      super(NAME, FACTORY);
      light = new RDXPointLight();

      Model model = RDXModelBuilder.buildModel(meshBuilder -> meshBuilder.addSphere(0.1f, Color.YELLOW), "pointModel");
      setRealisticModel(model);

      Box3D collisionBox = new Box3D(0.1f, 0.1f, 0.1f);

      getBoundingSphere().setRadius(collisionBox.getSize().length() / 2.0);

      setCollisionModel(meshBuilder ->
      {
         Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
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

   public RDXPointLight getLight()
   {
      return light;
   }
}
