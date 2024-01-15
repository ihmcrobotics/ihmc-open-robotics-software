package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.lighting.RDXDirectionalLight;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class RDXDirectionalLightObject extends RDXEnvironmentObject
{
   public static final String NAME = "Directional Light";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXDirectionalLightObject.class);

   private final RDXDirectionalLight light;

   public RDXDirectionalLightObject()
   {
      super(NAME, FACTORY);
      this.light = new RDXDirectionalLight();

      Model model = RDXModelBuilder.buildModel(meshBuilder -> meshBuilder.addBox(0.2f, 0.2f, 0.05f, Color.YELLOW), "directionalModel");
      setRealisticModel(model);
      Box3D collisionBox = new Box3D(0.2f, 0.2f, 0.05f);

      getBoundingSphere().setRadius(collisionBox.getSize().length() / 2.0);

      setCollisionModel(meshBuilder ->
      {
         Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox(0.21f, 0.21f, 0.06f, color);
      });
      setCollisionGeometryObject(collisionBox);
   }

   @Override
   public void updateRenderablesPoses()
   {
      super.updateRenderablesPoses();

      Tuple3DReadOnly position = this.getObjectTransform().getTranslation();

      Vector3D rotation = new Vector3D();
      this.getObjectTransform().getRotation().getRotationVector(rotation);

      light.getPosition().set(position.getX32(), position.getY32(), position.getZ32());
      light.getDirection().set(rotation.getX32(), rotation.getY32(), rotation.getZ32());
      light.update();
   }

   public RDXDirectionalLight getLight()
   {
      return light;
   }
}
