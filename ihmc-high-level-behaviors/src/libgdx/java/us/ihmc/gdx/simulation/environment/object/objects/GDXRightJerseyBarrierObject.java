package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class GDXRightJerseyBarrierObject extends GDXEnvironmentObject
{
   public static final String NAME = "Right Jersey Barrier";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXRightJerseyBarrierObject.class);

   public  GDXRightJerseyBarrierObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.load("environmentObjects/jerseyBarrier/BarrierRight.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 0.92;
      double sizeY = 0.46;
      double sizeZ = 0.809;
      setMass(20.0f);
      getCollisionShapeOffset().getTranslation().add(-sizeX / 2.0 +.07, 0.01, sizeZ / 2.0);
      getBoundingSphere().setRadius(5.0);
      getBoundingSphere().getPosition().add(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionModel(meshBuilder ->
                        {
                           Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
                           meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
                           meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
                        });
      setCollisionGeometryObject(collisionBox);
   }


}
