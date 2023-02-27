package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class RDXLeftJerseyBarrierObject extends RDXEnvironmentObject
{
   public static final String NAME = "Left Jersey Barrier";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXLeftJerseyBarrierObject.class);

   public RDXLeftJerseyBarrierObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/jerseyBarrier/BarrierLeft.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 1.055;
      double sizeY = 0.46;
      double sizeZ = 0.809;
      setMass(20.0f);
      getCollisionShapeOffset().getTranslation().add(sizeX / 2.0 - 0.08 , 0, sizeZ / 2.0);
      getBoundingSphere().setRadius(5.0);
      getBoundingSphere().getPosition().add(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
     setCollisionModel(meshBuilder ->
                        {
                           Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
                           meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
                           meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
                        });
      setCollisionGeometryObject(collisionBox);
   }


}
