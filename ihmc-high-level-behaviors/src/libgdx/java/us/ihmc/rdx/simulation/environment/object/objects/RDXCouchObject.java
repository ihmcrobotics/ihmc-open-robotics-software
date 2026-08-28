package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXCouchObject extends RDXEnvironmentObject
{
   public static final String NAME = "Couch";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXCouchObject.class);

   public RDXCouchObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/couch/Couch.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 2.0;
      double sizeY = 0.8;
      double sizeZ = 0.6;
      setMass(60.0f);
      getCollisionShapeOffset().getTranslation().add(sizeX / 2.0 + 0.05, -sizeY / 2.0, sizeZ / 2.0);
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