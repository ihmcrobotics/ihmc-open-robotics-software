package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXWallObject extends RDXEnvironmentObject
{
   public static final String NAME = "Wall";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXWallObject.class);

   public RDXWallObject()
   {
      super(NAME, FACTORY);
      loadRealisticModel("environmentObjects/wall/wall.glb");

      double sizeX = 0.12;
      double sizeY = 2.0;
      double sizeZ = 2.2;
      setMass(500.0f);
      getCollisionShapeOffset().getTranslation().set(0.0, 0.0, sizeZ / 2.0);
      getBoundingSphere().setRadius(3.0);
      getBoundingSphere().getPosition().set(0.0, 0.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionModel(meshBuilder ->
                        {
                           Color color = LibGDXTools.toLibGDX(YoAppearance.DarkGray());
                           meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
                           meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color);
                        });
      setCollisionGeometryObject(collisionBox);
   }
}
