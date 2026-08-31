package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXTrashCanObject extends RDXEnvironmentObject
{
   public static final String NAME = "TrashCan";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXTrashCanObject.class);

   public RDXTrashCanObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/trashCan/TrashCan.g3dj");
      setRealisticModel(realisticModel);

      // Mesh bounds are roughly 0.64 x 1.0 x 0.74 and nearly centered on the origin. The old
      // 0.2^3 box was offset, so click-to-select / the gizmo outline missed the visible can.
      double sizeX = 0.65;
      double sizeY = 0.65;
      double sizeZ = 0.75;
      setMass(10.0f);
      getCollisionShapeOffset().getTranslation().set(0.0, 0.0, 0.0);
      getBoundingSphere().setRadius(0.85);
      getBoundingSphere().getPosition().set(0.0, 0.0, 0.0);
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
