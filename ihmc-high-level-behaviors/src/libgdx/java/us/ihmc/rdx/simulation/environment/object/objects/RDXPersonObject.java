package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.behaviors.simulation.RigidBodySceneObjectDefinitions;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXPersonObject extends RDXEnvironmentObject
{
   public static final String NAME = "Person";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXPersonObject.class);

   public RDXPersonObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load(RigidBodySceneObjectDefinitions.PERSON_VISUAL_MODEL_FILE_PATH);
      setRealisticModel(realisticModel);

      double sizeX = 0.5;
      double sizeY = 1.0;
      double sizeZ = 1.8;
      setMass(80.0f);
      getCollisionShapeOffset().getTranslation().add(0.0, 0.0, sizeZ / 2.0);
      getBoundingSphere().setRadius(5.0);
      getBoundingSphere().getPosition().add(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionModel(meshBuilder ->
                        {
                           Color color = LibGDXTools.toLibGDX(YoAppearance.DarkGray());
                           meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
                           meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
                        });
      setCollisionGeometryObject(collisionBox);
   }
}