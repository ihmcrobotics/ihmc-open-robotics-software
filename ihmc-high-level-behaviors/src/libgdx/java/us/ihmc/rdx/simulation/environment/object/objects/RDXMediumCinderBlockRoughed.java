package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXMediumCinderBlockRoughed extends RDXEnvironmentObject
{
   public static final String NAME = "Medium Cinder Block Roughed";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXMediumCinderBlockRoughed.class);

   public RDXMediumCinderBlockRoughed()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/mediumCinderBlockRoughed/MediumCinderBlockRoughed.g3dj");
      setRealisticModel(realisticModel);

      double height = 0.141535; // these were measured in blender
      double width = 0.188522;
      double length = 0.393001;
      setMass(6.0f);
      Box3D collisionBox = new Box3D(length, width, height);
      setCollisionGeometryObject(collisionBox);
      getBoundingSphere().setRadius(collisionBox.getSize().length() / 2.0);
//      setBtCollisionShape(new btBoxShape(new Vector3((float) length / 2.0f, (float) width / 2.0f, (float) height / 2.0f)));
   }
}
