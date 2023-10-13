package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXPunchingBagObject extends RDXEnvironmentObject
{
   public static final String NAME = "PunchingBag";
   public static final String VISUAL_MODEL_FILE_PATH = "environmentObjects/punchingBag/punchingBag.g3dj";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXPunchingBagObject.class);

   public RDXPunchingBagObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load(VISUAL_MODEL_FILE_PATH);
      setRealisticModel(realisticModel);

      // TODO update collision box
      double size = 0.35;
      getBoundingSphere().setRadius(0.5);
      setMass(0.3f);
      Box3D collisionBox = new Box3D(1.05, 1.05, 1.8);
      setCollisionGeometryObject(collisionBox);
   }
}