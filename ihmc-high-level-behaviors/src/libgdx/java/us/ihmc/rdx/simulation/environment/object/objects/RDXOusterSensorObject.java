package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXOusterSensorObject extends RDXEnvironmentObject
{
   public static final String NAME = "Ouster Sensor";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXOusterSensorObject.class);

   public RDXOusterSensorObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/ousterSensor/Ouster.g3dj");
      setRealisticModel(realisticModel);

      getRealisticModelOffset().setRotationEulerAndZeroTranslation(EuclidCoreTools.TwoPI / 2.0f, 0.0f, 0.0f);

      getBoundingSphere().setRadius(0.2);
      double sizeX = 0.075;
      double sizeY = 0.075;
      double sizeZ = 0.08;
      setMass(0.3f);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      getCollisionShapeOffset().getTranslation().addZ(0.03f);
   }
}
