package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXOusterSensorObject extends GDXEnvironmentObject
{
   public static final String NAME = "Ouster Sensor";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXOusterSensorObject.class);

   public GDXOusterSensorObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.load("environmentObjects/ousterSensor/Ouster.g3dj");
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
