package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXLabFloorObject extends RDXEnvironmentObject
{
   public static final String NAME = "Lab Floor";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXLabFloorObject.class);

   private final Box3D collisionBox;
   private final Point3D secondIntersection = new Point3D();

   public RDXLabFloorObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/labFloor/LabFloor.g3dj");
      setRealisticModel(realisticModel);

      // LabFloor.g3dj is a unit plane scaled to 20 x 20 m
      double sizeX = 20.0;
      double sizeY = 20.0;
      double sizeZ = 0.01;
      setMass(10000.0f);
      getBoundingSphere().setRadius(0.5 * Math.hypot(sizeX, sizeY));
      getCollisionShapeOffset().getTranslation().setZ(-0.5 * sizeZ);
      collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
   }

   /** Exact ray-box test; the default sampling test misses a box this thin. */
   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      return collisionBox.intersectionWith(pickRay, intersectionToPack, secondIntersection) > 0;
   }

   @Override
   public void setSelected(boolean selected)
   {
      setRawIsSelected(selected);
   }
}
