package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXPalletObject extends RDXEnvironmentObject
{
   public static final String NAME = "Pallet";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXPalletObject.class);

   public RDXPalletObject()
   {
      super(NAME, FACTORY);
      loadRealisticModel("environmentObjects/pallet/Pallet.g3dj");

      double sizeX = 1.21;
      double sizeY = 1.013;
      double sizeZ = 0.155;
      setMass(30.0f);
      Sphere3D boundingSphere = new Sphere3D(0.7);
      getBoundingSphere().setRadius(0.7);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
//      setBtCollisionShape(new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f)));
   }
}
