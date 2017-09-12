package us.ihmc.exampleSimulations.exampleContact;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;

public class ExampleTerrainWithTable extends CombinedTerrainObject3D
{
   public ExampleTerrainWithTable()
   {
      super("ExampleTerrainWithTable");
      
//      this.addTable(0.0, 0.0, 1.0, 0.5, 0.0, 1.0);
      
      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.setRotationEulerAndZeroTranslation(new Vector3D(0.0, 0.0, Math.PI/4.0));
      configuration.setTranslation(new Vector3D(3.0, 0.0, 0.6));
      
      this.addRotatableTable(configuration, 2.0, 1.0, 1.2, 0.05);
      this.addTable(-1.5, -1.0, 1.5, 1.0, 0.45, 0.5);

      this.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0);
   }
}
