package us.ihmc.exampleSimulations.exampleContact;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;

public class ExampleTerrainWithTable extends CombinedTerrainObject3D
{
   public ExampleTerrainWithTable()
   {
      super("ExampleTerrainWithTable");
      
//      this.addTable(0.0, 0.0, 1.0, 0.5, 0.0, 1.0);
      
      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.setEuler(new Vector3d(0.0, 0.0, Math.PI/4.0));
      configuration.setTranslation(new Vector3d(3.0, 0.0, 0.6));
      
      this.addRotatableTable(configuration, 2.0, 1.0, 1.2, 0.05);
      this.addTable(-1.5, -1.0, 1.5, 1.0, 0.45, 0.5);

      this.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0);
   }
}
