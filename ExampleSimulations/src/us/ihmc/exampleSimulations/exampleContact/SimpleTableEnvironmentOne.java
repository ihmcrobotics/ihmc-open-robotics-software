package us.ihmc.exampleSimulations.exampleContact;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;

public class SimpleTableEnvironmentOne extends CombinedTerrainObject3D
{
   private static final double DEFAULT_FIELD_LENGTH = 150.0;

   private double FIELD_LENGTH, FIELD_HEIGHT, FIELD_WIDTH;

   public SimpleTableEnvironmentOne(double fieldLength)
   {
      super("SimpleTableEnvironmentOne");

      FIELD_LENGTH = fieldLength;
      FIELD_HEIGHT = FIELD_LENGTH * 0.01;
      FIELD_WIDTH = FIELD_LENGTH * 0.5;

      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.setRotationEulerAndZeroTranslation(new Vector3D(0.0, 0.0, Math.toRadians(45.0)));
      configuration.setTranslation(new Vector3D(4.0, 4.0, 0.7));
      this.addRotatableTable(configuration, 4.0, 2.0, 1.6, 0.1);

      // Table 1
      this.addTable(-1, -1, 3, 1, 1.3, 1.4);

      // Table 2
//      this.addTable(-0.5, -0.5, 0.5, 0.5, 2.4, 2.6);

   }
   
}
