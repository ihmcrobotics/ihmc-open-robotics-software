package us.ihmc.simulationConstructionSetTools.util.environments;


import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;

public class SimpleBoxEnvironment extends CombinedTerrainObject3D
{
   private static final double DEFAULT_FIELD_LENGTH = 150.0;

   private double FIELD_LENGTH, FIELD_HEIGHT, FIELD_WIDTH;

   public SimpleBoxEnvironment(double fieldLength)
   {
      super("SimpleBoxEnvironment");

      FIELD_LENGTH = fieldLength;
      FIELD_HEIGHT = FIELD_LENGTH * 0.01;
      FIELD_WIDTH = FIELD_LENGTH * 0.5;

      // Box 1
      this.addBox(0.0, 0.0, 4.0, 4.0, 0.0, 1.0);

      // Box 2
      this.addBox(1.0, 1.0, 3.0, 3.0, 1.0, 3.0);

   }

}
