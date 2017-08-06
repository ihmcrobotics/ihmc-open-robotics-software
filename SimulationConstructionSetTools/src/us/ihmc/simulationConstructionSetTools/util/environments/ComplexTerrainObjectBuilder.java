package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class ComplexTerrainObjectBuilder
{
   private final double startingLocationX, startingLocationY, courseYaw;
   private final CombinedTerrainObject3D complexTerrainObject;

   public ComplexTerrainObjectBuilder(String name, double startingLocationX, double startingLocationY, double courseYaw)
   {
      this.startingLocationX = startingLocationX;
      this.startingLocationY = startingLocationY;
      this.courseYaw = courseYaw;

      complexTerrainObject = new CombinedTerrainObject3D(name);
   }

   public void addTerrainObject(TerrainObject3D terrainObject3D)
   {

   }

   public CombinedTerrainObject3D getComplexTerrainObject()
   {
      return complexTerrainObject;
   }
}
