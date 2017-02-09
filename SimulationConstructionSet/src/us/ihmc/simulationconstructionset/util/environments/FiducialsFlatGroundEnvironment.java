package us.ihmc.simulationconstructionset.util.environments;

import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class FiducialsFlatGroundEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   public FiducialsFlatGroundEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      double angle = 0.0;
      double radius = 3.0;
      for (Fiducial fiducial : Fiducial.values)
      {
         Vector3d position = new Vector3d(radius * Math.cos(angle), radius * Math.sin(angle), 1.7);
         CombinedTerrainObject3D fiducualTerrainObject = DefaultCommonAvatarEnvironment.addFiducial(position, angle, fiducial);
         combinedTerrainObject.addTerrainObject(fiducualTerrainObject);

         angle += 2.0 * Math.PI / Fiducial.values.length;
      }
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
