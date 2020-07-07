package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.List;

public class FiducialEnvironmentForDoorBehavior implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   public FiducialEnvironmentForDoorBehavior()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      Vector3D position = new Vector3D(0.025875, 0.68183125, 1.1414125);
      CombinedTerrainObject3D fiducualTerrainObject = DefaultCommonAvatarEnvironment.addFlatFiducial(0.2032, position, 0.0, Fiducial.FIDUCIAL150);
      combinedTerrainObject.addTerrainObject(fiducualTerrainObject);
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
