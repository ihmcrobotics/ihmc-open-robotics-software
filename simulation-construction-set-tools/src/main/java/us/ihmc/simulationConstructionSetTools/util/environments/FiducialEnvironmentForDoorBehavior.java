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

      Vector3D position = new Vector3D(2.0, 0.0, 1.14);
      CombinedTerrainObject3D fiducualTerrainObject = DefaultCommonAvatarEnvironment.addFlatFiducial(0.2032, position, 0.0, Fiducial.FIDUCIAL50);
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
