package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public interface CommonAvatarEnvironmentInterface
{
   public abstract TerrainObject3D getTerrainObject3D();

   public default List<? extends Robot> getEnvironmentRobots()
   {
      return null;
   }

   public default void createAndSetContactControllerToARobot()
   {
   }

   public default void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   public default void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
