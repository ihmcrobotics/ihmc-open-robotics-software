package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public interface CommonAvatarEnvironmentInterface
{
   public abstract TerrainObject3D getTerrainObject3D();

   public abstract List<? extends Robot> getEnvironmentRobots();

   public abstract void createAndSetContactControllerToARobot();

   public abstract void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints);

   public abstract void addSelectableListenerToSelectables(SelectableObjectListener selectedListener);
}
