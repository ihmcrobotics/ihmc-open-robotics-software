package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.List;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject3D;

public interface CommonAvatarEnvironmentInterface
{
   public abstract TerrainObject3D getTerrainObject3D();

   public abstract List<? extends Robot> getEnvironmentRobots();

   public abstract void createAndSetContactControllerToARobot();

   public abstract void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints);

   public abstract void addSelectableListenerToSelectables(SelectableObjectListener selectedListener);
}
