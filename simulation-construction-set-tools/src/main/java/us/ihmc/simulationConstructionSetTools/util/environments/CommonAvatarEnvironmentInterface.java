package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public interface CommonAvatarEnvironmentInterface
{
   static final Vector3D defaultGravity = new Vector3D(0.0, 0.0, -9.81);

   public abstract TerrainObject3D getTerrainObject3D();

   public abstract List<? extends Robot> getEnvironmentRobots();

   public abstract void createAndSetContactControllerToARobot();

   public abstract void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints);

   public abstract void addSelectableListenerToSelectables(SelectableObjectListener selectedListener);

   default Vector3D getGravityVectorWorldFrame()
   {
      return defaultGravity;
   }
}
