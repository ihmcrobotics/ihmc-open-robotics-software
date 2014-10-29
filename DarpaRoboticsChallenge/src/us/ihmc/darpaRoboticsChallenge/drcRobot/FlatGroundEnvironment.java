package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.List;

import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject3D;

public class FlatGroundEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D flatGround;

   public FlatGroundEnvironment()
   {
      flatGround = new CombinedTerrainObject3D("FlatGround");
      flatGround.addBox(-20.0, -20.0, 20.0, 20.0, -0.03, 0.0);
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return flatGround;
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
