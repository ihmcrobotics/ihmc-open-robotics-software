package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class StepDownEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D stepGround;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public StepDownEnvironment(double stepEnd, double stepHeight)
   {
      stepGround = DefaultCommonAvatarEnvironment.setUpGround("Ground");
      stepGround.addBox(-0.5, -1.0, stepEnd, 1.0, stepHeight);
   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return stepGround;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return environmentRobots;
   }
}
