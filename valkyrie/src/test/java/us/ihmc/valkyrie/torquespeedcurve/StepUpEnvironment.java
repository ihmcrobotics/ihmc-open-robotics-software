package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class StepUpEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D stepGround;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public StepUpEnvironment(double stepStart, double stepHeight)
   {
      stepGround = DefaultCommonAvatarEnvironment.setUpGround("Ground");
      stepGround.addBox(stepStart, -1.0, stepStart + 4.0, 1.0, stepHeight);
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
