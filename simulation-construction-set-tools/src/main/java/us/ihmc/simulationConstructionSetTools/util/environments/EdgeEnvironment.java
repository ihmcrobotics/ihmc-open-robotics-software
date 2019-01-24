package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class EdgeEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D flatGround;

   public EdgeEnvironment()
   {
      flatGround = DefaultCommonAvatarEnvironment.setUpGround("Ground");
      flatGround.addBox(0.0, -1.0, 0.5, 1.0, 0.5, YoAppearance.Beige());
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return flatGround;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
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
