package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class CartRobotRacingEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final int numberOfPlates = 5;
   private final double plateSize = 2.0;
   private final double plateHeightGap = 0.3;
   private final double plateThickness = 0.1;

   public CartRobotRacingEnvironment(boolean useRampTerrain)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());

      for (int i = 0; i < numberOfPlates; i++)
      {
         combinedTerrainObject.addBox(-0.5 * plateSize + plateSize * i, -0.5 * plateSize, 0.5 * plateSize + plateSize * i, 0.5 * plateSize,
                                      -plateThickness - plateHeightGap * i, -plateHeightGap * i);
      }

      combinedTerrainObject.addSphere(0.5, 0.0, 0.0, 0.05, YoAppearance.AliceBlue());

      RigidBodyTransform cylinderObstacleTransform = new RigidBodyTransform();
      cylinderObstacleTransform.appendRollRotation(0.5 * Math.PI);

      combinedTerrainObject.addCylinder(cylinderObstacleTransform, 0.3, 0.05, YoAppearance.AliceBlue());
      
      if (useRampTerrain)
         combinedTerrainObject.addRamp(0.8, -0.7, 2.0, 0.7, 0.5, YoAppearance.AliceBlue());
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
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
