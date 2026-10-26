package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class FlatGroundEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D flatGround = new CombinedTerrainObject3D("flatGround");
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public FlatGroundEnvironment()
   {
      this(1, 1);
   }

   public FlatGroundEnvironment(int numberX, int numberY)
   {
      this(numberX, numberY, 0.0);
   }

   public FlatGroundEnvironment(double heightOffset)
   {
      this(9, 9, heightOffset);
   }


   public FlatGroundEnvironment(int numberX, int numberY, double heightOffset)
   {
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");
      double sizeXY = 50.0;

      for (int i = 0; i < numberX; i++)
      {
         for (int j = 0; j < numberY; j++)
         {
            Box3D box = new Box3D(sizeXY, sizeXY, 1.0);
            box.getPosition().setX(i * sizeXY - 0.5 * sizeXY * (numberX - 1));
            box.getPosition().setY(j * sizeXY - 0.5 * sizeXY * (numberY - 1));
            box.getPosition().setZ(-0.5 + heightOffset);
            flatGround.addTerrainObject(new RotatableBoxTerrainObject(box, texture));
         }
      }
   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return flatGround;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return environmentRobots;
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
