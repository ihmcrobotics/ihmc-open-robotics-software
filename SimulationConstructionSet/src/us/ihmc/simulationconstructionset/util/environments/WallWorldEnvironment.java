package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class WallWorldEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final double wallMinY, wallMaxY;
   
   public WallWorldEnvironment(double wallMinY, double wallMaxY)
   {
      this.wallMinY = wallMinY;
      this.wallMaxY = wallMaxY;
      
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      combinedTerrainObject.addTerrainObject(setupWallSlightlyInFront("WallInFront"));
   }
   
   private CombinedTerrainObject3D setupWallSlightlyInFront(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      double xStart = 0.6;
      
      AppearanceDefinition appearance = YoAppearance.Green();
      appearance.setTransparency(0.25);
      
      combinedTerrainObject.addBox(xStart, wallMinY, xStart + 0.1, wallMaxY, 0.5, 1.8, appearance);
      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-0.18, wallMinY, 0.2, wallMaxY, -0.05, 0.0, YoAppearance.Gold());
      combinedTerrainObject.addBox(-0.5, wallMinY - 1.5, 0.5, wallMinY, -0.05, 0.0, YoAppearance.Gold());
      combinedTerrainObject.addBox(-0.5, wallMaxY, 0.5, wallMaxY + 1.5, -0.05, 0.0, YoAppearance.Gold());

//    URL fileURL = DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
//      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");
//
//      Transform3D location = new Transform3D();
//      location.setTranslation(new Vector3d(0, 0, -0.5));
//
//      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, 45, 45, 1), texture);
//      combinedTerrainObject.addTerrainObject(newBox);
//      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3d(location, 200, 200, 0.75), YoAppearance.DarkGray());
//      combinedTerrainObject.addTerrainObject(newBox2);

      return combinedTerrainObject;
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub
      
   }

}
