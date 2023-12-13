package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class MeshTerrainEnvironment implements CommonAvatarEnvironmentInterface
{
   public enum TerrainDifficulty
   {
      EASY, MEDIUM, WORK_PLATFORM;
   }

   private final CombinedTerrainObject3D meshTerrain = new CombinedTerrainObject3D("meshTerrain");
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public MeshTerrainEnvironment()
   {
      this(MeshTerrainObjectFactory.createWalkwayObject());
   }

   public MeshTerrainEnvironment(TerrainObject3D... terrainObjects)
   {
      for (TerrainObject3D terrainObject : terrainObjects)
         meshTerrain.addTerrainObject(terrainObject);
   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return meshTerrain;
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