package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObjectParameters;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class MeshTerrainEnvironment implements CommonAvatarEnvironmentInterface
{
   public enum TerrainDifficulty
   {
      EASY, MEDIUM, HARD;
   }

   private final CombinedTerrainObject3D meshTerrain = new CombinedTerrainObject3D("meshTerrain");
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public MeshTerrainEnvironment()
   {
      this(TerrainDifficulty.EASY);
   }

   public MeshTerrainEnvironment(TerrainDifficulty terrainDifficulty)
   {
      if (terrainDifficulty==TerrainDifficulty.EASY)
      {
         RigidBodyTransform configuration = new RigidBodyTransform();
         Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion orientation = new Quaternion(-Math.PI / 2.0, 0.0, 0.0);
         configuration.set(orientation,translation);
         
         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/walkway/walkway.obj",configuration);
         meshTerrain.addTerrainObject(meshTerrainObject);
      }
      else if (terrainDifficulty==TerrainDifficulty.MEDIUM)
      {
         RigidBodyTransform configuration = new RigidBodyTransform();
         Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion orientation = new Quaternion(-Math.PI / 2.0, 0.0, 0.0);
         configuration.set(orientation,translation);
         
         MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters();
         parameters.setMaxNoOfHulls(1024);
         parameters.setMaxNoOfVertices(256);
         parameters.setVoxelResolution(500000);
         parameters.setShowDecomposedMeshGraphics(false);
         parameters.setShowUndecomposedMeshGraphics(true);
         
         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/roughTerrain/roughTerrain.obj",parameters, configuration);
         meshTerrain.addTerrainObject(meshTerrainObject);
      }
         
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