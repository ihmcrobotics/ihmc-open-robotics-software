package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObjectParameters;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
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
      this(TerrainDifficulty.EASY);
   }

   public MeshTerrainEnvironment(TerrainDifficulty terrainDifficulty)
   {
      if (terrainDifficulty == TerrainDifficulty.EASY)
      {
         RigidBodyTransform configuration = new RigidBodyTransform();
         Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion orientation = new Quaternion(-Math.PI / 2.0, 0.0, 0.0);
         configuration.set(orientation, translation);

         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/walkway/walkway.obj", configuration);
         meshTerrain.addTerrainObject(meshTerrainObject);
      }
      else if (terrainDifficulty == TerrainDifficulty.MEDIUM)
      {
         RigidBodyTransform configuration = new RigidBodyTransform();
         Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion orientation = new Quaternion(-Math.PI / 2.0, 0.0, 0.0);
         configuration.set(orientation, translation);

         MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters();
         parameters.setMaxNoOfHulls(1024);
         parameters.setMaxNoOfVertices(256);
         parameters.setVoxelResolution(500000);
         parameters.setShowDecomposedMeshGraphics(false);
         parameters.setShowUndecomposedMeshGraphics(true);

         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/roughTerrain/roughTerrain.obj", parameters, configuration);
         meshTerrain.addTerrainObject(meshTerrainObject);
      }
      else if (terrainDifficulty == TerrainDifficulty.WORK_PLATFORM)
      {
         RigidBodyTransform configuration = new RigidBodyTransform();
         Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion orientation = new Quaternion(-Math.PI / 2.0, 0.0, 0.0);
         configuration.set(orientation, translation);

         MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters();
         parameters.setMaxNoOfHulls(100);
         parameters.setMaxNoOfVertices(20);
         parameters.setVoxelResolution(8000000);
         parameters.setShowDecomposedMeshGraphics(false);
         parameters.setShowUndecomposedMeshGraphics(true);

         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("C:/Users/Khizar/Downloads/workPlatForm/workPlatForm.obj", parameters, configuration);
         meshTerrain.addTerrainObject(meshTerrainObject);

         // Adding FlatGround
         YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");
         double sizeXY = 50.0;
         Box3D box = new Box3D(sizeXY, sizeXY, 1.0);
         box.getPosition().setX(0.0);
         box.getPosition().setY(0.0);
         box.getPosition().setZ(-0.5);
         meshTerrain.addTerrainObject(new RotatableBoxTerrainObject(box, texture));
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