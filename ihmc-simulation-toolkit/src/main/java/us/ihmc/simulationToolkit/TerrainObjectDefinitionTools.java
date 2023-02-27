package us.ihmc.simulationToolkit;

import java.util.List;

import us.ihmc.robotModels.description.RobotDescriptionConverter;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class TerrainObjectDefinitionTools
{
   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment)
   {
      CollidableHelper collidableHelper = new CollidableHelper();
      String robotCollisionName = "robot";
      String terrainCollisionName = "terrain";
      return toTerrainObjectDefinition(environment, collidableHelper, robotCollisionName, terrainCollisionName);
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment,
                                                                   CollidableHelper collidableHelper,
                                                                   String terrainCollisionMask,
                                                                   String... interactableCollisionGroups)
   {
      return toTerrainObjectDefinition(environment.getTerrainObject3D(), collidableHelper, terrainCollisionMask, interactableCollisionGroups);
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(TerrainObject3D terrainObject3D,
                                                                   CollidableHelper collidableHelper,
                                                                   String terrainCollisionMask,
                                                                   String... interactableCollisionGroups)
   {
      return toTerrainObjectDefinition(terrainObject3D,
                                       collidableHelper.getCollisionMask(terrainCollisionMask),
                                       collidableHelper.createCollisionGroup(interactableCollisionGroups));
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment, long terrainCollisionMask, long collisionGroup)
   {
      return toTerrainObjectDefinition(environment.getTerrainObject3D(), terrainCollisionMask, collisionGroup);
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(TerrainObject3D terrainObject3D, long terrainCollisionMask, long collisionGroup)
   {
      TerrainObjectDefinition output = new TerrainObjectDefinition();
      List<Collidable> collidables = ExperimentalSimulation.toCollidables(terrainCollisionMask, collisionGroup, terrainObject3D);

      for (Collidable collidable : collidables)
      {
         output.addCollisionShapeDefinition(RobotDefinitionTools.toCollisionShapeDefinition(collidable));
      }

      List<VisualDefinition> visualDefinitions = RobotDescriptionConverter.toVisualDefinitions(terrainObject3D.getLinkGraphics());
      visualDefinitions.forEach(output::addVisualDefinition);

      return output;
   }
}
