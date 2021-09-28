package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;

public class TerrainObjectDefinitionTools
{
   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment,
                                                                   CollidableHelper collidableHelper,
                                                                   String terrainCollisionMask,
                                                                   String... interactableCollisionGroups)
   {
      return toTerrainObjectDefinition(environment,
                                       collidableHelper.getCollisionMask(terrainCollisionMask),
                                       collidableHelper.createCollisionGroup(interactableCollisionGroups));
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment, long terrainCollisionMask, long collisionGroup)
   {
      TerrainObjectDefinition output = new TerrainObjectDefinition();

      List<Collidable> collidables = ExperimentalSimulation.toCollidables(terrainCollisionMask, collisionGroup, environment);

      for (Collidable collidable : collidables)
      {
         output.addCollisionShapeDefinition(RobotDefinitionTools.toCollisionShapeDefinition(collidable));
      }

      List<VisualDefinition> visualDefinitions = RobotDefinitionTools.toVisualDefinitions(environment.getTerrainObject3D().getLinkGraphics());
      visualDefinitions.forEach(output::addVisualDefinition);

      return output;
   }
}
