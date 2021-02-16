package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.robotics.physics.Collidable;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition.MaterialDefinition;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;

public class TerrainObjectDefinitionTools
{
   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment)
   {
      TerrainObjectDefinition output = new TerrainObjectDefinition();

      List<Collidable> collidables = ExperimentalSimulation.toCollidables(-1, -1, environment);

      for (Collidable collidable : collidables)
      {
         CollisionShapeDefinition collisionShapeDefinition = RobotDefinitionTools.toCollisionShapeDefinition(collidable);
         output.addCollisionShapeDefinition(collisionShapeDefinition);
         output.addVisualDefinition(new VisualDefinition(collisionShapeDefinition.getOriginPose(),
                                                         collisionShapeDefinition.getGeometryDefinition(),
                                                         new MaterialDefinition(ColorDefinitions.Maroon())));
      }

      return output;
   }
}
