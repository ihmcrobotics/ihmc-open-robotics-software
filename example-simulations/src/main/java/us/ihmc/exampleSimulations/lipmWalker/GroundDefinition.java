package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class GroundDefinition extends TerrainObjectDefinition
{
   public GroundDefinition()
   {
      super();
      // flat ground
      RigidBodyTransform groundPose = new RigidBodyTransform();
      groundPose.getRotation();
      groundPose.appendTranslation(0.0, 0.0, -0.05);

      GeometryDefinition groundGeometryDefinition = new Box3DDefinition(100.0, 100.0, 0.1);
      ColorDefinition color = ColorDefinitions.DarkSlateGray();
      color.setAlpha(0.5);
      addVisualDefinition(new VisualDefinition(groundPose, groundGeometryDefinition, new MaterialDefinition(color)));
      addCollisionShapeDefinition(new CollisionShapeDefinition(groundPose, groundGeometryDefinition));
   }
}
