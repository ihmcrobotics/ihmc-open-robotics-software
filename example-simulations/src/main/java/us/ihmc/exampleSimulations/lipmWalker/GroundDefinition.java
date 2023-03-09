package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class GroundDefinition extends TerrainObjectDefinition
{
   public GroundDefinition()
   {
      super();
      // flat ground
      GeometryDefinition groundGeometryDefinition = new Box3DDefinition(10.0, 10.0, 0.1);
      addVisualDefinition(new VisualDefinition(new RigidBodyTransform(), groundGeometryDefinition, new MaterialDefinition(ColorDefinitions.DarkSlateGray())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(), groundGeometryDefinition));
   }
}
