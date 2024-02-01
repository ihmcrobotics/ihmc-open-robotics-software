package us.ihmc.exampleSimulations.planarWalker.BWC;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class FlatGroundDefinition extends TerrainObjectDefinition
{
   public FlatGroundDefinition()
   {
      this(0.0);
   }

   public FlatGroundDefinition(double slopeAngle)
   {
      super();
      RigidBodyTransform originPose = new RigidBodyTransform();
      originPose.getRotation().setToPitchOrientation(slopeAngle);
      originPose.appendTranslation(0.0, 0.0, -0.25);

      GeometryDefinition groundGeometryDefinition = new Box3DDefinition(10000.0, 10000.0, 0.50);
      addVisualDefinition(new VisualDefinition(originPose, groundGeometryDefinition, new MaterialDefinition(ColorDefinitions.DeepSkyBlue())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(originPose, groundGeometryDefinition));
   }
}