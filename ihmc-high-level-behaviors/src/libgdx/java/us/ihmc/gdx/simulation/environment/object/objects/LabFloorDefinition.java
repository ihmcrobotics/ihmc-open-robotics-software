package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class LabFloorDefinition extends TerrainObjectDefinition
{
   private final YawPitchRollTransformDefinition originPose = new YawPitchRollTransformDefinition();

   public LabFloorDefinition()
   {
      double sizeX = 10.0;
      double sizeY = 10.0;
      double sizeZ = 1.0;

      Box3DDefinition box3DDefinition = new Box3DDefinition();
      box3DDefinition.setSize(sizeX, sizeY, sizeZ);

      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/labFloor/LabFloor.g3dj");
      getVisualDefinitions().add(new VisualDefinition(modelFileGeometryDefinition, new MaterialDefinition(ColorDefinitions.White())));
      getCollisionShapeDefinitions().add(new CollisionShapeDefinition(originPose, box3DDefinition));
   }
}
