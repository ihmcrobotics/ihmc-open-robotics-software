package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.AffineTransformDefinition;
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
   private final AffineTransformDefinition originTransform;
   private final YawPitchRollTransformDefinition originPose;

   public LabFloorDefinition()
   {
      double sizeX = 20.0;
      double sizeY = 20.0;
      double sizeZ = 1.0;

      Box3DDefinition box3DDefinition = new Box3DDefinition();
      box3DDefinition.setSize(sizeX, sizeY, sizeZ);

      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/labFloor/LabFloor.g3dj");
      originTransform = new AffineTransformDefinition(new YawPitchRoll(), new Point3D());
      getVisualDefinitions().add(new VisualDefinition(originTransform, modelFileGeometryDefinition, new MaterialDefinition(ColorDefinitions.White())));
      originPose = new YawPitchRollTransformDefinition(0.0, 0.0, -sizeZ / 2.0);
      getCollisionShapeDefinitions().add(new CollisionShapeDefinition(originPose, box3DDefinition));
   }

   public void translate(double x, double y, double z)
   {
      originTransform.getTranslation().add(x, y, z);
      originPose.getTranslation().add(x, y, z);
   }
}
