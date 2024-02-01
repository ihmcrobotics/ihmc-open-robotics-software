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

public class FlatGroundDefinition extends TerrainObjectDefinition
{
   public FlatGroundDefinition()
   {
      double sizeX = 45.0;
      double sizeY = 45.0;
      double sizeZ = 1.0;

      Box3DDefinition box3DDefinition = new Box3DDefinition();
      box3DDefinition.setSize(sizeX, sizeY, sizeZ);

//      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/flatGround/FlatGround.obj");
      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/flatGround/FlatGround.g3dj");
      AffineTransformDefinition visualOriginPose = new AffineTransformDefinition(new YawPitchRoll(), new Point3D());
      getVisualDefinitions().add(new VisualDefinition(visualOriginPose, modelFileGeometryDefinition, new MaterialDefinition(ColorDefinitions.White())));
      YawPitchRollTransformDefinition collisionShapeOriginPose = new YawPitchRollTransformDefinition(0.0, 0.0, -sizeZ / 2.0);
      getCollisionShapeDefinitions().add(new CollisionShapeDefinition(collisionShapeOriginPose, box3DDefinition));
   }

   public void translate(double x, double y, double z)
   {
      for (VisualDefinition visualDefinition : getVisualDefinitions())
      {
         visualDefinition.getOriginPose().getTranslation().add(x, y, z);
      }

      for (CollisionShapeDefinition collisionShapeDefinition : getCollisionShapeDefinitions())
      {
         collisionShapeDefinition.getOriginPose().getTranslation().add(x, y, z);
      }
   }
}
