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

public class DownwardSlopedGroundDefinition extends TerrainObjectDefinition
{
   public DownwardSlopedGroundDefinition()
   {
      double sizeX = 45.0;
      double sizeY = 45.0;
      double sizeZ = 1.0;

      Box3DDefinition box3DDefinition = new Box3DDefinition();
      box3DDefinition.setSize(sizeX, sizeY, sizeZ);

      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/flatGround/FlatGround.g3dj");
      AffineTransformDefinition visualOriginPose = new AffineTransformDefinition(new YawPitchRoll(), new Point3D());
      getVisualDefinitions().add(new VisualDefinition(visualOriginPose, modelFileGeometryDefinition, new MaterialDefinition(ColorDefinitions.White())));
      YawPitchRollTransformDefinition collisionShapeOriginPose = new YawPitchRollTransformDefinition(0.0, 0.0, -sizeZ / 2.0);
      getCollisionShapeDefinitions().add(new CollisionShapeDefinition(collisionShapeOriginPose, box3DDefinition));

      translate(-20.0, 0.0, 0.0);

      Box3DDefinition box3DDefinition2 = new Box3DDefinition();
      box3DDefinition2.setSize(sizeX, sizeY, sizeZ);

      ModelFileGeometryDefinition modelFileGeometryDefinition2 = new ModelFileGeometryDefinition("environmentObjects/flatGround/FlatGround.g3dj");
      AffineTransformDefinition visualOriginPose3 = new AffineTransformDefinition(new YawPitchRoll(), new Point3D());
      VisualDefinition visualDefinition = new VisualDefinition(visualOriginPose3, modelFileGeometryDefinition2, new MaterialDefinition(ColorDefinitions.White()));
      getVisualDefinitions().add(visualDefinition);
      YawPitchRollTransformDefinition collisionShapeOriginPose4 = new YawPitchRollTransformDefinition(0.0, 0.0, -sizeZ / 2.0);
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(collisionShapeOriginPose4, box3DDefinition2);
      getCollisionShapeDefinitions().add(collisionShapeDefinition);

      double z = -1.2;
      visualDefinition.getOriginPose().getTranslation().add(25.0, 0.0, z);
      collisionShapeDefinition.getOriginPose().getTranslation().add(25.0, 0.0, z);
      double angdeg = 3;
      visualDefinition.getOriginPose().getLinearTransform().appendPitchRotation(Math.toRadians(angdeg));
      collisionShapeDefinition.getOriginPose().getRotation().appendPitchRotation(Math.toRadians(angdeg));
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
