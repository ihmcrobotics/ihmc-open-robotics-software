package us.ihmc.perception.sceneGraph.multiBodies.door;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class DoorFrameDefinition extends RigidBodyDefinition
{
   private final boolean ENABLE_SUPPORT_BOARD_COLLISIONS = true;

   public DoorFrameDefinition()
   {
      super("frameBody");

      double sizeX = 0.0889;
      double sizeY = 0.0889;
      double sizeZ = 2.159;

      setMass(300.0f);
      double radiusOfGyrationPercent = 0.8;
      setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(getMass(),
                                                                           radiusOfGyrationPercent * sizeX,
                                                                           radiusOfGyrationPercent * sizeY,
                                                                           radiusOfGyrationPercent * sizeZ));

      Vector3D centerOfMassInModelFrame = new Vector3D(0.0, (0.939887 / 2.0) - 0.006046, 0.1);
      getInertiaPose().getTranslation().set(centerOfMassInModelFrame);
      getInertiaPose().getRotation().setToZero();

      VisualDefinition frameModelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition(DoorSceneNodeDefinitions.DOOR_FRAME_VISUAL_MODEL_FILE_PATH);
      frameModelVisualDefinition.setGeometryDefinition(geometryDefinition);
      addVisualDefinition(frameModelVisualDefinition);

      Point3D latchSidePostOffset = new Point3D();
      latchSidePostOffset.add(0.0, -0.006064, 0.0);
      latchSidePostOffset.add(0.0, -0.0889 / 2.0, 2.159 / 2.0);
      addCollisionShape(sizeX, sizeY, sizeZ, latchSidePostOffset);

      if (ENABLE_SUPPORT_BOARD_COLLISIONS)
      {
         Point3D hingeSideSupportBoardOffset = new Point3D();
         hingeSideSupportBoardOffset.add(0.0, -0.006064, 0.0);
         hingeSideSupportBoardOffset.add(0.0, -0.0889, 0.0);
         hingeSideSupportBoardOffset.add(0.0, -0.0381 / 2.0, 0.0889 / 2.0f);
         addCollisionShape(1.016, 0.0381, 0.0889, hingeSideSupportBoardOffset);
      }

      Point3D hingeSidePostOffset = new Point3D();
      hingeSidePostOffset.add(0.0, 0.93415, 0.0);
      hingeSidePostOffset.add(0.0, 0.0889 / 2.0, 2.159 / 2.0);
      addCollisionShape(sizeX, sizeY, sizeZ, hingeSidePostOffset);

      if (ENABLE_SUPPORT_BOARD_COLLISIONS)
      {
         // shortened: 0.881195 long
         // short edge 0.372505 from center
         // scootch 0.0680925
         Point3D latchSideSupportBoardOffset = new Point3D();
         latchSideSupportBoardOffset.add(0.0, 0.93415, 0.0);
         latchSideSupportBoardOffset.add(0.0, 0.0889, 0.0);
         latchSideSupportBoardOffset.add(0.0680925, 0.0381 / 2.0, 0.0889 / 2.0f);
//         addCollisionShape(1.016, 0.0381, 0.0889, latchSideSupportBoardOffset);
         addCollisionShape(0.881195, 0.0381, 0.0889, latchSideSupportBoardOffset);
      }
   }

   private void addCollisionShape(double sizeX, double sizeY, double sizeZ, Point3D offset)
   {
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      Box3DDefinition boxCollisionDefinition = new Box3DDefinition();
      boxCollisionDefinition.setSize(sizeX, sizeY, sizeZ);
      collisionShapeDefinition.setGeometryDefinition(boxCollisionDefinition);
      collisionShapeDefinition.getOriginPose().set(new YawPitchRoll(), offset);
      addCollisionShapeDefinition(collisionShapeDefinition);
   }
}
