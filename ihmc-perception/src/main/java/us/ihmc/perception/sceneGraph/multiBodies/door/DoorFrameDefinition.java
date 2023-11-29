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

import static us.ihmc.perception.sceneGraph.multiBodies.door.DoorModelParameters.*;

public class DoorFrameDefinition extends RigidBodyDefinition
{
   private final boolean ENABLE_SUPPORT_BOARD_COLLISIONS = true;

   public DoorFrameDefinition()
   {
      super("frameBody");

      double pillarSizeX = DOOR_FRAME_PILLAR_SIZE_X;
      double pillarSizeY = DOOR_FRAME_PILLAR_SIZE_X;
      double pillarSizeZ = DOOR_FRAME_PILLAR_SIZE_Z;

      // Like a bunch of cinder blocks
      float heavyMass = 300.0f;
      setMass(heavyMass);
      double radiusOfGyrationPercent = 0.8;
      setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(getMass(),
                                                                           radiusOfGyrationPercent * pillarSizeX,
                                                                           radiusOfGyrationPercent * pillarSizeY,
                                                                           radiusOfGyrationPercent * pillarSizeZ));

      double lowCoMForStabilityZ = 0.05;
      Vector3D centerOfMassInModelFrame = new Vector3D(0.0, (DOOR_PANEL_WIDTH / 2.0) - DOOR_FRAME_HINGE_OFFSET, lowCoMForStabilityZ);
      getInertiaPose().getTranslation().set(centerOfMassInModelFrame);
      getInertiaPose().getRotation().setToZero();

      VisualDefinition frameModelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition(DoorSceneNodeDefinitions.DOOR_FRAME_VISUAL_MODEL_FILE_PATH);
      frameModelVisualDefinition.setGeometryDefinition(geometryDefinition);
      addVisualDefinition(frameModelVisualDefinition);

      Point3D latchSidePostOffset = new Point3D();
      latchSidePostOffset.add(0.0, -DOOR_FRAME_HINGE_OFFSET, 0.0);
      latchSidePostOffset.add(0.0, -DOOR_FRAME_PILLAR_SIZE_X / 2.0, DOOR_FRAME_PILLAR_SIZE_Z / 2.0);
      addCollisionShape(pillarSizeX, pillarSizeY, pillarSizeZ, latchSidePostOffset);

      double flatboardSizeX = 0.63;
      double flatboardSizeY = 0.39;
      double flatboardSizeZ = 0.02;
      double twoX4Width = 0.0381;
      if (ENABLE_SUPPORT_BOARD_COLLISIONS)
      {
         Point3D hingeSideSupportBoardOffset = new Point3D();
         hingeSideSupportBoardOffset.add(0.0, -DOOR_FRAME_HINGE_OFFSET, 0.0);
         hingeSideSupportBoardOffset.add(0.0, -DOOR_FRAME_PILLAR_SIZE_X, 0.0);
         hingeSideSupportBoardOffset.add(0.0, -twoX4Width / 2.0, DOOR_FRAME_PILLAR_SIZE_X / 2.0f);
         double twoX4Length = 1.016;
         addCollisionShape(twoX4Length, twoX4Width, DOOR_FRAME_PILLAR_SIZE_X, hingeSideSupportBoardOffset);

         Point3D hingeSideSupportWeightBaseOffset = new Point3D();
         hingeSideSupportWeightBaseOffset.add(0.0, -DOOR_FRAME_HINGE_OFFSET, 0.0);
         hingeSideSupportWeightBaseOffset.add(0.0, -DOOR_FRAME_PILLAR_SIZE_X, 0.0);
         hingeSideSupportWeightBaseOffset.add(0.0, -twoX4Width, 0.0);
         hingeSideSupportWeightBaseOffset.add(0.0, -flatboardSizeY / 2.0, flatboardSizeZ / 2.0);
         addCollisionShape(flatboardSizeX, flatboardSizeY, 0.02, hingeSideSupportWeightBaseOffset);
      }

      Point3D hingeSidePostOffset = new Point3D();
      double toLatchPillarY = 0.93415;
      hingeSidePostOffset.add(0.0, toLatchPillarY, 0.0);
      hingeSidePostOffset.add(0.0, DOOR_FRAME_PILLAR_SIZE_X / 2.0, DOOR_FRAME_PILLAR_SIZE_Z / 2.0);
      addCollisionShape(pillarSizeX, pillarSizeY, pillarSizeZ, hingeSidePostOffset);

      if (ENABLE_SUPPORT_BOARD_COLLISIONS)
      {
         // shortened: 0.881195 long
         // short edge 0.372505 from center
         // scootch 0.0680925
         Point3D latchSideSupportBoardOffset = new Point3D();
         latchSideSupportBoardOffset.add(0.0, toLatchPillarY, 0.0);
         latchSideSupportBoardOffset.add(0.0, DOOR_FRAME_PILLAR_SIZE_X, 0.0);
         latchSideSupportBoardOffset.add(0.0680925, twoX4Width / 2.0, DOOR_FRAME_PILLAR_SIZE_X / 2.0f);
         double sideFloorBoardLength = 0.881195;
         addCollisionShape(sideFloorBoardLength, twoX4Width, DOOR_FRAME_PILLAR_SIZE_X, latchSideSupportBoardOffset);

         Point3D hingeSideSupportWeightBaseOffset = new Point3D();
         hingeSideSupportWeightBaseOffset.add(0.0, toLatchPillarY, 0.0);
         hingeSideSupportWeightBaseOffset.add(0.0, DOOR_FRAME_PILLAR_SIZE_X, 0.0);
         hingeSideSupportWeightBaseOffset.add(0.0, twoX4Width, 0.0);
         hingeSideSupportWeightBaseOffset.add(0.0, flatboardSizeY / 2.0, flatboardSizeZ / 2.0);
         addCollisionShape(flatboardSizeX, flatboardSizeY, 0.02, hingeSideSupportWeightBaseOffset);
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
