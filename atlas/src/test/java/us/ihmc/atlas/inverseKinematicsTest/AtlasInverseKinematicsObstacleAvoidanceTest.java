package us.ihmc.atlas.inverseKinematicsTest;

import java.util.List;

import com.badlogic.gdx.math.collision.BoundingBox;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.inverseKinematics.AvatarInverseKinematicsObstacleAvoidanceTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;

public class AtlasInverseKinematicsObstacleAvoidanceTest extends AvatarInverseKinematicsObstacleAvoidanceTest
{
   AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public boolean keepSCSUp()
   {
      return true;
   }

   @Override
   public int getNumberOfObstacles()
   {
      return 10;
   }

   @Override
   public boolean specifyObstacle()
   {
      return false;
   }

   @Override
   public List<FramePoint3D> getObstaclePositions()
   {
      return null;
   }

   @Override
   public List<CollisionShape> getListOfObstacles()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public BoundingBox getWorkspaceBoundsForCreatingObstacles()
   {
      // TODO Auto-generated method stub
      return null;
   }
   
}
