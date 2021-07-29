package us.ihmc.atlas.contactEstimation;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.avatar.contactEstimation.AvatarExternalContactEstimationSimulation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.physics.RobotCollisionModel;

public class AtlasExternalContactEstimationSimulation extends AvatarExternalContactEstimationSimulation
{
   static String jointName = "l_leg_kny";	static Point3D offset = new Point3D( 0.080, -0.040, -0.248 );
//   static String jointName = "l_leg_kny";	static Point3D offset = new Point3D( 0.093, -0.043, -0.099 );
//   static String jointName = "l_leg_kny";	static Point3D offset = new Point3D( 0.101,  0.017, -0.130 );
//   static String jointName = "l_leg_kny";	static Point3D offset = new Point3D( 0.052,  0.073, -0.157 );
//   static String jointName = "l_leg_kny";	static Point3D offset = new Point3D(-0.024,  0.070, -0.196 );
//   static String jointName = "l_leg_kny";	static Point3D offset = new Point3D(-0.068, -0.006, -0.228 );

   public AtlasExternalContactEstimationSimulation()
   {
      super(jointName, offset);
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
   }

   @Override
   protected RobotCollisionModel getCollisionModel()
   {
      return new AtlasMultiContactCollisionModel(new AtlasJointMap(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, new AtlasPhysicalProperties()));
   }

   public static void main(String[] args)
   {
      new AtlasExternalContactEstimationSimulation();
   }
}
