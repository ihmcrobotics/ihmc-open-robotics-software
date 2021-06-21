package us.ihmc.atlas.stepReachability;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasKinematicsCollisionModel;
import us.ihmc.atlas.parameters.AtlasSimulationCollisionModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.reachabilityMap.footstep.HumanoidStepReachabilityCalculator;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.RobotCollisionModel;

public class AtlasStepReachabilityCalculator extends HumanoidStepReachabilityCalculator
{
   public AtlasStepReachabilityCalculator() throws Exception
   {
      super();
   }

   public static void main(String[] args) throws Exception
   {
      new AtlasStepReachabilityCalculator();
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
   }

   @Override
   protected RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap)
   {
      AtlasKinematicsCollisionModel collisionModel = new AtlasKinematicsCollisionModel(jointMap);
      return collisionModel;
   }
}
