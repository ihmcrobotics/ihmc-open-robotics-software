package us.ihmc.atlas.stepReachability;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasAvoidanceCollisionModel;
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
   protected String getResourcesDirectory()
   {
      return "ihmc-open-robotics-software/atlas/src/main/resources";
   }

   @Override
   protected RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap)
   {
      AtlasAvoidanceCollisionModel collisionModel = new AtlasAvoidanceCollisionModel(jointMap);
      return collisionModel;
   }
}
