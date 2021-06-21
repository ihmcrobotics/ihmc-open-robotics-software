package us.ihmc.valkyrie.stepReachability;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.footstep.HumanoidStepReachabilityCalculator;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;

public class ValkyrieStepReachabilityCalculator extends HumanoidStepReachabilityCalculator
{
   public ValkyrieStepReachabilityCalculator() throws Exception
   {
      super();
   }

   public static void main(String[] args) throws Exception
   {
      new ValkyrieStepReachabilityCalculator();
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Override
   protected RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap)
   {
      ValkyrieSimulationCollisionModel collisionModel = new ValkyrieSimulationCollisionModel(jointMap);
      collisionModel.setCollidableHelper(new CollidableHelper(), "robot", "ground");
      return collisionModel;
   }
}
