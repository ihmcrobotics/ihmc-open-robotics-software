package us.ihmc.perception.tools;

import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class PerceptionFilterTools
{

   public static void applyCollisionFilter(PlanarRegionsList planarRegionsList, CollidingScanRegionFilter collisionFilter)
   {
      // Filter out regions that are colliding with the body
      collisionFilter.update();
      int regionIndex = 0;
      while (regionIndex < planarRegionsList.getNumberOfPlanarRegions())
      {
         if (!collisionFilter.test(regionIndex, planarRegionsList.getPlanarRegion(regionIndex)))
            planarRegionsList.pollPlanarRegion(regionIndex);
         else
            ++regionIndex;
      }
   }

   public static CollidingScanRegionFilter createHumanoidShinCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
         CollisionShapeTester shapeTester = new CollisionShapeTester();
         for (RobotSide robotSide : RobotSide.values)
         {
            List<JointBasics> joints = new ArrayList<>();
            RigidBodyBasics shin = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
            MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
            joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
         }
         return new CollidingScanRegionFilter(shapeTester);
   }
}
