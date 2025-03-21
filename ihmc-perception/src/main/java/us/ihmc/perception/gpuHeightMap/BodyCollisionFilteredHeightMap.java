package us.ihmc.perception.gpuHeightMap;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.depthData.CollisionShapeTester;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class BodyCollisionFilteredHeightMap
{
//   public static void getRobotBodyShapes(FullHumanoidRobotModel fullRobotModel)
//   {
//      for (RobotSide robotSide : RobotSide.values)
//
//      {
//         List<JointBasics> joints = new ArrayList<>();
//         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide)
//                                              .getParentJoint()
//                                              .getPredecessor()
//                                              .getParentJoint()
//                                              .getPredecessor();
//         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
//         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
//      }
//      return new
//
//            CollidingScanRegionFilter(shapeTester);
//   }

   public static boolean isPointInSphere(Point3D center, double radius, Point3D point)
   {

      double distance = Math.sqrt(
            Math.pow(point.getX() - center.getX(), 2) + Math.pow(point.getY() - center.getY(), 2) + Math.pow(
                  point.getZ() - center.getZ(), 2));

      return distance <= radius;
   }
}
