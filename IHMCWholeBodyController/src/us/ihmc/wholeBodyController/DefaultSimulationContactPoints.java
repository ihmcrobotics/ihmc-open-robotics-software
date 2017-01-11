package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class DefaultSimulationContactPoints implements SimulationFootContactPoints
{
   @Override
   public Map<String, List<Tuple3d>> getContactPoints(double footLength, double footWidth, double toeWidth, DRCRobotJointMap jointMap,
         SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      HashMap<String, List<Tuple3d>> ret = new HashMap<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Tuple3d> footContactPoints = new ArrayList<>();
         String parentJointName = jointMap.getJointBeforeFootName(robotSide);

         //SCS Sim contactPoints
         int nContactPointsX = 2;
         int nContactPointsY = 2;

         double dx = 1.01 * footLength / (nContactPointsX - 1.0);
         double xOffset = 1.01 * footLength / 2.0;

         for (int ix = 1; ix <= nContactPointsX; ix++)
         {
            double alpha = (ix - 1.0) / (nContactPointsX - 1.0);
            double footWidthAtCurrentX = (1.0 - alpha) * 1.01 * footWidth + alpha * 1.01 * toeWidth;
            double dy = footWidthAtCurrentX / (nContactPointsY - 1.0);
            double yOffset = footWidthAtCurrentX / 2.0;

            for (int iy = 1; iy <= nContactPointsY; iy++)
            {
               double x = (ix - 1.0) * dx - xOffset;
               double y = (iy - 1.0) * dy - yOffset;
               RigidBodyTransform transformToParentJointFrame = soleToAnkleFrameTransforms.get(robotSide);

               Point3d contactPoint = new Point3d(x, y, 0.0);
               transformToParentJointFrame.transform(contactPoint);
               footContactPoints.add(contactPoint);
            }
         }

         ret.put(parentJointName, footContactPoints);
      }

      return ret;
   }

   @Override
   public boolean useSoftContactPointParameters()
   {
      return false;
   }

}
