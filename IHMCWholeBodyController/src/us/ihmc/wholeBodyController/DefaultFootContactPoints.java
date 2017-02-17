package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class DefaultFootContactPoints implements FootContactPoints
{
   @Override
   public Map<String, List<Tuple3DBasics>> getSimulationContactPoints(double footLength, double footWidth, double toeWidth, DRCRobotJointMap jointMap,
         SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      HashMap<String, List<Tuple3DBasics>> ret = new HashMap<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Tuple3DBasics> footContactPoints = new ArrayList<>();
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

               Point3D contactPoint = new Point3D(x, y, 0.0);
               transformToParentJointFrame.transform(contactPoint);
               footContactPoints.add(contactPoint);
            }
         }

         ret.put(parentJointName, footContactPoints);
      }

      return ret;
   }

   @Override
   public SideDependentList<List<Tuple2DBasics>> getControllerContactPoints(double footLength, double footWidth, double toeWidth)
   {
      SideDependentList<List<Tuple2DBasics>> ret = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Tuple2DBasics> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
         contactPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
         contactPoints.add(new Point2D(footLength / 2.0, -toeWidth / 2.0));
         contactPoints.add(new Point2D(footLength / 2.0, toeWidth / 2.0));
         ret.put(robotSide, contactPoints);
      }

      return ret;
   }

   @Override
   public SideDependentList<Tuple2DBasics> getToeOffContactPoints(double footLength, double footWidth, double toeWidth)
   {
      SideDependentList<Tuple2DBasics> ret = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
         ret.put(robotSide, new Point2D(footLength / 2.0, 0.0));

      return ret;
   }

   @Override
   public boolean useSoftContactPointParameters()
   {
      return false;
   }

}
