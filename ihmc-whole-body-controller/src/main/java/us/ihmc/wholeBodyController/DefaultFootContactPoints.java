package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.partNames.LeggedJointNameMap;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public class DefaultFootContactPoints<E extends Enum<E> & RobotSegment<E>> implements FootContactPoints<E>
{
   private final E[] robotSegments;
   public DefaultFootContactPoints(E[] robotSegments)
   {
      this.robotSegments = robotSegments;
   }

   @Override
   public Map<String, List<Tuple3DBasics>> getSimulationContactPoints(double footLength, double footWidth, double toeWidth, LeggedJointNameMap<E> jointMap,
         SegmentDependentList<E, RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      HashMap<String, List<Tuple3DBasics>> ret = new HashMap<>();

      for (E segment : robotSegments)
      {
         ArrayList<Tuple3DBasics> footContactPoints = new ArrayList<>();
         String parentJointName = jointMap.getJointBeforeFootName(segment);

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
               RigidBodyTransform transformToParentJointFrame = soleToAnkleFrameTransforms.get(segment);

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
   public SegmentDependentList<E, List<Tuple2DBasics>> getControllerContactPoints(double footLength, double footWidth, double toeWidth)
   {
      SegmentDependentList<E, List<Tuple2DBasics>> ret = new SegmentDependentList<>(robotSegments[0].getClassType());

      for (E segment : robotSegments)
      {
         ArrayList<Tuple2DBasics> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
         contactPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
         contactPoints.add(new Point2D(footLength / 2.0, -toeWidth / 2.0));
         contactPoints.add(new Point2D(footLength / 2.0, toeWidth / 2.0));
         ret.put(segment, contactPoints);
      }

      return ret;
   }

   @Override
   public SegmentDependentList<E, Tuple2DBasics> getToeOffContactPoints(double footLength, double footWidth, double toeWidth)
   {
      SegmentDependentList<E, Tuple2DBasics> ret = new SegmentDependentList<>(robotSegments[0].getClassType());

      for (E segment : robotSegments)
         ret.put(segment, new Point2D(footLength / 2.0, 0.0));

      return ret;
   }

   @Override
   public SegmentDependentList<E, LineSegment2D> getToeOffContactLines(double footLength, double footWidth, double toeWidth)
   {
      SegmentDependentList<E, LineSegment2D> ret = new SegmentDependentList<>(robotSegments[0].getClassType());

      double footForward = footLength / 2.0;
      double halfToeWidth = toeWidth / 2.0;

      for (E segment : robotSegments)
         ret.put(segment, new LineSegment2D(new Point2D(footForward, -halfToeWidth), new Point2D(footForward, halfToeWidth)));

      return ret;
   }

   @Override
   public boolean useSoftContactPointParameters()
   {
      return false;
   }

}
