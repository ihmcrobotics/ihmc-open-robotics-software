package us.ihmc.wholeBodyController;

import java.util.*;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.partNames.LeggedJointNameMap;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public class AdditionalSimulationContactPoints<E extends Enum<E> & RobotSegment<E>> implements FootContactPoints<E>
{
   private final int nContactPointsX;
   private final int nContactPointsY;
   private final boolean edgePointsOnly;

   private final boolean useSoftContactPointParameters;
   private final E[] robotSegments;

   public AdditionalSimulationContactPoints(E[] robotSegments, int nContactPointsX, int nContactPointsY, boolean edgePointsOnly, boolean useSoftContactPointParameters)
   {
      this.robotSegments = robotSegments;
      this.nContactPointsX = nContactPointsX;
      this.nContactPointsY = nContactPointsY;
      this.edgePointsOnly = edgePointsOnly;

      this.useSoftContactPointParameters = useSoftContactPointParameters;
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
               if (edgePointsOnly && ix != 1 && ix != nContactPointsX && iy != 1 && iy != nContactPointsY) // Only put points along the edges
                  continue;

               double x = (ix - 1) * dx - xOffset;
               double y = (iy - 1) * dy - yOffset;
               double z = 0.01 * ((xOffset - Math.abs(x))/xOffset + (yOffset - Math.abs(y))/yOffset);

               Point3D contactPoint = new Point3D(x, y, z);
               RigidBodyTransform transformToParentJointFrame = soleToAnkleFrameTransforms.get(segment);
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
      {
         ret.put(segment, new Point2D(footLength / 2.0, 0.0));
      }

      return ret;
   }

   @Override
   public SegmentDependentList<E, LineSegment2D> getToeOffContactLines(double footLength, double footWidth, double toeWidth)
   {
      SegmentDependentList<E, LineSegment2D> ret = new SegmentDependentList<>(robotSegments[0].getClassType());

      double footForward = footLength / 2.0;
      double halfToeWidth = toeWidth / 2.0;

      for (E segment : robotSegments)
      {
         ret.put(segment, new LineSegment2D(new Point2D(footForward, -halfToeWidth), new Point2D(footForward, halfToeWidth)));
      }

      return ret;
   }

   @Override
   public boolean useSoftContactPointParameters()
   {
      return useSoftContactPointParameters;
   }

}
