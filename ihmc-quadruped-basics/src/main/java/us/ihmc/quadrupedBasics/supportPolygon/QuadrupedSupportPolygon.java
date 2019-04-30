package us.ihmc.quadrupedBasics.supportPolygon;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.robotSide.*;

import java.io.Serializable;

import static us.ihmc.robotics.robotSide.RobotQuadrant.*;

public class QuadrupedSupportPolygon extends FrameConvexPolygon2D implements Serializable
{
   private static final long serialVersionUID = 4247638266737494462L;

   private final RecyclingQuadrantDependentList<FramePoint3D> footsteps = new RecyclingQuadrantDependentList<>(FramePoint3D.class);

   private final FramePoint3D temporaryFramePoint = new FramePoint3D();
   private final FrameVector3D tempPlaneNormalInWorld = new FrameVector3D();

   private final FrameVector3D[] tempVectorsForInCirclePoint = new FrameVector3D[] {new FrameVector3D(), new FrameVector3D(), new FrameVector3D(),
         new FrameVector3D()};
   private final FramePoint3D[] tempPointListForInCirclePoint = new FramePoint3D[4];
   private final FramePoint3D tempInCircleCenter = new FramePoint3D();

   private final FramePoint3D tempIntersection = new FramePoint3D();

   private final FrameVector3D[] tempVectorsForCommonSupportPolygon = new FrameVector3D[] {new FrameVector3D(), new FrameVector3D()};
   private final Point2D[] tempPointsForCornerCircle = new Point2D[] {new Point2D(), new Point2D(), new Point2D(), new Point2D()};
   private final Vector2D tempVectorForCornerCircle = new Vector2D();

   private final FramePoint2D tempFramePoint2dOne = new FramePoint2D();

   void printOutPolygon(String string)
   {
      System.out.print(getClass().getSimpleName() + ": " + string);
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         System.out.print("\n" + robotQuadrant + " " + getFootstep(robotQuadrant));
      }
      System.out.println();
   }

   @Override
   public String toString()
   {
      String string = getClass().getSimpleName();
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         string += "\n" + robotQuadrant + " " + getFootstep(robotQuadrant);
      }

      string += super.toString();
      return string;
   }

   @Override
   public void changeFrame(ReferenceFrame referenceFrame)
   {
      super.changeFrame(referenceFrame);

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
         getFootstep(robotQuadrant).changeFrame(referenceFrame);
   }

   /**
    * This method cycles over the cases 0 - 4 for the number
    * of Footsteps in the polygon. The ordering of the pairs
    * starts with the FL and goes clockwise. E.g., if the swing leg is the FR,
    * the pairs will be: FL-HR, HR-HL, HL-FL
    *
    * @return LegName[][]
    */
   public RobotQuadrant[][] getLegPairs()
   {
      int numberOfLegs = getNumberOfVertices();
      switch (numberOfLegs)
      {
      case 0:
      case 1:
         throw new UndefinedOperationException("No leg pairs exist");
      case 2:
         RobotQuadrant firstLeg = getFirstSupportingQuadrant();
         RobotQuadrant lastLeg = getLastSupportingQuadrant();
         return new RobotQuadrant[][] {{firstLeg, lastLeg}};
      case 3:
         switch (getFirstNonSupportingQuadrant())
         {
         case FRONT_LEFT:
            return new RobotQuadrant[][] {{RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT}, {RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT},
                  {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT}};
         case FRONT_RIGHT:
            return new RobotQuadrant[][] {{RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_RIGHT}, {RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT},
                  {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT}};
         case HIND_RIGHT:
            return new RobotQuadrant[][] {{RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT}, {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_LEFT},
                  {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT}};
         case HIND_LEFT:
            return new RobotQuadrant[][] {{RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT}, {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT},
                  {RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_LEFT}};
         }
      case 4:
         return new RobotQuadrant[][] {{RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT}, {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT},
               {RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT}, {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT}};
      default:
         throw new RuntimeException();
      }
   }

   public RobotQuadrant[] getSupportingQuadrantsInOrder()
   {
      return footsteps.quadrants();
   }

   public RobotQuadrant getFirstSupportingQuadrant()
   {
      if (footsteps.containsQuadrant(RobotQuadrant.FRONT_LEFT)) // begin punching nanos
         return RobotQuadrant.FRONT_LEFT;
      else if (footsteps.containsQuadrant(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (footsteps.containsQuadrant(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (footsteps.containsQuadrant(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else
         throw new EmptySupportPolygonException();
   }

   public RobotQuadrant getFirstNonSupportingQuadrant()
   {
      if (!footsteps.containsQuadrant(RobotQuadrant.FRONT_LEFT))
         return RobotQuadrant.FRONT_LEFT;
      else if (!footsteps.containsQuadrant(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (!footsteps.containsQuadrant(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (!footsteps.containsQuadrant(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else
         throw new RuntimeException("Polygon is full");
   }

   public RobotQuadrant getLastSupportingQuadrant()
   {
      if (footsteps.containsQuadrant(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else if (footsteps.containsQuadrant(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (footsteps.containsQuadrant(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (footsteps.containsQuadrant(RobotQuadrant.FRONT_LEFT))
         return RobotQuadrant.FRONT_LEFT;
      else
         throw new EmptySupportPolygonException();
   }

   public RobotQuadrant getLastNonSupportingQuadrant()
   {
      if (!footsteps.containsQuadrant(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else if (!footsteps.containsQuadrant(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (!footsteps.containsQuadrant(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (!footsteps.containsQuadrant(RobotQuadrant.FRONT_LEFT))
         return RobotQuadrant.FRONT_LEFT;
      else
         throw new RuntimeException("Polygon is full");
   }

   public RobotQuadrant getNextClockwiseSupportingQuadrant(RobotQuadrant robotQuadrant)
   {
      RobotQuadrant prospectiveQuadrant = robotQuadrant;
      for (int i = 0; i < 4; i++)
      {
         prospectiveQuadrant = prospectiveQuadrant.getNextClockwiseQuadrant();
         if (footsteps.containsQuadrant(prospectiveQuadrant))
            return prospectiveQuadrant;
      }

      throw new EmptySupportPolygonException();
   }

   public RobotQuadrant getNextCounterClockwiseSupportingQuadrant(RobotQuadrant robotQuadrant)
   {
      RobotQuadrant prospectiveQuadrant = robotQuadrant;
      for (int i = 0; i < 4; i++)
      {
         prospectiveQuadrant = prospectiveQuadrant.getNextCounterClockwiseQuadrant();
         if (footsteps.containsQuadrant(prospectiveQuadrant))
            return prospectiveQuadrant;
      }

      throw new EmptySupportPolygonException();
   }

   public FramePoint3D getFootstep(RobotQuadrant robotQuadrant)
   {
      return footsteps.get(robotQuadrant);
   }

   public FramePoint3D reviveFootstep(RobotQuadrant robotQuadrant)
   {
      return footsteps.add(robotQuadrant);
   }

   public void set(QuadrupedSupportPolygon polygon)
   {
      super.set(polygon);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         setFootstep(robotQuadrant, polygon.getFootstep(robotQuadrant));
      }
   }

   public void setFootstep(RobotQuadrant robotQuadrant, FramePoint3DReadOnly footstep)
   {
      if (footstep == null)
         return;

      footsteps.add(robotQuadrant).setIncludingFrame(footstep);
      footsteps.get(robotQuadrant).changeFrame(getReferenceFrame());
      updatePolygon();
   }

   public void setFootstep(RobotQuadrant robotQuadrant, ReferenceFrame footstepFrame)
   {
      if (footstepFrame == null)
         return;

      footsteps.add(robotQuadrant).setToZero(footstepFrame);
      footsteps.get(robotQuadrant).changeFrame(getReferenceFrame());
      updatePolygon();
   }

   public void removeFootstep(RobotQuadrant robotQuadrant)
   {
      footsteps.remove(robotQuadrant);
      updatePolygon();
   }

   public void clearFootsteps()
   {
      footsteps.clear();
      updatePolygon();
   }

   public void updatePolygon()
   {
      super.clear();
      for (RobotQuadrant supportingQuadrant : getSupportingQuadrantsInOrder())
      {
         super.addVertexMatchingFrame(getFootstep(supportingQuadrant));
      }
      super.update();
   }

   /**
    * Replaces the stored footstep with the passed in one.
    *
    * @param support polygon to pack
    * @param quadrant to replace
    * @param resulting footstep
    */
   public void getAndReplaceFootstep(QuadrupedSupportPolygon supportPolygonToPack, RobotQuadrant quadrant, FramePoint3D footstep)
   {
      supportPolygonToPack.set(this);
      supportPolygonToPack.setFootstep(quadrant, footstep);
   }

   public void getAndRemoveFootstep(QuadrupedSupportPolygon supportPolygonToPack, RobotQuadrant quadrantToRemove)
   {
      supportPolygonToPack.set(this);
      supportPolygonToPack.removeFootstep(quadrantToRemove);
   }

   public void getAndSwapSameSideFootsteps(QuadrupedSupportPolygon supportPolygonToPack, RobotSide sideToSwap)
   {
      supportPolygonToPack.setFootstep(getQuadrant(RobotEnd.HIND, sideToSwap), getFootstep(getQuadrant(RobotEnd.FRONT, sideToSwap)));
      supportPolygonToPack.setFootstep(getQuadrant(RobotEnd.FRONT, sideToSwap), getFootstep(getQuadrant(RobotEnd.HIND, sideToSwap)));
   }

   /**
    * Translates this polygon in X and Y.
    */
   public void translate(Vector3DReadOnly translateBy)
   {
      translate(translateBy.getX(), translateBy.getY(), translateBy.getZ());
   }

   /**
    * Translates this polygon in X and Y.
    */
   public void translate(double x, double y, double z)
   {
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         getFootstep(robotQuadrant).add(x, y, z);
      }
      super.translate(x, y);
   }

   /**
    * Rotates the feet about the Centroid, keeping the z heights.
    *
    * WARNING: Generates garbage!
    *
    * @return SupportPolygon
    */
   public void yawAboutCentroid(double yaw)
   {
      getCentroid(temporaryFramePoint);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint3D footstep = getFootstep(quadrant);
         if (footstep != null)
         {
            FramePoint3D rotatedPoint = new FramePoint3D();
            GeometryTools.yawAboutPoint(footstep, temporaryFramePoint, yaw, rotatedPoint);
            footstep.set(rotatedPoint);
         }
      }
   }

   public boolean containsFootstep(RobotQuadrant robotQuadrant)
   {
      return footsteps.containsQuadrant(robotQuadrant);
   }

   /**
    * Get footstep with least Z height.
    *
    * @return lowest footstep
    */
   public RobotQuadrant getLowestFootstep()
   {
      double minZ = Double.POSITIVE_INFINITY;
      RobotQuadrant lowest = null;

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         double footZ = getFootstep(robotQuadrant).getZ();
         if (footZ < minZ)
         {
            minZ = footZ;
            lowest = robotQuadrant;
         }
      }

      return lowest;
   }

   /**
    * Get footstep with most Z height.
    *
    * @return highest footstep
    */
   public RobotQuadrant getHighestFootstep()
   {
      double maxZ = Double.NEGATIVE_INFINITY;
      RobotQuadrant highest = null;

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         double footZ = getFootstep(robotQuadrant).getZ();
         if (footZ > maxZ)
         {
            maxZ = footZ;
            highest = robotQuadrant;
         }
      }

      return highest;
   }

   public double getLowestFootstepZHeight()
   {
      return getFootstep(getLowestFootstep()).getZ();
   }

   public RobotQuadrant getClosestFootstep(FramePoint3D pointToCompare)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      RobotQuadrant closestQuadrant = null;
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         double distance = getFootstep(robotQuadrant).distance(pointToCompare);
         if (distance < minDistance)
         {
            closestQuadrant = robotQuadrant;
            minDistance = distance;
         }
      }
      return closestQuadrant;
   }

   /**
    * if you pick up a leg and want equal weight distribution, pretend that there are two legs on top of eachother
    * @param centroidToPack
    */
   public void getCentroidEqualWeightingEnds(FramePoint3D centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      int size = getNumberOfVertices();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (getFootstep(robotQuadrant) != null)
         {
            centroidToPack.add(getFootstep(robotQuadrant));
         }
         else
         {
            FramePoint3D acrossBodyFootstep = getFootstep(robotQuadrant.getAcrossBodyQuadrant());
            if (acrossBodyFootstep != null)
            {
               centroidToPack.add(acrossBodyFootstep);
               size++;
            }
         }
      }

      centroidToPack.scale(1.0 / size);
   }

   public void getCentroid(FramePoint3D centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         centroidToPack.add(getFootstep(robotQuadrant));
      }

      centroidToPack.scale(1.0 / getNumberOfVertices());
   }

   /**
    * gets the Centroid of the supplied quadrants and sets the z value to the average between the lowest foot in
    * the front and hind in world frame. If no Hinds or no Fronts are supplied it will use the lowest foot given.
    * If no quadrants are supplied the result will be Zeros in world.
    */
   public boolean getCentroidAveragingLowestZHeightsAcrossEnds(FramePoint3DBasics centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      double frontZ = Double.MAX_VALUE;
      double hindZ = Double.MAX_VALUE;

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         FramePoint3D footstep = getFootstep(robotQuadrant);
         centroidToPack.add(footstep);

         double currentFootZ = footstep.getZ();
         if (robotQuadrant.isQuadrantInFront() && currentFootZ < frontZ)
         {
            frontZ = currentFootZ;
         }
         else if (robotQuadrant.isQuadrantInHind() && currentFootZ < hindZ)
         {
            hindZ = currentFootZ;
         }
      }

      centroidToPack.scale(1.0 / getNumberOfVertices());

      if (centroidToPack.containsNaN())
      {
         String message = "Centroid position is incorrect. \n\tSupporting quadrants are : ";
         for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
            message += "\n\t" + robotQuadrant.name() + " = " + getFootstep(robotQuadrant);
         message += "\nCentroid = " + centroidToPack;
         PrintTools.warn(message);
         return false;
      }

      double averageZ = 0.0;
      if (hindZ < Double.MAX_VALUE)
      {
         averageZ += hindZ;
      }
      if (frontZ < Double.MAX_VALUE)
      {
         averageZ += frontZ;
      }
      averageZ /= 2.0;

      if (!Double.isFinite(averageZ))
      {
         PrintTools.warn("Average Z of the centroid is incorrect. hindZ = " + hindZ + ", frontZ = " + frontZ + ", averageZ = " + averageZ);
         return false;
      }

      centroidToPack.setZ(averageZ);

      return true;
   }

   /**
    * gets the weighted Centroid of support polyon and sets the z value to the average between the lowest foot in
    * the front and hind in world frame. If no Hinds or no Fronts are supplied it will use the lowest foot given.
    * If no quadrants are available the result will be Zeros in world.
    */
   public void getCentroidWithEqualWeightedEndsAveragingLowestZHeightsAcrossEnds(FramePoint3D centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      getCentroidEqualWeightingEnds(centroidToPack);

      double frontZ = Double.MAX_VALUE;
      double hindZ = Double.MAX_VALUE;

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         FramePoint3D footstep = getFootstep(robotQuadrant);
         double currentFootZ = footstep.getZ();
         if (robotQuadrant.isQuadrantInFront() && currentFootZ < frontZ)
         {
            frontZ = currentFootZ;
         }
         else if (robotQuadrant.isQuadrantInHind() && currentFootZ < hindZ)
         {
            hindZ = currentFootZ;
         }
      }

      double averageZ = 0.0;
      if (hindZ < Double.MAX_VALUE)
      {
         averageZ += hindZ;
      }
      if (frontZ < Double.MAX_VALUE)
      {
         averageZ += frontZ;
      }
      averageZ /= 2.0;
      centroidToPack.setZ(averageZ);
   }

   private final FramePoint3D tempFramePointForCentroids = new FramePoint3D();

   /** returns success **/
   public boolean getCentroidFramePoseAveragingLowestZHeightsAcrossEnds(FramePose3DBasics framePose)
   {
      double nominalPitch = getNominalPitch();
      double nominalRoll = getNominalRoll();
      double nominalYaw = getNominalYaw();

      if (!Double.isFinite(nominalPitch))
      {
         PrintTools.warn("Nominal pitch is not finite " + nominalPitch + ", not updating the pose.");
         return false;
      }

      if (!Double.isFinite(nominalRoll))
      {
         PrintTools.warn("Nominal roll is not finite " + nominalRoll + ", not updating the pose.");
         return false;
      }

      if (!Double.isFinite(nominalYaw))
      {
         PrintTools.warn("Nominal yaw is not finite " + nominalYaw + ", not updating the pose.");
         return false;
      }

      if (!getCentroidAveragingLowestZHeightsAcrossEnds(tempFramePointForCentroids))
      {
         PrintTools.warn("Centroid contains NaN, not updating the pose.");
         return false;
      }

      framePose.setOrientationYawPitchRoll(nominalYaw, nominalPitch, nominalRoll);
      framePose.setPosition(tempFramePointForCentroids);

      return true;
   }

   public void getWeightedCentroidFramePoseAveragingLowestZHeightsAcrossEnds(FramePose3D framePose)
   {
      double nominalPitch = getNominalPitch();
      double nominalRoll = getNominalRoll();
      double nominalYaw = getNominalYaw();

      getCentroidWithEqualWeightedEndsAveragingLowestZHeightsAcrossEnds(tempFramePointForCentroids);
      framePose.setOrientationYawPitchRoll(nominalYaw, nominalPitch, nominalRoll);
      framePose.setPosition(tempFramePointForCentroids);
   }

   public void getCentroid2d(FramePoint2D centroidToPack2d)
   {
      centroidToPack2d.setToZero();

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         centroidToPack2d.add(getFootstep(robotQuadrant).getX(), getFootstep(robotQuadrant).getY());
      }

      centroidToPack2d.scale(1.0 / getNumberOfVertices());
   }

   /**
    * getBounds modifies the min and max points passed in to the min and max
    * xy keys contained in the set of Footsteps that make up the polygon
    *
    * @param minToPack Point2d  Minimum x and y value contained in footsteps list
    * @param maxToPack Point2d  Maximum x and y value contained in footsteps list
    */
   public void getBounds(Point2D minToPack, Point2D maxToPack)
   {
      minToPack.setX(Double.POSITIVE_INFINITY);
      minToPack.setY(Double.POSITIVE_INFINITY);
      maxToPack.setX(Double.NEGATIVE_INFINITY);
      maxToPack.setY(Double.NEGATIVE_INFINITY);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footstep = getFootstep(robotQuadrant);
         if (footstep != null)
         {
            if (footstep.getX() < minToPack.getX())
            {
               minToPack.setX(footstep.getX());
            }

            if (footstep.getY() < minToPack.getY())
            {
               minToPack.setY(footstep.getY());
            }

            if (footstep.getX() > maxToPack.getX())
            {
               maxToPack.setX(footstep.getX());
            }

            if (footstep.getY() > maxToPack.getY())
            {
               maxToPack.setY(footstep.getY());
            }
         }
      }
   }

   /**
    * Returns true if the given (x,y) value is inside the polygon. This test
    * ignores Z keys for the polygon. The test works by computing the minimum
    * distance from each polygon line segment to the point.
    *
    * @param point Point2d
    * @return boolean
    */
   public boolean isPointInside(FramePoint3DReadOnly point)
   {
      tempFramePoint2dOne.set(point);
      return isPointInside(tempFramePoint2dOne);
   }

   public double signedDistance(FramePoint3D point)
   {
      tempFramePoint2dOne.set(point);
      return signedDistance(tempFramePoint2dOne);
   }

   /**
    * Get the radius of the largest circle that can be
    * drawn in the polygon.
    *
    * @return radius of the in circle
    */
   public double getInCircleRadius2d()
   {
      return getInCircle2d(tempInCircleCenter);
   }

   /**
    * Get the radius and center point of the largest
    * circle that can be drawn in the polygon.
    *
    * @param center of circle point to pack
    * @return radius of the in circle
    */
   public double getInCircle2d(FramePoint3D inCircleCenterToPack)
   {
      getInCirclePoint2d(inCircleCenterToPack);

      double minimumRadius = Double.MAX_VALUE;
      double radius;

      for (RobotQuadrant[] legPair : getLegPairs())
      {
         radius = GeometryTools.distanceFromPointToLine2d(inCircleCenterToPack, getFootstep(legPair[0]), getFootstep(legPair[1]));

         if (radius < minimumRadius)
         {
            minimumRadius = radius;
         }
      }

      return minimumRadius;
   }

   /**
    * getInCirclePoint
    *
    * This method assumes the points are in a specific order (U-shape).
    * It returns the InCircle Point based on the two angles formed by the three line segments
    *
    * @return Point2d in circle point
    */
   public void getInCirclePoint2d(FramePoint3D intersectionToPack)
   {
      if (getNumberOfVertices() < 3)
      {
         throw new UndefinedOperationException("InCirclePoint only defined for 3 and 4 legs. getNumberOfVertices() = " + getNumberOfVertices());
      }

      int i = 0;
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         tempPointListForInCirclePoint[i++] = getFootstep(robotQuadrant);
      }

      if (getNumberOfVertices() == 3)
      {
         tempPointListForInCirclePoint[3] = tempPointListForInCirclePoint[0];
      }

      // get first two line segments
      tempVectorsForInCirclePoint[0].sub(tempPointListForInCirclePoint[0], tempPointListForInCirclePoint[1]);
      tempVectorsForInCirclePoint[0].normalize();
      tempVectorsForInCirclePoint[1].sub(tempPointListForInCirclePoint[2], tempPointListForInCirclePoint[1]);
      tempVectorsForInCirclePoint[1].normalize();

      // normalize and subtract to get vector that bisects the first angle
      FrameVector3D v2p = tempVectorsForInCirclePoint[0];
      v2p.add(tempVectorsForInCirclePoint[1]);
      v2p.normalize();

      // get second two line segments
      tempVectorsForInCirclePoint[2].sub(tempPointListForInCirclePoint[3], tempPointListForInCirclePoint[2]);
      tempVectorsForInCirclePoint[2].normalize();
      tempVectorsForInCirclePoint[3].sub(tempPointListForInCirclePoint[1], tempPointListForInCirclePoint[2]);
      tempVectorsForInCirclePoint[3].normalize();

      // normalize and subtract to get vector that bisects the second angle
      FrameVector3D v3p = tempVectorsForInCirclePoint[2];
      v3p.add(tempVectorsForInCirclePoint[3]);
      v3p.normalize();

      // find intersection point of the two bisecting vectors
      GeometryTools.getIntersectionBetweenTwoLines2d(tempPointListForInCirclePoint[1], v2p, tempPointListForInCirclePoint[2], v3p, intersectionToPack);
   }

   /**
    * Get the distance of the point inside of the in circle.
    *
    * @param point
    * @return distance
    */
   public double getDistanceInsideInCircle2d(FramePoint3D point)
   {
      double inCircleRadius = getInCircle2d(tempInCircleCenter);
      double distanceToInCircleCenter = point.distanceXY(tempInCircleCenter);
      return (inCircleRadius - distanceToInCircleCenter);
   }

   /**
    * Computes a nominal yaw angle. If triangle support, then the angle from the back to front support foot on the same side.
    * If a quad, then the average of the two front to back angles.
    *
    * @return double
    */
   public double getNominalYaw()
   {
      return getNominalYaw(footsteps, getNumberOfVertices());
   }

   public static double getNominalYaw(QuadrantDependentList<? extends FramePoint3DReadOnly> solePositions, int numberOfVertices)
   {
      if (numberOfVertices >= 3)
      {
         double deltaX = 0.0;
         double deltaY = 0.0;

         if (solePositions.containsKey(FRONT_LEFT) && solePositions.containsKey(HIND_LEFT))
         {
            deltaX += solePositions.get(FRONT_LEFT).getX() - solePositions.get(HIND_LEFT).getX();
            deltaY += solePositions.get(FRONT_LEFT).getY() - solePositions.get(HIND_LEFT).getY();
         }
         if (solePositions.containsKey(FRONT_RIGHT) && solePositions.containsKey(HIND_RIGHT))
         {
            deltaX += solePositions.get(FRONT_RIGHT).getX() - solePositions.get(HIND_RIGHT).getX();
            deltaY += solePositions.get(FRONT_RIGHT).getY() - solePositions.get(HIND_RIGHT).getY();
         }

         if (!Double.isFinite(deltaX))
            throw new IllegalArgumentException("deltaX is invalid = " + deltaX);
         if (!Double.isFinite(deltaY))
            throw new IllegalArgumentException("deltaY is invalid = " + deltaY);

         return Math.atan2(deltaY, deltaX);
      }
      else
      {
         throw new UndefinedOperationException("Undefined for less than 3 vertices. vertices = " + numberOfVertices);
      }
   }

   /**
    * Angle from hind left to hind right footstep.
    */
   public double getNominalYawHindLegs()
   {
      if (footsteps.containsQuadrant(RobotQuadrant.HIND_RIGHT) && footsteps.containsQuadrant(RobotQuadrant.HIND_LEFT))
      {
         double deltaX = getFootstep(HIND_RIGHT).getX() - getFootstep(HIND_LEFT).getX();
         double deltaY = getFootstep(HIND_RIGHT).getY() - getFootstep(HIND_LEFT).getY();
         return Math.atan2(deltaY, deltaX);
      }
      else
      {
         throw new UndefinedOperationException("Polygon must contain both hind legs.");
      }
   }

   /**
    * Computes a nominal pitch angle. If triangle support, then the angle from the front to back support foot on the same side.
    * If a quad, then the average of the two front to back angles.
    * @return double
    */
   double getNominalPitch()
   {
      if (getNumberOfVertices() >= 3)
      {
         double deltaX = 0.0;
         double deltaY = 0.0;
         double deltaZ = 0.0;

         if ((footsteps.containsQuadrant(RobotQuadrant.FRONT_LEFT) && footsteps.containsQuadrant(RobotQuadrant.HIND_LEFT)))
         {
            deltaX += getFootstep(RobotQuadrant.FRONT_LEFT).getX() - getFootstep(RobotQuadrant.HIND_LEFT).getX();
            deltaY += getFootstep(RobotQuadrant.FRONT_LEFT).getY() - getFootstep(RobotQuadrant.HIND_LEFT).getY();
            deltaZ += getFootstep(RobotQuadrant.FRONT_LEFT).getZ() - getFootstep(RobotQuadrant.HIND_LEFT).getZ();
         }

         if (footsteps.containsQuadrant(RobotQuadrant.FRONT_RIGHT) && footsteps.containsQuadrant(RobotQuadrant.HIND_RIGHT))
         {
            deltaX += getFootstep(RobotQuadrant.FRONT_RIGHT).getX() - getFootstep(RobotQuadrant.HIND_RIGHT).getX();
            deltaY += getFootstep(RobotQuadrant.FRONT_RIGHT).getY() - getFootstep(RobotQuadrant.HIND_RIGHT).getY();
            deltaZ += getFootstep(RobotQuadrant.FRONT_RIGHT).getZ() - getFootstep(RobotQuadrant.HIND_RIGHT).getZ();
         }

         double length = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
         if (length < 1e-3)
            throw new UndefinedOperationException("Polygon is too small");

         if (deltaZ > length)
            throw new IllegalArgumentException("Somehow ended up with a delta Z bigger than the length. deltaZ = " + deltaZ + ", length = " + length);

         return -Math.asin(deltaZ / length);
      }
      else
      {
         throw new UndefinedOperationException("Undefined for less than 3 vertices. vertices = " + getNumberOfVertices());
      }
   }

   /**
    * Computes a nominal roll angle. If triangle support, then the angle from the front to back support foot on the same side.
    * If a quad, then the average of the two front to back angles.
    * @return double
    */
   double getNominalRoll()
   {
      if (getNumberOfVertices() >= 3)
      {
         double deltaX = 0.0;
         double deltaY = 0.0;
         double deltaZ = 0.0;

         if (containsFootstep(FRONT_LEFT) && containsFootstep(FRONT_RIGHT))
         {
            deltaX += getFootstep(FRONT_LEFT).getX() - getFootstep(FRONT_RIGHT).getX();
            deltaY += getFootstep(FRONT_LEFT).getY() - getFootstep(FRONT_RIGHT).getY();
            deltaZ += getFootstep(FRONT_LEFT).getZ() - getFootstep(FRONT_RIGHT).getZ();
         }

         if (containsFootstep(HIND_LEFT) && containsFootstep(HIND_RIGHT))
         {
            deltaX = getFootstep(HIND_LEFT).getX() - getFootstep(HIND_RIGHT).getX();
            deltaY = getFootstep(HIND_LEFT).getY() - getFootstep(HIND_RIGHT).getY();
            deltaZ = getFootstep(HIND_LEFT).getZ() - getFootstep(HIND_RIGHT).getZ();
         }

         double length = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
         if (length < 1e-3)
            throw new UndefinedOperationException("Polygon is too small");

         if (deltaZ > length)
            throw new IllegalArgumentException("Somehow ended up with a delta Z bigger than the length. deltaZ = " + deltaZ + ", length = " + length);

         return Math.asin(deltaZ / length);
      }
      else
      {
         throw new UndefinedOperationException("Undefined for less than 3 vertices. vertices = " + getNumberOfVertices());
      }
   }

   /**
    * Check if the polygons are same getNumberOfVertices and contain the same quadrants.
    *
    * @param polygonToCompare
    * @return contain same quadrants
    */
   public boolean containsSameQuadrants(QuadrupedSupportPolygon polygonToCompare)
   {
      if (getNumberOfVertices() != polygonToCompare.getNumberOfVertices())
         return false;

      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         if (!polygonToCompare.containsFootstep(robotQuadrant))
         {
            return false;
         }
      }

      return true;
   }

   /**
    *  This method returns the common support polygon bewteen this and the supplied supportPolygon
    *
    *  *** Assumes regular gait
    *
    * @param polygonToCompare SupportPolygon
    *        1) must contain only three kegs
    *        2) exactly two footsteps must be the same
    * @param quadrantToAssignToIntersection LegName is the legname to assign to the intersection footstep
    *        3) must be one of the swinging (same side) leg names
    * @return SupportPolygon That is common to both (can be null for opposite side swing legs)
    *        This should be the two matching feet and the intersection of the two tror lines
    */
   public void getCommonTriangle2d(QuadrupedSupportPolygon polygonToCompare, QuadrupedSupportPolygon commonPolygonToPack,
                                   RobotQuadrant quadrantToAssignToIntersection)
   {
      // verify both have exactly three legs
      if (getNumberOfVertices() != 3)
         throw new UndefinedOperationException("This supportPolygon must contain exactly three legs, not " + getNumberOfVertices());
      if (polygonToCompare.getNumberOfVertices() != 3)
         throw new UndefinedOperationException("Supplied supportPolygon must contain exactly three legs, not " + polygonToCompare.getNumberOfVertices());
      // verify exactly two legs epsilon match
      if (getNumberOfEqualFootsteps(polygonToCompare) != 2)
         throw new UndefinedOperationException("There must be exactly two similar foosteps not " + getNumberOfEqualFootsteps(polygonToCompare));
      // verify swing legs are not diagonals quadrants
      if (getFirstNonSupportingQuadrant() == polygonToCompare.getFirstNonSupportingQuadrant().getDiagonalOppositeQuadrant())
         throw new UndefinedOperationException("Swing quadrants must not be diagonal opposites.");
      if (commonPolygonToPack == this)
         throw new RuntimeException("Can't pack into self. commonPolygonToPack = this");

      // return null if swing legs are not same side *** Assumes regular gait ***
      RobotQuadrant thisSwingLeg = getFirstNonSupportingQuadrant();
      RobotQuadrant compareSwingLeg = polygonToCompare.getFirstNonSupportingQuadrant();

      // verify specified swing leg name is one of the swinging (same side) leg names
      if (quadrantToAssignToIntersection != thisSwingLeg && quadrantToAssignToIntersection != compareSwingLeg)
         throw new UndefinedOperationException("The specified intersection quadrant must be one of the swinging (same side) leg names");

      FrameVector3D direction1 = tempVectorsForCommonSupportPolygon[0];
      direction1.sub(getFootstep(compareSwingLeg.getDiagonalOppositeQuadrant()), getFootstep(compareSwingLeg));

      FrameVector3D direction2 = tempVectorsForCommonSupportPolygon[1];
      direction2.sub(polygonToCompare.getFootstep(thisSwingLeg.getDiagonalOppositeQuadrant()), polygonToCompare.getFootstep(thisSwingLeg));

      commonPolygonToPack.clear();
      FramePoint3D intersection = commonPolygonToPack.reviveFootstep(quadrantToAssignToIntersection);
      GeometryTools
            .getIntersectionBetweenTwoLines2d(getFootstep(compareSwingLeg), direction1, polygonToCompare.getFootstep(thisSwingLeg), direction2, intersection);

      commonPolygonToPack.setFootstep(thisSwingLeg.getDiagonalOppositeQuadrant(), getFootstep(thisSwingLeg.getDiagonalOppositeQuadrant()));
      commonPolygonToPack.setFootstep(compareSwingLeg.getDiagonalOppositeQuadrant(), getFootstep(compareSwingLeg.getDiagonalOppositeQuadrant()));
   }

   /**
    *  This method compares this to another support polygon and returns the number of matching footsetps
    *
    * @return int the number of footsteps that epsilon match
    */
   public int getNumberOfEqualFootsteps(QuadrupedSupportPolygon polygonToCompare)
   {
      int numberOfEqual = 0;
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         if (polygonToCompare.getFootstep(robotQuadrant) != null && getFootstep(robotQuadrant)
               .epsilonEquals(polygonToCompare.getFootstep(robotQuadrant), 0.0005))
         {
            ++numberOfEqual;
         }
      }

      return numberOfEqual;
   }

   /**
    * Returns the shrunken common triangle.
    *
    * @param nextSupportPolygon
    * @param shrunkenCommonPolygonToPack
    * @param tempCommonSupportPolygon Can be any non-null polygon.
    * @param quadrantForIntersection
    * @param frontDistance
    * @param sideDistance
    * @param hindDistance
    */
   public void getShrunkenCommonTriangle2d(QuadrupedSupportPolygon nextSupportPolygon, QuadrupedSupportPolygon shrunkenCommonPolygonToPack,
                                           QuadrupedSupportPolygon tempCommonSupportPolygon, RobotQuadrant quadrantForIntersection, double frontDistance,
                                           double sideDistance, double hindDistance)
   {
      getCommonTriangle2d(nextSupportPolygon, tempCommonSupportPolygon, quadrantForIntersection);

      shrunkenCommonPolygonToPack.set(tempCommonSupportPolygon);

      RobotQuadrant swingLeg = tempCommonSupportPolygon.getFirstNonSupportingQuadrant();
      RobotQuadrant swingLegSameSide = swingLeg.getSameSideQuadrant();
      RobotQuadrant nextEdgeQuadrant = tempCommonSupportPolygon.getNextClockwiseSupportingQuadrant(swingLegSameSide);
      RobotQuadrant previousEdgeQuadrant = tempCommonSupportPolygon.getNextCounterClockwiseSupportingQuadrant(swingLegSameSide);

      RobotQuadrant frontEdgeQuadrant;
      RobotQuadrant sideEdgeQuadrant;
      RobotQuadrant hindEdgeQuadrant;
      if (swingLeg.isQuadrantOnLeftSide())
      {
         frontEdgeQuadrant = swingLegSameSide;
         sideEdgeQuadrant = nextEdgeQuadrant;
         hindEdgeQuadrant = previousEdgeQuadrant;
      }
      else
      {
         frontEdgeQuadrant = previousEdgeQuadrant;
         sideEdgeQuadrant = nextEdgeQuadrant;
         hindEdgeQuadrant = swingLegSameSide;
      }

      tempCommonSupportPolygon.getShrunkenPolygon2d(shrunkenCommonPolygonToPack, frontEdgeQuadrant, frontDistance);
      tempCommonSupportPolygon.set(shrunkenCommonPolygonToPack);
      tempCommonSupportPolygon.getShrunkenPolygon2d(shrunkenCommonPolygonToPack, sideEdgeQuadrant, sideDistance);
      tempCommonSupportPolygon.set(shrunkenCommonPolygonToPack);
      tempCommonSupportPolygon.getShrunkenPolygon2d(shrunkenCommonPolygonToPack, hindEdgeQuadrant, hindDistance);
   }

   /**
    * Shrinks all sides of the polygon.
    *
    * @param distance to shrink
    */
   public void shrinkPolygon2d(double distance)
   {
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         getShrunkenPolygon2d(this, robotQuadrant, distance);
      }
   }

   /**
    * Shrinks one side of the polygon.
    *
    * @param distance to shrink
    */
   public void shrinkPolygon2d(RobotQuadrant robotQuadrant, double distance)
   {
      getShrunkenPolygon2d(this, robotQuadrant, distance);
   }

   /**
    * Shrinks one side of the polygon.
    *
    * @param shrunkenPolygonToPack
    * @param sideToShrink side to shrink is the clockwise connected edge to the provided quadrant (ex. FRONT_LEFT -> TOP)
    * @param distance
    */
   public void getShrunkenPolygon2d(QuadrupedSupportPolygon shrunkenPolygonToPack, RobotQuadrant sideToShrink, double distance)
   {
      shrunkenPolygonToPack.set(this);

      if (getNumberOfVertices() >= 3)
      {
         RobotQuadrant nextEdgeQuadrant = getNextClockwiseSupportingQuadrant(sideToShrink);
         RobotQuadrant previousEdgeQuadrant = getNextCounterClockwiseSupportingQuadrant(sideToShrink);

         FramePoint3D originalShrinkEdgeFoot = getFootstep(sideToShrink);
         FramePoint3D originalNextEdgeFoot = getFootstep(nextEdgeQuadrant);
         FramePoint3D originalPreviousEdgeFoot = getFootstep(previousEdgeQuadrant);

         FramePoint3D shrunkenShrinkEdgeFoot = shrunkenPolygonToPack.getFootstep(sideToShrink);
         FramePoint3D shrunkenNextEdgeFoot = shrunkenPolygonToPack.getFootstep(nextEdgeQuadrant);

         FrameVector3D shrinkDirection = tempPlaneNormalInWorld;
         shrinkDirection.sub(originalShrinkEdgeFoot, originalNextEdgeFoot);
         GeometryTools.getPerpendicularVector2d(shrinkDirection, shrinkDirection);
         shrinkDirection.normalize();
         shrinkDirection.scale(distance);

         shrunkenShrinkEdgeFoot.add(shrinkDirection);
         shrunkenNextEdgeFoot.add(shrinkDirection);

         GeometryTools.getIntersectionBetweenTwoLines2d(shrunkenNextEdgeFoot, shrunkenShrinkEdgeFoot, originalPreviousEdgeFoot, originalShrinkEdgeFoot,
                                                        shrunkenShrinkEdgeFoot);
         GeometryTools.getIntersectionBetweenTwoLines2d(shrunkenShrinkEdgeFoot, shrunkenNextEdgeFoot, originalPreviousEdgeFoot, originalNextEdgeFoot,
                                                        shrunkenNextEdgeFoot);
      }
      else if (getNumberOfVertices() == 2)
      {
         RobotQuadrant nextEdgeQuadrant = getNextClockwiseSupportingQuadrant(sideToShrink);
         RobotQuadrant shrinkQuadrant = getNextClockwiseSupportingQuadrant(nextEdgeQuadrant);

         FramePoint3D shrinkFootstep = getFootstep(shrinkQuadrant);
         FramePoint3D shrinkTowardsFootstep = getFootstep(nextEdgeQuadrant);

         FrameVector3D shrinkVector = tempVectorsForInCirclePoint[0];
         shrinkVector.sub(shrinkTowardsFootstep, shrinkFootstep);
         shrinkVector.normalize();
         shrinkVector.scale(distance);
         shrinkFootstep.add(shrinkVector);
      }
      else
      {
         throw new UndefinedOperationException("Can only shrink 2, 3 or 4 sided polygons");
      }
   }

   /**
    * getLeftTrotLeg: returns the left leg of the polyogn if the polygon is a ValidTrotPolygon.
    * If the polygon is not a ValidTrotPolygon, then it throws an exception
    *
    * @return LegName
    */
   public RobotQuadrant getLeftTrotLeg()
   {
      if (!QuadrupedSupportPolygonTools.isValidTrotPolygon(this))
         throw new RuntimeException("SupportPolygon is not a ValidTrotPolygon");

      RobotQuadrant firstLeg = getFirstSupportingQuadrant();
      if (firstLeg.isQuadrantOnLeftSide())
      {
         return firstLeg;
      }

      return firstLeg.getDiagonalOppositeQuadrant();
   }

   /**
    * getRightTrotLeg: returns the left leg of the polygon if the polygon is a ValidTrotPolygon.
    * If the polygon is not a ValidTrotPolygon, then it throws an exception
    *
    * @return LegName
    */
   public RobotQuadrant getRightTrotLeg()
   {
      if (!QuadrupedSupportPolygonTools.isValidTrotPolygon(this))
         throw new RuntimeException("SupportPolygon is not a ValidTrotPolygon");

      RobotQuadrant firstLeg = getFirstSupportingQuadrant();
      if (firstLeg.isQuadrantOnRightSide())
      {
         return firstLeg;
      }

      return firstLeg.getDiagonalOppositeQuadrant();
   }

   /**
    * Gets distance from P1 to trotLine specified by front quadrant.
    */
   public double getDistanceFromP1ToTrotLineInDirection2d(RobotQuadrant trotQuadrant, FramePoint3DReadOnly p1, FramePoint3DReadOnly p2)
   {
      boolean intersectionExists = GeometryTools
            .getIntersectionBetweenTwoLines2d(p1, p2, getFootstep(trotQuadrant), getFootstep(trotQuadrant.getDiagonalOppositeQuadrant()), tempIntersection);

      if (intersectionExists)
      {
         //beacuse this is a 2d method, set Z's to be the same
         tempIntersection.setZ(p1.getZ());
         return tempIntersection.distance(p1);
      }
      else
      {
         return Double.POSITIVE_INFINITY;
      }
   }

   /**
    * Gets the center of a circle of radius in the corner of the triangle.
    * Returns true if the requested radius is greater than the in circle radius.
    *
    * This only works on triangles and must have getNumberOfVertices of 3.
    *
    * @param cornerToPutCircle
    * @param cornerCircleRadius
    * @param centerToPack
    * @return false if the center passed the centroid
    */
   public boolean getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant cornerToPutCircle, double cornerCircleRadius,
                                                                                           FramePoint2D centerToPack)
   {
      if (getNumberOfVertices() == 3)
      {
         if (cornerCircleRadius <= getInCircleRadius2d())
         {
            getCenterOfCircleOfRadiusInCornerOfPolygon(cornerToPutCircle, cornerCircleRadius, centerToPack);

            return true;
         }
         else
         {
            return false;
         }
      }
      else
      {
         throw new UndefinedOperationException("Triangle getNumberOfVertices must be 3. getNumberOfVertices = " + getNumberOfVertices());
      }
   }

   /**
    * Gets the center point of a circle of radius in the corner of a polygon.
    *
    * This polygon must have getNumberOfVertices of 3 or 4.
    *
    * @param cornerToPutCircle
    * @param radius
    * @param centerToPack
    */
   public void getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant cornerToPutCircle, double cornerCircleRadius, FramePoint2D centerToPack)
   {
      if (containsFootstep(cornerToPutCircle))
      {
         if (getNumberOfVertices() >= 3)
         {
            // Corner and A and B form a V with corner as the vertex
            FramePoint3D cornerPoint = getFootstep(cornerToPutCircle);
            FramePoint3D pointA = getFootstep(getNextClockwiseSupportingQuadrant(cornerToPutCircle));
            FramePoint3D pointB = getFootstep(getNextCounterClockwiseSupportingQuadrant(cornerToPutCircle));

            double cornerToA = cornerPoint.distance(pointA);
            double cornerToB = cornerPoint.distance(pointB);
            double aToB = pointA.distance(pointB);

            double theta = EuclidGeometryTools.unknownTriangleAngleByLawOfCosine(cornerToA, cornerToB, aToB);

            Point2D tempCorner = tempPointsForCornerCircle[0];
            Point2D tempA = tempPointsForCornerCircle[1];
            Point2D tempB = tempPointsForCornerCircle[2];

            tempCorner.set(cornerPoint);
            tempA.set(pointA);
            tempB.set(pointB);

            double bisectTheta = 0.5 * theta;

            double radiusOffsetAlongBisector = cornerCircleRadius * (Math.sin(Math.PI / 2.0) / Math.sin(bisectTheta));
            Point2D adjacentBisector = tempPointsForCornerCircle[3];
            EuclidGeometryTools.triangleBisector2D(tempA, tempCorner, tempB, adjacentBisector);

            Vector2D bisectorVector = tempVectorForCornerCircle;
            bisectorVector.set(adjacentBisector.getX() - cornerPoint.getX(), adjacentBisector.getY() - cornerPoint.getY());
            double scalar = radiusOffsetAlongBisector / bisectorVector.length();

            bisectorVector.scale(scalar);

            tempCorner.add(bisectorVector);
            centerToPack.set(tempCorner);
         }
         else
         {
            throw new UndefinedOperationException("Polygon getNumberOfVertices must be 3 or 4. getNumberOfVertices = " + getNumberOfVertices());
         }
      }
      else
      {
         throw new UndefinedOperationException("Polygon must contain " + cornerToPutCircle);
      }
   }

   public boolean epsilonEquals(QuadrupedSupportPolygon polyTwo)
   {
      if (polyTwo == null)
         return false;

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint3D thisFootstep = getFootstep(quadrant);
         FramePoint3D otherFootstep = polyTwo.getFootstep(quadrant);

         if (otherFootstep == null || !thisFootstep.epsilonEquals(otherFootstep, 0.005))
         {
            return false;
         }
      }

      return super.epsilonEquals(polyTwo, 0.005);
   }
}
