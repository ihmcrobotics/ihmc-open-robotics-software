package us.ihmc.quadrupedRobotics.supportPolygon;

import static us.ihmc.robotics.robotSide.RobotQuadrant.FRONT_LEFT;
import static us.ihmc.robotics.robotSide.RobotQuadrant.FRONT_RIGHT;
import static us.ihmc.robotics.robotSide.RobotQuadrant.HIND_LEFT;
import static us.ihmc.robotics.robotSide.RobotQuadrant.HIND_RIGHT;
import static us.ihmc.robotics.robotSide.RobotQuadrant.getQuadrant;

import java.io.Serializable;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class QuadrupedSupportPolygon implements Serializable
{
   private static final long serialVersionUID = 4247638266737494462L;
   
   private final QuadrantDependentFootstepList footsteps = new QuadrantDependentFootstepList();
   
   private final FrameConvexPolygon2d tempFrameConvexPolygon2d = new FrameConvexPolygon2d();
   
   private final FramePoint temporaryFramePoint = new FramePoint();
   private final FramePoint temporaryFramePoint2 = new FramePoint();
   private final FrameVector tempPlaneNormalInWorld = new FrameVector();
   
   private final FrameVector[] tempVectorsForInCirclePoint = new FrameVector[] {new FrameVector(), new FrameVector(), new FrameVector(), new FrameVector()};
   private final FramePoint[] tempPointListForInCirclePoint = new FramePoint[4];
   private final FramePoint tempInCircleCenter = new FramePoint();
   
   private final FrameVector[] tempVectorsForCommonSupportPolygon = new FrameVector[] {new FrameVector(), new FrameVector()};

   public QuadrupedSupportPolygon()
   {
      
   }
   
   public QuadrupedSupportPolygon(QuadrantDependentList<FramePoint> footsteps)
   {
      for (RobotQuadrant robotQuadrant : footsteps.quadrants())
      {
         setFootstep(robotQuadrant, footsteps.get(robotQuadrant));
      }
   }

   /**
    * Copies the support polygon with copies of all of the FramePoints.
    * 
    * @param polygon to copy
    */
   public QuadrupedSupportPolygon(QuadrupedSupportPolygon polygon) 
   {
      for (RobotQuadrant robotQuadrant : polygon.getSupportingQuadrantsInOrder())
      {
         footsteps.set(robotQuadrant, polygon.getFootstep(robotQuadrant));
      }
   }
   
   private class QuadrantDependentFootstepList extends QuadrantDependentList<FramePoint>
   {
      private final FramePoint[] framePointsForStorageWhenNull = new FramePoint[4];
      {
         framePointsForStorageWhenNull[0] = new FramePoint();
         framePointsForStorageWhenNull[1] = new FramePoint();
         framePointsForStorageWhenNull[2] = new FramePoint();
         framePointsForStorageWhenNull[3] = new FramePoint();
      }
      
      public QuadrantDependentFootstepList()
      {
         super();
      }
      
      @Override
      public void set(RobotQuadrant robotQuadrant, FramePoint element)
      {
         // do nothing
         if (element == get(robotQuadrant))
         {
            return;
         }
         // remove
         if (element == null && containsQuadrant(robotQuadrant))
         {
            super.set(robotQuadrant, element);
            return;
         }
         // add
         else if (element != null && !containsQuadrant(robotQuadrant))
         {
            FramePoint storageWhenNull = framePointsForStorageWhenNull[robotQuadrant.ordinal()];
            storageWhenNull.setIncludingFrame(element);
            super.set(robotQuadrant, storageWhenNull);
            return;
         }
         // replace
         else if (element != get(robotQuadrant))
         {
            get(robotQuadrant).setIncludingFrame(element);
         }
      }
      
      @Override
      public FramePoint remove(RobotQuadrant robotQuadrant)
      {
         // remove
         if (containsQuadrant(robotQuadrant))
         {
            framePointsForStorageWhenNull[robotQuadrant.ordinal()] = get(robotQuadrant);
            return super.remove(robotQuadrant);
         }
         // do nothing
         else
         {
            return get(robotQuadrant);
         }
      }
      
      @Override
      public FramePoint get(RobotQuadrant key)
      {
         return super.get(key);
      }

      @Override
      public void clear()
      {
         for (RobotQuadrant robotQuadrant : quadrants())
         {
            framePointsForStorageWhenNull[robotQuadrant.ordinal()] = get(robotQuadrant);
         }
         
         super.clear();
      }
   }
   
   public void printOutPolygon(String string)
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
      for(RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         string += "\n" + robotQuadrant + " " + getFootstep(robotQuadrant);
      }
      return string;
   }

   /**
    * @return the reference frame of the first non-null footstep.
    */
   public ReferenceFrame getReferenceFrame()
   {
      return getFootstep(getFirstSupportingQuadrant()).getReferenceFrame();
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         getFootstep(robotQuadrant).changeFrame(referenceFrame);
      }
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
      int numberOfLegs = size();
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
            return new RobotQuadrant[][]
            {
               {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT},
               {RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT},
               {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT}
            };
         case FRONT_RIGHT:
            return new RobotQuadrant[][]
            {
               {RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_RIGHT},
               {RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT},
               {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT}
            };
         case HIND_RIGHT:
            return new RobotQuadrant[][]
            {
               {RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT},
               {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_LEFT},
               {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT}
            };
         case HIND_LEFT:
            return new RobotQuadrant[][]
            {
               {RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT},
               {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT},
               {RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_LEFT}
            };
         }
      case 4:
         return new RobotQuadrant[][]
         {
            {RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT},
            {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT},
            {RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT},
            {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT}
         };
      default:
         throw new RuntimeException();
      }
   }
   
   public int size()
   {
      return footsteps.size();
   }
   
   public RobotQuadrant[] getSupportingQuadrantsInOrder()
   {
      return footsteps.quadrants();
   }

   public RobotQuadrant getFirstSupportingQuadrant()
   {
      if (containsFootstep(RobotQuadrant.FRONT_LEFT)) // begin punching nanos
         return RobotQuadrant.FRONT_LEFT;
      else if (containsFootstep(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (containsFootstep(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (containsFootstep(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else
         throw new EmptySupportPolygonException();
   }
   
   public RobotQuadrant getFirstNonSupportingQuadrant()
   {
      if (!containsFootstep(RobotQuadrant.FRONT_LEFT))
         return RobotQuadrant.FRONT_LEFT;
      else if (!containsFootstep(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (!containsFootstep(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (!containsFootstep(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else
         throw new RuntimeException("Polygon is full");
   }
   
   public RobotQuadrant getLastSupportingQuadrant()
   {
      if (containsFootstep(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else if (containsFootstep(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (containsFootstep(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (containsFootstep(RobotQuadrant.FRONT_LEFT))
         return RobotQuadrant.FRONT_LEFT;
      else
         throw new EmptySupportPolygonException();
   }

   public RobotQuadrant getLastNonSupportingQuadrant()
   {
      if (!containsFootstep(RobotQuadrant.HIND_LEFT))
         return RobotQuadrant.HIND_LEFT;
      else if (!containsFootstep(RobotQuadrant.HIND_RIGHT))
         return RobotQuadrant.HIND_RIGHT;
      else if (!containsFootstep(RobotQuadrant.FRONT_RIGHT))
         return RobotQuadrant.FRONT_RIGHT;
      else if (!containsFootstep(RobotQuadrant.FRONT_LEFT))
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
         if (containsFootstep(prospectiveQuadrant))
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
         if (containsFootstep(prospectiveQuadrant))
            return prospectiveQuadrant;
      }
      
      throw new EmptySupportPolygonException();
   }
    
   public FramePoint getFootstep(RobotQuadrant robotQuadrant)
   {
      return footsteps.get(robotQuadrant);
   }
   
   public FramePoint getFootstepOrCreateIfNonSupporting(RobotQuadrant robotQuadrant)
   {
      if  (!containsFootstep(robotQuadrant))
         setFootstep(robotQuadrant, new FramePoint());
      
      return getFootstep(robotQuadrant);
   }
   
   public void set(QuadrupedSupportPolygon polygon)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         setFootstep(robotQuadrant, polygon.getFootstep(robotQuadrant));
      }
   }

   public void setFootstep(RobotQuadrant robotQuadrant, FramePoint footstep)
   {
      footsteps.set(robotQuadrant, footstep);
   }

   public void removeFootstep(RobotQuadrant robotQuadrant)
   {
      footsteps.remove(robotQuadrant);
   }

   public void clear()
   {
      footsteps.clear();
   }

   /**
    * Replaces the stored footstep with the passed in one.
    * 
    * @param support polygon to pack
    * @param quadrant to replace
    * @param resulting footstep
    */
   public void getAndReplaceFootstep(QuadrupedSupportPolygon supportPolygonToPack, RobotQuadrant quadrant, FramePoint footstep)
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
   public void translate(Vector3d translateBy)
   {
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         getFootstep(robotQuadrant).add(translateBy);
      }
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
   }

   /**
    * Rotates the feet about the Centroid, keeping the z heights.
    *
    * @return SupportPolygon
    */
   public void yawAboutCentroid(double yaw)
   {
      getCentroid2d(temporaryFramePoint);
   
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint footstep = getFootstep(quadrant);
         if (containsFootstep(footstep))
         {
            FramePoint rotatedPoint = new FramePoint();
            footstep.yawAboutPoint(temporaryFramePoint, rotatedPoint, yaw);
            footstep.set(rotatedPoint);
         }
      }
   }

   private boolean containsFootstep(FramePoint footstep)
   {
      if (footstep == null)
         return false;
      else
         return true;
   }

   public boolean containsFootstep(RobotQuadrant robotQuadrant)
   {
      return footsteps.containsQuadrant(robotQuadrant);
   }

   public void packYoFrameConvexPolygon2d(YoFrameConvexPolygon2d yoFrameConvexPolygon2d)
   {      
      tempFrameConvexPolygon2d.clear();
      tempFrameConvexPolygon2d.changeFrame(getReferenceFrame());
      for (RobotQuadrant supportingQuadrant : getSupportingQuadrantsInOrder())
      {
         tempFrameConvexPolygon2d.addVertexByProjectionOntoXYPlane(getFootstep(supportingQuadrant));
      }
      tempFrameConvexPolygon2d.update();

      yoFrameConvexPolygon2d.setFrameConvexPolygon2d(tempFrameConvexPolygon2d);
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

   public RobotQuadrant getClosestFootstep(FramePoint pointToCompare)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      RobotQuadrant closestQuadrant = null;
      for(RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         double distance = getFootstep(robotQuadrant).distance(pointToCompare);
         if(distance < minDistance)
         {
            closestQuadrant = robotQuadrant;
            minDistance = distance;
         }
      }
      return closestQuadrant;
   }

   public void getCentroid2d(FramePoint centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         centroidToPack.add(getFootstep(robotQuadrant));
      }
      
      centroidToPack.scale(1.0 / size());
   }

   public void getCentroid2d(FramePoint2d centroidToPack2d)
   {
      centroidToPack2d.setToZero();
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         centroidToPack2d.add(getFootstep(robotQuadrant).getX(), getFootstep(robotQuadrant).getY());
      }
      
      centroidToPack2d.scale(1.0 / size());
   }

   /**
    * getBounds modifies the min and max points passed in to the min and max
    * xy values contained in the set of Footsteps that make up the polygon
    *
    * @param minToPack Point2d  Minimum x and y value contained in footsteps list
    * @param maxToPack Point2d  Maximum x and y value contained in footsteps list
    */
   public void getBounds(Point2d minToPack, Point2d maxToPack)
   {
      minToPack.x = minToPack.y = Double.POSITIVE_INFINITY;
      maxToPack.x = maxToPack.y = Double.NEGATIVE_INFINITY;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = getFootstep(robotQuadrant);
         if (containsFootstep(footstep))
         {
            if (footstep.getX() < minToPack.x)
            {
               minToPack.x = footstep.getX();
            }

            if (footstep.getY() < minToPack.y)
            {
               minToPack.y = footstep.getY();
            }

            if (footstep.getX() > maxToPack.x)
            {
               maxToPack.x = footstep.getX();
            }

            if (footstep.getY() > maxToPack.y)
            {
               maxToPack.y = footstep.getY();
            }
         }
      }
   }

   /**
    * Returns true if the given (x,y) value is inside the polygon. This test
    * ignores Z values for the polygon. The test works by computing the minimum
    * distance from each polygon line segment to the point.
    *
    * @param point Point2d
    * @return boolean
    */
   public boolean isInside(FramePoint point)
   {
      if (distanceInside2d(point) > 0.0)
      {
         return true;
      }

      return false;
   }
   
   /**
    * Returns true if the given (x,y) value is inside the polygon. This test
    * ignores Z values for the polygon. The test works by computing the minimum
    * distance from each polygon line segment to the point.
    *
    * @param point Point2d
    * @return boolean
    */
   public boolean isInside(FramePoint2d point)
   {
      temporaryFramePoint.setXY(point);
      if (distanceInside2d(temporaryFramePoint) > 0.0)
      {
         return true;
      }

      return false;
   }

   /**
    * Returns the distance the given (x,y) value is inside the polygon.  This is defined
    * as the minimum distance from the point to each line segment on the polygon.
    * If the point is outside the polygon, the distance will be negative and represent
    * the farthest distance from the point to each edge.
    * This test ignores Z values for the polygon.
    *
    * @param point Point2d
    * @return boolean
    */
   public double distanceInside2d(FramePoint point)
   {
      if (size() == 1)
      {
         return -point.distance(getFootstep(getFirstSupportingQuadrant()));
      }
      else if (size() == 2)
      {
         FramePoint pointOne = getFootstep(getFirstSupportingQuadrant());
         FramePoint pointTwo = getFootstep(getLastSupportingQuadrant());
         return -Math.abs(GeometryTools.distanceFromPointToLine2d(point, pointOne, pointTwo));
      }
      else
      {
         double closestDistance = Double.POSITIVE_INFINITY;
         
         for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
         {
            FramePoint pointOne = getFootstep(robotQuadrant);
            FramePoint pointTwo = getFootstep(getNextClockwiseSupportingQuadrant(robotQuadrant));
            
            double distance = computeDistanceToSideOfSegment(point, pointOne, pointTwo);
            if (distance < closestDistance)
            {
               closestDistance = distance;
            }
         }
         
         return closestDistance;
      }
   }
   
   private double computeDistanceToSideOfSegment(FramePoint point, FramePoint pointOne, FramePoint pointTwo)
   {
      double x0 = point.getX();
      double y0 = point.getY();
      
      double x1 = pointOne.getX();
      double y1 = pointOne.getY();
      
      double x2 = pointTwo.getX();
      double y2 = pointTwo.getY();
      
      double numerator = (y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1;
      double denominator = Math.sqrt((y2-y1) * (y2-y1) + (x2-x1) * (x2-x1));
      
      return numerator/denominator;
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
   public double getInCircle2d(FramePoint inCircleCenterToPack)
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
    * @param p1 Point2d point defining the three line segments (must be in order)
    * @param p2 Point2d point defining the three line segments (must be in order)
    * @param p3 Point2d point defining the three line segments (must be in order)
    * @param p4 Point2d point defining the three line segments (must be in order)
    * @return Point2d incirlce point
    */
   public void getInCirclePoint2d(FramePoint intersectionToPack)
   {
      if (size() < 3)
      {
         throw new UndefinedOperationException("InCirclePoint only defined for 3 and 4 legs. size() = " + size());
      }
      
      int i = 0;
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         tempPointListForInCirclePoint[i++] = getFootstep(robotQuadrant);
      }
      
      if (size() == 3)
      {
         tempPointListForInCirclePoint[3] = tempPointListForInCirclePoint[0];
      }
      
      // get first two line segments
      tempVectorsForInCirclePoint[0].sub(tempPointListForInCirclePoint[0], tempPointListForInCirclePoint[1]);
      tempVectorsForInCirclePoint[0].normalize();
      tempVectorsForInCirclePoint[1].sub(tempPointListForInCirclePoint[2], tempPointListForInCirclePoint[1]);
      tempVectorsForInCirclePoint[1].normalize();

      // normalize and subtract to get vector that bisects the first angle
      FrameVector v2p = tempVectorsForInCirclePoint[0];
      v2p.add(tempVectorsForInCirclePoint[1]);
      v2p.normalize();

      // get second two line segments
      tempVectorsForInCirclePoint[2].sub(tempPointListForInCirclePoint[3], tempPointListForInCirclePoint[2]);
      tempVectorsForInCirclePoint[2].normalize();
      tempVectorsForInCirclePoint[3].sub(tempPointListForInCirclePoint[1], tempPointListForInCirclePoint[2]);
      tempVectorsForInCirclePoint[3].normalize();

      // normalize and subtract to get vector that bisects the second angle
      FrameVector v3p = tempVectorsForInCirclePoint[2];
      v3p.add(tempVectorsForInCirclePoint[3]);
      v3p.normalize();

      // find intersection point of the two bisecting vectors
      GeometryTools.getIntersectionBetweenTwoLines2d(intersectionToPack, tempPointListForInCirclePoint[1], v2p, tempPointListForInCirclePoint[2], v3p);
   }

   /**
    * Get the distance of the point inside of the in circle.
    *
    * @param point
    * @return distance
    */
   public double distanceInsideInCircle2d(FramePoint point)
   {
      double inCircleRadius = getInCircle2d(tempInCircleCenter);
      double distanceToInCircleCenter = point.getXYPlaneDistance(tempInCircleCenter);
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
      if (size() >= 3)
      {
         double deltaX = 0.0;
         double deltaY = 0.0;
         
         if (containsFootstep(FRONT_LEFT) && containsFootstep(HIND_LEFT))
         {
            deltaX += getFootstep(FRONT_LEFT).getX() - getFootstep(HIND_LEFT).getX();
            deltaY += getFootstep(FRONT_LEFT).getY() - getFootstep(HIND_LEFT).getY();
         }
         if (containsFootstep(FRONT_RIGHT) && containsFootstep(HIND_RIGHT))
         {
            deltaX += getFootstep(FRONT_RIGHT).getX() - getFootstep(HIND_RIGHT).getX();
            deltaY += getFootstep(FRONT_RIGHT).getY() - getFootstep(HIND_RIGHT).getY();
         }
         
         return Math.atan2(deltaY, deltaX);
      }
      else
      {
         throw new UndefinedOperationException("Undefined for less than 3 size. size = " + size());
      }
   }

   /**
    * Angle from hind left to hind right footstep.
    */
   public double getNominalYawHindLegs()
   {
      if (containsFootstep(RobotQuadrant.HIND_RIGHT) && containsFootstep(RobotQuadrant.HIND_LEFT))
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
   public double getNominalPitch()
   {
      if (size() >= 3)
      {
         double deltaX = 0.0;
         double deltaY = 0.0;
         double deltaZ = 0.0;
   
         if ((containsFootstep(RobotQuadrant.FRONT_LEFT) && containsFootstep(RobotQuadrant.HIND_LEFT)))
         {
            deltaX += getFootstep(RobotQuadrant.FRONT_LEFT).getX() - getFootstep(RobotQuadrant.HIND_LEFT).getX();
            deltaY += getFootstep(RobotQuadrant.FRONT_LEFT).getY() - getFootstep(RobotQuadrant.HIND_LEFT).getY();
            deltaZ += getFootstep(RobotQuadrant.FRONT_LEFT).getZ() - getFootstep(RobotQuadrant.HIND_LEFT).getZ();
         }
   
         if (containsFootstep(RobotQuadrant.FRONT_RIGHT) && containsFootstep(RobotQuadrant.HIND_RIGHT))
         {
            deltaX += getFootstep(RobotQuadrant.FRONT_RIGHT).getX() - getFootstep(RobotQuadrant.HIND_RIGHT).getX();
            deltaY += getFootstep(RobotQuadrant.FRONT_RIGHT).getY() - getFootstep(RobotQuadrant.HIND_RIGHT).getY();
            deltaZ += getFootstep(RobotQuadrant.FRONT_RIGHT).getZ() - getFootstep(RobotQuadrant.HIND_RIGHT).getZ();
         }
         
         double length = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
         if (length < 1e-3)
            throw new UndefinedOperationException("Polygon is too small");
         
         return -Math.asin(deltaZ / length);
      }
      else
      {
         throw new UndefinedOperationException("Undefined for less than 3 size. size = " + size());
      }
   }

   /**
    * Computes a nominal roll angle. If triangle support, then the angle from the front to back support foot on the same side.
    * If a quad, then the average of the two front to back angles.
    * @return double
    */
   public double getNominalRoll()
   {
      if (size() >= 3)
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
         
         return Math.asin(deltaZ / length);
      }
      else
      {
         throw new UndefinedOperationException("Undefined for less than 3 size. size = " + size());
      }
   }

   /**
    * Returns the average position of the vertices in the polygon
    * corresponding to front foot locations. NaN is front feet aren't
    * supporting.
    *
    * @params framePointToPack
    */
   public void getFrontMidpoint(FramePoint framePointToPack)
   {
      if (containsFootstep(RobotQuadrant.FRONT_LEFT) && containsFootstep(RobotQuadrant.FRONT_RIGHT))
      {
         framePointToPack.setToZero();
         framePointToPack.add(getFootstep(RobotQuadrant.FRONT_LEFT));
         framePointToPack.add(getFootstep(RobotQuadrant.FRONT_RIGHT));
         framePointToPack.scale(0.5);
      }
      else if (containsFootstep(RobotQuadrant.FRONT_LEFT))
      {
         framePointToPack.set(getFootstep(RobotQuadrant.FRONT_LEFT));
      }
      else if (containsFootstep(RobotQuadrant.FRONT_RIGHT))
      {
         framePointToPack.set(getFootstep(RobotQuadrant.FRONT_RIGHT));
      }
      else
      {
         throw new UndefinedOperationException("Polygon must contain a front footstep");
      }
   }

   /**
    * Returns the average position of the vertices in the polygon
    * corresponding to hind foot locations. NaN is hind feet aren't
    * supporting.
    *
    * @params framePointToPack
    */
   public void getHindMidpoint(FramePoint framePointToPack)
   {
      if (containsFootstep(RobotQuadrant.HIND_LEFT) && containsFootstep(RobotQuadrant.HIND_RIGHT))
      {
         framePointToPack.setToZero();
         framePointToPack.add(getFootstep(RobotQuadrant.HIND_LEFT));
         framePointToPack.add(getFootstep(RobotQuadrant.HIND_RIGHT));
         framePointToPack.scale(0.5);
      }
      else if (containsFootstep(RobotQuadrant.HIND_LEFT))
      {
         framePointToPack.set(getFootstep(RobotQuadrant.HIND_LEFT));
      }
      else if (containsFootstep(RobotQuadrant.HIND_RIGHT))
      {
         framePointToPack.set(getFootstep(RobotQuadrant.HIND_RIGHT));
      }
      else
      {
         throw new UndefinedOperationException("Polygon must contain a hind footstep");
      }
   }

   /**
    * getDistancesToSupportLines
    *
    * @param point Point2d
    * @param removeSwingLeg boolean
    * @return double[]
    */
   public double[] getDistancesToSupportLines(Point2d point)
   {
      int numberOfLegs = size();
      int numberOfEdges;

      // Because of the singular case of 2 feet having one edge
      if (numberOfLegs > 2)
      {
         numberOfEdges = numberOfLegs;
      }
      else
      {
         numberOfEdges = numberOfLegs - 1;
      }

      double[] distances = new double[numberOfEdges];

      RobotQuadrant indexStart = getLastSupportingQuadrant();
      RobotQuadrant indexEnd = getFirstSupportingQuadrant();

      // indexStart must be greater than indexEnd
      if (indexStart.ordinal() < indexEnd.ordinal())
      {
         throw new RuntimeException("SupportPolygon.distanceInside(): indexStart must be > indexEnd");
      }

      FramePoint pointStart = getFootstep(indexStart);
      FramePoint pointEnd = getFootstep(indexEnd);

      // FramePoint pointStart = getFootstep(footsteps.size() - 1).getPositionFramePoint();
      // FramePoint pointEnd = getFootstep(0).getPositionFramePoint();

      if ((!pointStart.getReferenceFrame().isWorldFrame()) || (!pointEnd.getReferenceFrame().isWorldFrame()))
      {
         throw new RuntimeException("SupportPolygon.distanceInside() should only be called from a Polygon in a World Frame!!");
      }

      Point2d lineStart = new Point2d();
      Point2d lineEnd = new Point2d();
      pointStart.getPoint2d(lineStart);
      pointEnd.getPoint2d(lineEnd);
      
      boolean onLeftSide = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);
      double distance = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);

      int counter = 0;
      if (onLeftSide)
         distances[counter] = -distance;
      else
         distances[counter] = distance;

      for (int i = indexEnd.ordinal() + 1; i <= indexStart.ordinal(); i++)
      {
         lineStart = lineEnd;
         FramePoint footstep = getFootstep(RobotQuadrant.getQuadrantNameFromOrdinal(i));
         if (containsFootstep(footstep))
         {
            counter++;
            footstep.getPoint2d(lineEnd);

            onLeftSide = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);
            distance = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);
            if (onLeftSide)
               distances[counter] = -distance;
            else
               distances[counter] = distance;
         }
      }

      return distances;
   }

   /**
    * Check if the polygons are same size and contain the same quadrants.
    * 
    * @param polygonToCompare
    * @return contain same quadrants
    */
   public boolean containsSameQuadrants(QuadrupedSupportPolygon polygonToCompare)
   {
      if (size() != polygonToCompare.size())
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
    *  This method returns the bisector point
    *  Given a triangle polygon, the bisector point is defined as
    *  the point that intersects the opposite side from the line bisecting the
    *  angle formed on the support leg that is on the same side as the swing leg
    *
    *  (see Geometry tools for math)
    *
    * @return FramePoint bisector point
    */
   public FramePoint getBisectorPoint()
   {
      // verify has exactly three legs
      if (size() != 3)
         throw new IllegalArgumentException("This supportPolygon must contain exactly three legs not " + size());

      RobotQuadrant nonSupportingQuadrant = getFirstNonSupportingQuadrant();
      Point2d A = new Point2d();
      getFootstep(nonSupportingQuadrant.getAcrossBodyQuadrant()).getPoint2d(A);
      Point2d B = new Point2d();
      getFootstep(nonSupportingQuadrant.getSameSideQuadrant()).getPoint2d(B);
      Point2d C = new Point2d();
      getFootstep(nonSupportingQuadrant.getDiagonalOppositeQuadrant()).getPoint2d(C);
      Point2d bisectingPoint = GeometryTools.getTriangleBisector(A, B, C);
      
      return new FramePoint(getReferenceFrame(), bisectingPoint.x, bisectingPoint.y, 0.0);
   }
   
   /**
    *  getStanceWidthFrontLegs
    * 
    *  @return double
    */
   public double getStanceLength(RobotSide robotSide)
   {
      if (size() != 4)
      {
         throw new RuntimeException("Need 4 legs for SupportPolygon.getStanceWidthFrontLegs()");
      }

      FramePoint frontFootstep = getFootstep(RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide));
      FramePoint endFootstep = getFootstep(RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide));

      return frontFootstep.distance(endFootstep);
   }
   
   /**
    *  getStanceWidthFrontLegs
    * 
    *  @return double
    */
   public double getStanceWidthFrontLegs()
   {
      if (!containsFootstep(RobotQuadrant.FRONT_LEFT) || !containsFootstep(RobotQuadrant.FRONT_RIGHT))
         throw new RuntimeException("Must have both front feet.");
      
      return getFootstep(RobotQuadrant.FRONT_LEFT).distance(getFootstep(RobotQuadrant.FRONT_RIGHT));
   }

   /**
    * getStanceWidthHindLegs
    *
    * @return double
    */
   public double getStanceWidthHindLegs()
   {
      if (!containsFootstep(RobotQuadrant.HIND_LEFT) || !containsFootstep(RobotQuadrant.HIND_RIGHT))
         throw new RuntimeException("Must have both hind feet.");
      
      return getFootstep(RobotQuadrant.HIND_LEFT).distance(getFootstep(RobotQuadrant.HIND_RIGHT));
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
   public void getCommonTriangle2d(QuadrupedSupportPolygon polygonToCompare, QuadrupedSupportPolygon commonPolygonToPack, RobotQuadrant quadrantToAssignToIntersection)
   {
      // verify both have exactly three legs
      if (size() != 3)
         throw new UndefinedOperationException("This supportPolygon must contain exactly three legs, not " + size());
      if (polygonToCompare.size() != 3)
         throw new UndefinedOperationException("Supplied supportPolygon must contain exactly three legs, not " + polygonToCompare.size());
      // verify exactly two legs epsilon match
      if (getNumberOfEqualFootsteps(polygonToCompare) != 2)
         throw new UndefinedOperationException("There must be exactly two similar foosteps not " + getNumberOfEqualFootsteps(polygonToCompare));

      // return null if swing legs are not same side *** Assumes regular gait ***
      RobotQuadrant thisSwingLeg = getFirstNonSupportingQuadrant();
      RobotQuadrant compareSwingLeg = polygonToCompare.getFirstNonSupportingQuadrant();

      // verify specified swing leg name is one of the swinging (same side) leg names
      if (quadrantToAssignToIntersection != thisSwingLeg && quadrantToAssignToIntersection != compareSwingLeg)
         throw new UndefinedOperationException("The specified intersection quadrant must be one of the swinging (same side) leg names");
      
      FrameVector direction1 = tempVectorsForCommonSupportPolygon[0];
      direction1.sub(getFootstep(compareSwingLeg.getDiagonalOppositeQuadrant()), getFootstep(compareSwingLeg));
      
      FrameVector direction2 = tempVectorsForCommonSupportPolygon[1];
      direction2.sub(polygonToCompare.getFootstep(thisSwingLeg.getDiagonalOppositeQuadrant()), polygonToCompare.getFootstep(thisSwingLeg));
      
      commonPolygonToPack.clear();
      FramePoint intersection = commonPolygonToPack.getFootstepOrCreateIfNonSupporting(quadrantToAssignToIntersection);
      GeometryTools.getIntersectionBetweenTwoLines2d(intersection, getFootstep(compareSwingLeg), direction1, polygonToCompare.getFootstep(thisSwingLeg), direction2);
      
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
         if (getFootstep(robotQuadrant).epsilonEquals(polygonToCompare.getFootstep(robotQuadrant), 0.0005))
         {
            ++numberOfEqual;
         }
      }
   
      return numberOfEqual;
   }

   /**
    *  This method returns the common support polygon bewteen this and the supplied supportPolygon
    *  shrunken by the specified amounts.  Each amount is the distance in meters the edge should be
    *  moved in parallel to itself.
    *
    *  *** Assumes regular gait 
    *
    * @param nextSupportPolygon SupportPolygon
    *        1) must contain only three legs
    *        2) exactly two footsteps must be the same
    * @param quadrantForIntersection LegName is the legname to assign to the intersection footstep
    *        3) must be one of the swinging (same side) leg names
    * @param frontDistance double  distance to shrink the front side
    * @param sideDistance double  distance to shrink the side side
    * @param hindDistance double  distance to shrink the hind side
    *
    * @return SupportPolygon That is common to both (can be null for opposite side swing legs)
    *        This should be the two matching feet and the intersection of the two tror lines
    *        Each side is shrunken by the specified distance.
    *        If the remaining distance is insufficent to shrink, then return null
    */
   public void getShrunkenCommonTriangle2d(QuadrupedSupportPolygon nextSupportPolygon, QuadrupedSupportPolygon shrunkenCommonPolygonToPack,
         RobotQuadrant quadrantForIntersection, double frontDistance, double sideDistance, double hindDistance)
   {
      QuadrupedSupportPolygon commonSupportPolygon = new QuadrupedSupportPolygon();
      getCommonTriangle2d(nextSupportPolygon, commonSupportPolygon, quadrantForIntersection);
      
      shrunkenCommonPolygonToPack.set(commonSupportPolygon);
      
      RobotQuadrant swingLeg = commonSupportPolygon.getFirstNonSupportingQuadrant();
      RobotQuadrant swingLegSameSide = swingLeg.getSameSideQuadrant();
      RobotQuadrant nextEdgeQuadrant = commonSupportPolygon.getNextClockwiseSupportingQuadrant(swingLegSameSide);
      RobotQuadrant previousEdgeQuadrant = commonSupportPolygon.getNextCounterClockwiseSupportingQuadrant(swingLegSameSide);
      
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
      
      commonSupportPolygon.getShrunkenPolygon2d(shrunkenCommonPolygonToPack, frontEdgeQuadrant, frontDistance);
      commonSupportPolygon.set(shrunkenCommonPolygonToPack);
      commonSupportPolygon.getShrunkenPolygon2d(shrunkenCommonPolygonToPack, sideEdgeQuadrant, sideDistance);
      commonSupportPolygon.set(shrunkenCommonPolygonToPack);
      commonSupportPolygon.getShrunkenPolygon2d(shrunkenCommonPolygonToPack, hindEdgeQuadrant, hindDistance);
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
      
      if (size() >= 3)
      {
         RobotQuadrant nextEdgeQuadrant = getNextClockwiseSupportingQuadrant(sideToShrink);
         RobotQuadrant previousEdgeQuadrant = getNextCounterClockwiseSupportingQuadrant(sideToShrink);
         
         FramePoint originalShrinkEdgeFoot = getFootstep(sideToShrink);
         FramePoint originalNextEdgeFoot = getFootstep(nextEdgeQuadrant);
         FramePoint originalPreviousEdgeFoot = getFootstep(previousEdgeQuadrant);
         
         FramePoint shrunkenShrinkEdgeFoot = shrunkenPolygonToPack.getFootstep(sideToShrink);
         FramePoint shrunkenNextEdgeFoot = shrunkenPolygonToPack.getFootstep(nextEdgeQuadrant);
         
         FrameVector shrinkDirection = tempPlaneNormalInWorld;
         shrinkDirection.sub(originalShrinkEdgeFoot, originalNextEdgeFoot);
         GeometryTools.getPerpendicularVector2d(shrinkDirection, shrinkDirection);
         shrinkDirection.normalize();
         shrinkDirection.scale(distance);
         
         shrunkenShrinkEdgeFoot.add(shrinkDirection);
         shrunkenNextEdgeFoot.add(shrinkDirection);
         
         GeometryTools.getIntersectionBetweenTwoLines2d(shrunkenShrinkEdgeFoot, shrunkenNextEdgeFoot, shrunkenShrinkEdgeFoot, originalPreviousEdgeFoot, originalShrinkEdgeFoot);
         GeometryTools.getIntersectionBetweenTwoLines2d(shrunkenNextEdgeFoot, shrunkenShrinkEdgeFoot, shrunkenNextEdgeFoot, originalPreviousEdgeFoot, originalNextEdgeFoot);
      }
      else
      {
         throw new UndefinedOperationException("Can only shrink 3 or 4 side polygon");
      }
   }
   
   /**
    *  If the two polygons differ only in that one footstep has moved, return that quadrant.
    *  
    * @param next polygon
    * @return quadrant that has moved
    */
   public RobotQuadrant getWhichFootstepHasMoved(QuadrupedSupportPolygon nextPolygon)
   {
      if (!containsSameQuadrants(nextPolygon))
      {
         throw new IllegalArgumentException("Polygons contain different quadrants");
      }
      
      RobotQuadrant swingLeg = null;
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         if (!getFootstep(robotQuadrant).epsilonEquals(nextPolygon.getFootstep(robotQuadrant), 1e-5))
         {
            if (swingLeg == null)
            {
               swingLeg = robotQuadrant;
            }
            else // make sure only one foot differs
            {
               throw new IllegalArgumentException("More than one foot differs");
            }
         }
      }
      
      if (swingLeg == null)
      {
         throw new IllegalArgumentException("No feet were different");
      }
      
      return swingLeg;
   }

   public double distanceInsideTrotLine(FramePoint framePoint)
   {
      if (size() == 4)
         return 0.0;

      RobotQuadrant swingLeg = getFirstNonSupportingQuadrant();

      FramePoint pointStart = getFootstep(swingLeg.getAcrossBodyQuadrant());
      FramePoint pointEnd = getFootstep(swingLeg.getSameSideQuadrant());

      temporaryFramePoint.set(pointStart);
      temporaryFramePoint2.set(pointEnd);

      boolean inside = GeometryTools.isPointOnLeftSideOfLine(framePoint, temporaryFramePoint, temporaryFramePoint2);

      if ((swingLeg == RobotQuadrant.HIND_LEFT) || (swingLeg == RobotQuadrant.FRONT_RIGHT))
      {
         inside = !inside;
      }

      double distance = GeometryTools.distanceFromPointToLine2d(framePoint, temporaryFramePoint, temporaryFramePoint2);

      if (!inside)
         distance = -distance;

      return distance;
   }

   public void getTrotLineMiddle(FramePoint framePointToPack)
   {
      RobotQuadrant swingLeg = getFirstNonSupportingQuadrant();

      FramePoint footstep1 = getFootstep(swingLeg.getAcrossBodyQuadrant());
      FramePoint footstep2 = getFootstep(swingLeg.getSameSideQuadrant());

      framePointToPack.set(footstep1);
      framePointToPack.add(footstep2);

      framePointToPack.scale(0.5);
   }

   public void get2DVectorPerpendicularToTrotLinePointingInside(FrameVector frameVectorToPack)
   {
      RobotQuadrant swingLeg = getFirstNonSupportingQuadrant();

      FramePoint footstep1 = getFootstep(swingLeg.getAcrossBodyQuadrant());
      FramePoint footstep2 = getFootstep(swingLeg.getSameSideQuadrant());

      frameVectorToPack.set(footstep2);
      frameVectorToPack.sub(footstep1);

      frameVectorToPack.setZ(0.0);
      double x = frameVectorToPack.getX();
      double y = frameVectorToPack.getY();

      if (swingLeg == RobotQuadrant.HIND_LEFT)
      {
         frameVectorToPack.setX(y);
         frameVectorToPack.setY(-x);
         frameVectorToPack.normalize();
      }
      else if (swingLeg == RobotQuadrant.HIND_RIGHT)
      {
         frameVectorToPack.setX(-y);
         frameVectorToPack.setY(x);
         frameVectorToPack.normalize();
      }
      else
      {
         throw new RuntimeException("Not implemented for fronts!");
      }
   }

   /**
    * Returns true if any of the legs are crossing.
    * By this what we mean is that as you trace out a path from FL to FR to HR to HL,
    * considering only the x,y coordinates, that the path will only take right turns
    * and no left turns.
    * @return true if any of the legs are crossing. Otherwise false.
    */
   public boolean areLegsCrossing()
   {
      // JEP: This is hardcoded like this instead of using points and vectors because it needs
      // to be really fast I think. I could be wrong though.

      double xFL, yFL, xFR, yFR, xHR, yHR, xHL, yHL;
      if (size() == 4)
      {
         xFL = getFootstep(RobotQuadrant.FRONT_LEFT).getX();
         yFL = getFootstep(RobotQuadrant.FRONT_LEFT).getY();
                       
         xFR = getFootstep(RobotQuadrant.FRONT_RIGHT).getX();
         yFR = getFootstep(RobotQuadrant.FRONT_RIGHT).getY();
                       
         xHR = getFootstep(RobotQuadrant.HIND_RIGHT).getX();
         yHR = getFootstep(RobotQuadrant.HIND_RIGHT).getY();
                       
         xHL = getFootstep(RobotQuadrant.HIND_LEFT).getX();
         yHL = getFootstep(RobotQuadrant.HIND_LEFT).getY();
      }
      else
      {
         xFL = (getFootstep(RobotQuadrant.FRONT_LEFT) == null) ? Double.NaN : getFootstep(RobotQuadrant.FRONT_LEFT).getX();
         yFL = (getFootstep(RobotQuadrant.FRONT_LEFT) == null) ? Double.NaN : getFootstep(RobotQuadrant.FRONT_LEFT).getY();
         
         xFR = (getFootstep(RobotQuadrant.FRONT_RIGHT) == null) ? Double.NaN : getFootstep(RobotQuadrant.FRONT_RIGHT).getX();
         yFR = (getFootstep(RobotQuadrant.FRONT_RIGHT) == null) ? Double.NaN : getFootstep(RobotQuadrant.FRONT_RIGHT).getY();
                
         xHR = (getFootstep(RobotQuadrant.HIND_RIGHT) == null) ? Double.NaN : getFootstep(RobotQuadrant.HIND_RIGHT).getX();
         yHR = (getFootstep(RobotQuadrant.HIND_RIGHT) == null) ? Double.NaN : getFootstep(RobotQuadrant.HIND_RIGHT).getY();
                
         xHL = (getFootstep(RobotQuadrant.HIND_LEFT) == null) ? Double.NaN : getFootstep(RobotQuadrant.HIND_LEFT).getX();
         yHL = (getFootstep(RobotQuadrant.HIND_LEFT) == null) ? Double.NaN : getFootstep(RobotQuadrant.HIND_LEFT).getY();
      }

      double xFLtoFR = xFR - xFL;
      double yFLtoFR = yFR - yFL;

      double xFRtoHR = xHR - xFR;
      double yFRtoHR = yHR - yFR;

      double xHRtoHL = xHL - xHR;
      double yHRtoHL = yHL - yHR;

      double xHLtoFL = xFL - xHL;
      double yHLtoFL = yFL - yHL;

      if (xFLtoFR * yFRtoHR - yFLtoFR * xFRtoHR > 0.0)
         return true;
      if (xFRtoHR * yHRtoHL - yFRtoHR * xHRtoHL > 0.0)
         return true;
      if (xHRtoHL * yHLtoFL - yHRtoHL * xHLtoFL > 0.0)
         return true;
      if (xHLtoFL * yFLtoFR - yHLtoFL * xFLtoFR > 0.0)
         return true;

      return false;

   }
   
   /**
    *  getDiagonalIntersection returns a point that has a value of the intersection
    *  of the diagonals in a quad stance.
    * 
    *  @return Point3d  A point that is the diagonal intersection of the quad stance
    *                   null if stance is not a quad
    */
   public void getDiagonalIntersection(FramePoint framePointToPack)
   {
      // Copied from http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
      // Uses the P = P1 + alpha (P2-P1) method rather than y = mx+b and thus doesn't have problems with vertical lines.

      FramePoint frontLeftFootstep = getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint frontRightFootstep = getFootstep(RobotQuadrant.FRONT_RIGHT);
      FramePoint hindRightFootstep = getFootstep(RobotQuadrant.HIND_RIGHT);
      FramePoint hindLeftFootstep = getFootstep(RobotQuadrant.HIND_LEFT);

      if (size() < 4)
      {
         throw new UndefinedOperationException("Must have 4 support legs");
      }

      double x1 = frontLeftFootstep.getX();
      double y1 = frontLeftFootstep.getY();

      double x2 = hindRightFootstep.getX();
      double y2 = hindRightFootstep.getY();

      double x3 = frontRightFootstep.getX();
      double y3 = frontRightFootstep.getY();

      double x4 = hindLeftFootstep.getX();
      double y4 = hindLeftFootstep.getY();

      double ua_numerator = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
      double denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

      // If denominator is zero then the lines are parallel. If numerator and denominator are zero then the lines are coincident.

      double ua = ua_numerator / denominator;

      double xp = x1 + ua * (x2 - x1);
      double yp = y1 + ua * (y2 - y1);

      framePointToPack.setIncludingFrame(getReferenceFrame(), xp, yp, 0.0);
   }

   /**
    * isValidTrotPolygon:
    * TRUE if there are exactly two legs in the polygon and they are diagonally opposite
    * FALSE if not true.
    *
    * @return boolean
    */
   public boolean isValidTrotPolygon()
   {
      // check that there are two legs in the polygon
      if (size() != 2)
         return false;

      RobotQuadrant firstLeg = getFirstSupportingQuadrant();
      RobotQuadrant diagonalLeg = firstLeg.getDiagonalOppositeQuadrant();
      
      if (getFootstep(diagonalLeg) == null)
         return false;

      return true;
   }

   /**
    * getLeftTrotLeg: returns the left leg of the polyogn if the polygon is a ValidTrotPolygon.
    * If the polygon is not a ValidTrotPolygon, then it throws an exception
    *
    * @return LegName
    */
   public RobotQuadrant getLeftTrotLeg()
   {
      if (!isValidTrotPolygon())
         throw new RuntimeException("SupportPolygon is not a ValidTrotPolygon");

      RobotQuadrant firstLeg = getFirstSupportingQuadrant();
      if(firstLeg.isQuadrantOnLeftSide())
      {
         return firstLeg;
      }
      
      return firstLeg.getDiagonalOppositeQuadrant();
   }

   /**
    * getRightTrotLeg: returns the left leg of the polyogn if the polygon is a ValidTrotPolygon.
    * If the polygon is not a ValidTrotPolygon, then it throws an exception
    *
    * @return LegName
    */
   public RobotQuadrant getRightTrotLeg()
   {
      if (!isValidTrotPolygon())
         throw new RuntimeException("SupportPolygon is not a ValidTrotPolygon");

      RobotQuadrant firstLeg = getFirstSupportingQuadrant();
      if(firstLeg.isQuadrantOnRightSide())
      {
         return firstLeg;
      }
      
      return firstLeg.getDiagonalOppositeQuadrant();
   }

   public boolean getTangentTangentRadiusCircleCenter(RobotQuadrant robotQuadrantToAnchorTo, double radius, FramePoint2d centerToPack)
   {
      if(containsFootstep(robotQuadrantToAnchorTo) && size() == 3)
      {
         double maximumInCircleRadius = getInCircleRadius2d();
         if(maximumInCircleRadius < radius)
         {
            return false;
         }
         
         FramePoint vertex = getFootstep(robotQuadrantToAnchorTo);
         FramePoint pointA = null;
         FramePoint pointB = null;
         
         for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            FramePoint footstep = getFootstep(robotQuadrant);
            if(footstep == null || robotQuadrant == robotQuadrantToAnchorTo)
            {
               continue;
            }
            if(pointA == null)
            {
               pointA = footstep;
            }
            else if(pointB == null)
            {
               pointB = footstep;
            }
         }
         
         Point2d vertex2d = new Point2d(vertex.getX(), vertex.getY());
         Point2d A2d = new Point2d(pointA.getX(), pointA.getY());
         Point2d B2d = new Point2d(pointB.getX(), pointB.getY());
         
         double VA = vertex.distance(pointA);
         double VB = vertex.distance(pointB);
         double AB = pointA.distance(pointB);
         
         double theta = GeometryTools.getUnknownTriangleAngleByLawOfCosine(VA, VB, AB);
         
         double bisectTheta = 0.5 * theta;
         
         double radiusOffsetAlongBisector = radius * (Math.sin(Math.PI / 2.0) / Math.sin(bisectTheta));
         Point2d adjacentBisector = GeometryTools.getTriangleBisector(A2d, vertex2d, B2d);
         
         Vector2d bisectorVector = new Vector2d(adjacentBisector.getX() - vertex.getX(), adjacentBisector.getY() - vertex.getY());
         double scalar = radiusOffsetAlongBisector / bisectorVector.length();
         
         bisectorVector.scale(scalar);
         
         vertex2d.add(bisectorVector);
         centerToPack.set(vertex2d);
         
         return true;
      }
      
      return false;
   }

   public void getIntersectionFromPointAcrossTrotLinePerpindicularToOutsideFeet(RobotQuadrant swingLeg, FramePoint pointAcrossTrotLine, FrameVector planeNormal, FramePoint pointToPack)
   {
      tempPlaneNormalInWorld.setIncludingFrame(planeNormal);
      tempPlaneNormalInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      tempPlaneNormalInWorld.normalize();
      RobotQuadrant acrossBodyQuadrant = swingLeg.getAcrossBodyQuadrant();
      RobotQuadrant sameSideQuadrant = swingLeg.getSameSideQuadrant();
      FramePoint lineStart = getFootstep(acrossBodyQuadrant);
      FramePoint lineEnd = getFootstep(sameSideQuadrant);
      pointToPack.set(GeometryTools.getIntersectionBetweenLineAndPlane(pointAcrossTrotLine, planeNormal, lineStart, lineEnd));
   }

   public boolean epsilonEquals(QuadrupedSupportPolygon polyTwo)
   {
      if (polyTwo == null)
         return false;
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint thisFootstep = getFootstep(quadrant);
         FramePoint otherFootstep = polyTwo.getFootstep(quadrant);
         
         if(!thisFootstep.epsilonEquals(otherFootstep, 0.005))
         {
            return false;
         }
      }
   
      return true;
   }
}
