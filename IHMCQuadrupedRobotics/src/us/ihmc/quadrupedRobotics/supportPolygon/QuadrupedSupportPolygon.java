package us.ihmc.quadrupedRobotics.supportPolygon;

import java.io.Serializable;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
@SuppressWarnings("all")
/**
 *
 * <p>Title: Polygon </p>
 *
 * <p>Description:
 *
 * This class holds anywhere between 0 and 4
 * Footsteps that form a polygon.
 * Methods cycle over all feet.
 *
 * </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class QuadrupedSupportPolygon implements Serializable
{
   protected final QuadrantDependentList<FramePoint> footsteps = new QuadrantDependentList<FramePoint>();

   public QuadrupedSupportPolygon()
   {
      
   }
   
   public QuadrupedSupportPolygon(QuadrantDependentList<FramePoint> footsteps)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footStep = footsteps.get(robotQuadrant);
         if(footStep != null)
         {
            this.footsteps.put(robotQuadrant, new FramePoint(footStep));
         }
      }
   }

   public QuadrupedSupportPolygon(QuadrupedSupportPolygon polygon)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footStep = polygon.getFootstep(robotQuadrant);
         if(footStep != null)
         {
            this.footsteps.put(robotQuadrant, new FramePoint(footStep));
         }
      }
   }
   
   public void printOutPolygon(String string)
   {
      System.out.print(string + " ");

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         System.out.print(robotQuadrant + ": " + footsteps.get(robotQuadrant) + "  , ");
      }

      System.out.println("\n");
   }
   

   /**
    * This will return the vector normal for each edge. The normal of an
    * edge points outward of the support polygon. The ordering is of the edges
    * starts with the FL and goes clockwise. E.g., if the swing leg is the FR,
    * the edges will be: FL-HR, HR-HL, HL-FL
    *
    * @return Vector2d[]
    */
   public Vector2d[] getEdgeNormals()
   {
      // make sure in world frame
      if (!this.getReferenceFrame().isWorldFrame())
         throw new RuntimeException("polygon must be in world frame");

      RobotQuadrant[][] legPairs = getLegPairs();

      Vector2d[] edgeNormals = new Vector2d[legPairs.length];

      for (int i = 0; i < legPairs.length; i++)
      {
         FramePoint point1 = this.getFootstep(legPairs[i][0]);
         FramePoint point2 = this.getFootstep(legPairs[i][1]);

         Vector2d edgeVector = new Vector2d(point2.getX(), point2.getY());
         edgeVector.sub(new Point2d(point1.getX(), point1.getY()));

         edgeNormals[i] = GeometryTools.getPerpendicularVector(edgeVector);
      }

      return edgeNormals;
   }

   public ReferenceFrame getReferenceFrame()
   {
      // Return the reference frame of the first non-null footstep.
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(robotQuadrant);
         if (footstep != null)
         {
            return footstep.getReferenceFrame();
         }
      }

      throw new RuntimeException("SupportPolygon.getReferenceFrame(): should not get here. There must be at least one non null footstep");
   }

   /**
    * Creates and returns a new copy of a Polygon whose footsteps have been redefined
    * according to the referenceFrame parameter.
    *
    * @param referenceFrame Frame
    *
    * @return SupportPolygon
    */
   public QuadrupedSupportPolygon changeFrameCopy(ReferenceFrame referenceFrame)
   {
      QuadrupedSupportPolygon newPolygon = new QuadrupedSupportPolygon(this);
      newPolygon.changeFrame(referenceFrame);
      return newPolygon;
   }

   private void changeFrame(ReferenceFrame referenceFrame)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footsteps.get(robotQuadrant).changeFrame(referenceFrame);
      }
   }

   /**
    * This method cycles over the cases 0 - 4 for the number
    * of Footsteps in the polygon.
    *
    * @return LegName[][]
    */
   public RobotQuadrant[][] getLegPairs()
   {
      RobotQuadrant[][] legPairs;

      int numberOfLegs = this.getNumberOfLegs();

      switch (numberOfLegs)
      {
         case 0 :
         case 1 :
            legPairs = null;    // BVB : Or should this read:  LegPairs = new LegName[][] { {} };

            break;

         case 2 :
            RobotQuadrant firstLeg = getFirstNonNullQuadrant();
            RobotQuadrant lastLeg = getLastNonNullQuadrant();
            legPairs = new RobotQuadrant[][]
            {
               {firstLeg, lastLeg}
            };

            break;

         case 3 :

            RobotQuadrant swingLeg = null;
            for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            {
               if (footsteps.get(robotQuadrant) == null)
               {
                  swingLeg = robotQuadrant;
               }
            }

            switch (swingLeg)
            {
               case FRONT_LEFT :
               {
                  legPairs = new RobotQuadrant[][]
                  {
                     {RobotQuadrant.HIND_LEFT, RobotQuadrant.HIND_RIGHT}, {RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_RIGHT}, {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_LEFT}
                  };

                  break;
               }

               case FRONT_RIGHT :
               {
                  legPairs = new RobotQuadrant[][]
                  {
                     {RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_LEFT}, {RobotQuadrant.HIND_LEFT, RobotQuadrant.HIND_RIGHT}, {RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_LEFT}
                  };

                  break;
               }

               case HIND_RIGHT :
               {
                  legPairs = new RobotQuadrant[][]
                  {
                     {RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_LEFT}, {RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT}, {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.FRONT_LEFT},
                  };

                  break;
               }

               case HIND_LEFT :
               {
                  legPairs = new RobotQuadrant[][]
                  {
                     {RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_RIGHT}, {RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_RIGHT}, {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.FRONT_LEFT},
                  };

                  break;
               }

               default :
               {
                  throw new RuntimeException("unknown leg name");
               }
            }

         case 4 :
            legPairs = new RobotQuadrant[][]
            {
               {RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_LEFT}, {RobotQuadrant.HIND_LEFT, RobotQuadrant.HIND_RIGHT}, {RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_RIGHT},
               {RobotQuadrant.FRONT_RIGHT, RobotQuadrant.FRONT_LEFT},
            };

            break;

         default :
         {
            throw new RuntimeException("unknown number of legs " + numberOfLegs);
         }

      }

      return legPairs;
   }

   private RobotQuadrant getLastNonNullQuadrant()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.reversedValues)
      {
         if(useThisLeg(robotQuadrant))
         {
            return robotQuadrant;
         }
      }
      return null;
   }

   private RobotQuadrant getFirstNonNullQuadrant()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if(useThisLeg(robotQuadrant))
         {
            return robotQuadrant;
         }
      }
      return null;
   }
   
   private RobotQuadrant getFirstNullQuadrant()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if(!useThisLeg(robotQuadrant))
         {
            return robotQuadrant;
         }
      }
      return null;
   }
   
   private RobotQuadrant getLastNullQuadrant()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.reversedValues)
      {
         if(!useThisLeg(robotQuadrant))
         {
            return robotQuadrant;
         }
      }
      return null;
   }
   
   public void set(QuadrupedSupportPolygon polygon)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footStep = polygon.getFootstep(robotQuadrant);
         if(footStep != null)
         {
            this.footsteps.put(robotQuadrant, new FramePoint(footStep));
         }
         else
         {
            this.footsteps.remove(robotQuadrant);
         }
      }
   }

   public FramePoint getFootstep(RobotQuadrant robotQuadrant)
   {
      return footsteps.get(robotQuadrant);
   }
   
   public void setFootstep(RobotQuadrant quadrant, FramePoint newFootstep)
   {
      FramePoint polygonFootstep = footsteps.get(quadrant);
      if(polygonFootstep != null)
      {
         polygonFootstep.set(newFootstep);
      }
      else
      {
         footsteps.put(quadrant, new FramePoint(newFootstep));
      }
   }

   /**
    * Returns the number of feet (vertices) in the polygon.
    *
    * @return int
    */
   public int getNumberOfLegs()
   {
      // check the number of null legs
      int numberOfLegs = 0;
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footStep = footsteps.get(robotQuadrant);
         if (useThisLeg(footStep))
         {
            numberOfLegs++;
         }
      }

      return numberOfLegs;
   }

   /**
    *  This method compares this to another support polygon and returns the number of matching footsetps
    *
    * @return int the number of footsteps that epsilon match
    */
   public int getNumberOfMatchinfFootsteps(QuadrupedSupportPolygon polygon)
   {
      // check the number of null legs
      int numberOfMatching = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint thisFootStep = this.getFootstep(robotQuadrant);
         FramePoint otherFootStep = polygon.getFootstep(robotQuadrant);
         
         if (thisFootStep != null && thisFootStep.epsilonEquals(otherFootStep, 0.0005))
         {
            numberOfMatching++;
         }
      }

      return numberOfMatching;
   }


   public ArrayList<RobotQuadrant> getLegs()
   {
      ArrayList<RobotQuadrant> ret = new ArrayList<RobotQuadrant>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (useThisLeg(robotQuadrant))
         {
            ret.add(robotQuadrant);
         }
      }

      return ret;
   }


   /**
    * getCentroid
    *
    * @param centroid FramePoint
    */
   public void getCentroid(FramePoint centroid)
   {
      centroid.set(0.0, 0.0, 0.0);

      int numberOfFootsteps = 0;
      for (RobotQuadrant quadrant: RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(quadrant);
         if (useThisLeg(footstep))
         {
            centroid.add(footstep);
            numberOfFootsteps++;
         }
      }

      centroid.scale(1.0 / ((double) numberOfFootsteps));
   }

   /**
    * Moves each footstep a given distance toward the opposite edge.
    * To guarantee shrinkage, we cap the shrink amount by half the polygon's
    * incircle radius.
    *
    * This method works only for triangle polygons where the footsteps
    * are Cartesian.
    *
    * NOTE: The Z values for the feet will remain unchanged;
    * we are shrinking the polygon in terms of (X,Y) positions only.
    *
    * @param shrinkDistance double
    * @return Polygon
    */
   public QuadrupedSupportPolygon getShrunkenPolygon(double shrinkDistance)
   {
      if (getNumberOfLegs() != 3)
      {
         throw new RuntimeException("This method works only for triangle polygons.");
      }

      // Cap the shrink amount by half the radius of the largest inscribed circle.
      shrinkDistance = Math.min(shrinkDistance, 0.5 * this.getInCircleRadius3Legs());

      QuadrantDependentList<FramePoint> newFeet = new QuadrantDependentList<FramePoint>();

      // Foot 0
      FramePoint footPoint = footsteps.get(RobotQuadrant.FRONT_LEFT);
      FramePoint point1 = footsteps.get(RobotQuadrant.FRONT_RIGHT);
      FramePoint point2 = footsteps.get(RobotQuadrant.HIND_RIGHT);
      Vector2d edgeVector = new Vector2d(point2.getX(), point2.getY());
      edgeVector.sub(new Point2d(point1.getX(), point1.getY()));
      Vector3d edgeNormal = new Vector3d(-edgeVector.y, edgeVector.x, 0.0);
      edgeNormal.normalize();
      edgeNormal.scale(shrinkDistance);
      footPoint.add(new FrameVector(footPoint.getReferenceFrame(), edgeNormal));
      newFeet.put(RobotQuadrant.FRONT_LEFT, footPoint);

      // Foot 1
      footPoint = footsteps.get(RobotQuadrant.FRONT_RIGHT);
      point1 = footsteps.get(RobotQuadrant.HIND_RIGHT);
      point2 = footsteps.get(RobotQuadrant.FRONT_LEFT);
      edgeVector = new Vector2d(point2.getX(), point2.getY());
      edgeVector.sub(new Point2d(point1.getX(), point1.getY()));
      edgeNormal = new Vector3d(-edgeVector.y, edgeVector.x, 0.0);
      edgeNormal.normalize();
      edgeNormal.scale(shrinkDistance);
      footPoint.add(new FrameVector(footPoint.getReferenceFrame(), edgeNormal));
      newFeet.put(RobotQuadrant.FRONT_RIGHT, footPoint);

      // Foot 2
      footPoint = footsteps.get(RobotQuadrant.HIND_RIGHT);
      point1 = footsteps.get(RobotQuadrant.FRONT_LEFT);
      point2 = point1 = footsteps.get(RobotQuadrant.FRONT_RIGHT);
      edgeVector = new Vector2d(point2.getX(), point2.getY());
      edgeVector.sub(new Point2d(point1.getX(), point1.getY()));
      edgeNormal = new Vector3d(-edgeVector.y, edgeVector.x, 0.0);
      edgeNormal.normalize();
      edgeNormal.scale(shrinkDistance);
      footPoint.add(new FrameVector(footPoint.getReferenceFrame(), edgeNormal));
      newFeet.put(RobotQuadrant.HIND_RIGHT, footPoint);

      return new QuadrupedSupportPolygon(newFeet);
   }

   /**
    * getLowestFootstep
    *
    * @param removeSwingLeg boolean
    * @return Footstep
    */
   public RobotQuadrant getLowestFootstep()
   {
      double minZ = Double.POSITIVE_INFINITY;
      RobotQuadrant lowest = null;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(robotQuadrant);
         if (useThisLeg(footstep))
         {
            double footZ = footstep.getZ();
            if (footZ < minZ)
            {
               minZ = footZ;
               lowest = robotQuadrant;
            }
         }
      }

      return lowest;
   }

   /**
    * getHighestFootstep
    *
    * @param removeSwingLeg boolean
    * @return Footstep
    */
   public RobotQuadrant getHighestFootstep()
   {
      double maxZ = Double.NEGATIVE_INFINITY;
      RobotQuadrant highest = null;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(robotQuadrant);
         if (useThisLeg(footstep))
         {
            double footZ = footstep.getZ();
            if (footZ > maxZ)
            {
               maxZ = footZ;
               highest = robotQuadrant;
            }
         }
      }

      return highest;
   }

   /**
    * getBounds modifies the min and max points passed in to the min and max
    * xy values contained in the set of Footsteps that make up the polygon
    *
    * @param min Point2d  Minimum x and y value contained in footsteps list
    * @param max Point2d  Maximum x and y value contained in footsteps list
    */
   public void getBounds(Point2d min, Point2d max)
   {
      min.x = min.y = Double.POSITIVE_INFINITY;
      max.x = max.y = Double.NEGATIVE_INFINITY;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(robotQuadrant);
         if (useThisLeg(footstep))
         {
            if (footstep.getX() < min.x)
            {
               min.x = footstep.getX();
            }

            if (footstep.getY() < min.y)
            {
               min.y = footstep.getY();
            }

            if (footstep.getX() > max.x)
            {
               max.x = footstep.getX();
            }

            if (footstep.getY() > max.y)
            {
               max.y = footstep.getY();
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
   public boolean isInside(Point2d point)
   {
      if (distanceInside(point) > 0.0)
      {
         return true;
      }

      return false;
   }

   /**
    * Projects a 3D point onto the plane defined by z=0.  Returns the 2D
    * projected point.
    *
    * @param point Point3d
    * @return Point2d
    */
   public static Point2d projectXY(FramePoint point)
   {
      Point2d point2d = new Point2d(point.getX(), point.getY());

      return point2d;
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
   public double distanceInside(Point2d point)
   {
      boolean inside = true;

      RobotQuadrant indexStart = getLastNonNullQuadrant();
      RobotQuadrant indexEnd = getFirstNonNullQuadrant();

      // indexStart must be greater than indexEnd
      if (indexStart.ordinal() < indexEnd.ordinal())
      {
         throw new RuntimeException("SupportPolygon.distanceInside(): indexStart must be > indexEnd");
      }
      FramePoint pointStart = this.footsteps.get(indexStart);
      FramePoint pointEnd = this.footsteps.get(indexEnd);

//    FramePoint pointStart = footsteps.get(footsteps.size() - 1).getPositionFramePoint();
//    FramePoint pointEnd = footsteps.get(0).getPositionFramePoint();

      if ((!pointStart.getReferenceFrame().isWorldFrame()) || (!pointEnd.getReferenceFrame().isWorldFrame()))
      {
         throw new RuntimeException("SupportPolygon.distanceInside() should only be called from a Polygon in a World Frame!!");
      }

      Point2d lineStart = projectXY(pointStart);
      Point2d lineEnd = projectXY(pointEnd);
      boolean onLeftSide = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);
      double distance = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);
      
      
      for (int i = indexEnd.ordinal() + 1; i <= indexStart.ordinal(); i++)
      {
         lineStart = lineEnd;
         FramePoint footstep = this.footsteps.get(RobotQuadrant.getQuadrantNameFromOrdinal(i));
         if (useThisLeg(footstep))
         {
            lineEnd = projectXY(footstep);

            if (onLeftSide != GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd))
            {
               inside = false;
            }

            distance = Math.min(distance, GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd));
         }
      }

      if (inside)
      {
         return distance;
      }
      else
      {
         return -distance;
      }
   }

   /**
    * getInCircleAllLegs
    *
    * This method packs the best (largest radius) incircle point and radius of the polygon excluding circles that are not inscribed
    * 2 or more feet cannot be in the same place, and legs cannot be crossed. All four legs have to be defined.
    *
    * @deprecated
    * @param bestInCirclePoint Point2d  to be overwritten by best incircle point of polygon
    * @param bestInCircleRadius double to be overwritten by best incircle radius of polygon
    */
   public double getInCircle(Point2d bestInCirclePoint)
   {
      double bestInCircleRadius = -1.0;
      Point2d intersection = null;

      int numberOfLegs = getNumberOfLegs();
      int overlappingFootSteps = getNumberOfFootstepsOnTopOfEachOther();

      if (numberOfLegs == 3)
      {
         if(overlappingFootSteps > 0)
         {
            return Double.NaN;
         }
         bestInCircleRadius = this.getInCircleRadius3Legs();
         intersection = this.getInCirclePoint3Legs();
         bestInCirclePoint.set(intersection);
      }

      else if (numberOfLegs == 4)
      {
         if(overlappingFootSteps > 1)
         {
            return Double.NaN;
         }
         
         FramePoint a = getFootstep(RobotQuadrant.FRONT_LEFT);
         FramePoint b = getFootstep(RobotQuadrant.FRONT_RIGHT);
         FramePoint c = getFootstep(RobotQuadrant.HIND_RIGHT);
         FramePoint d = getFootstep(RobotQuadrant.HIND_LEFT);

         Point2d p1 = new Point2d(a.getX(), a.getY());
         Point2d p2 = new Point2d(b.getX(), b.getY());
         Point2d p3 = new Point2d(c.getX(), c.getY());
         Point2d p4 = new Point2d(d.getX(), d.getY());

         double radius, farSideDistance;

         intersection = getInCirclePoint(p1, p2, p3, p4);

         if (intersection != null)
         {
            radius = GeometryTools.distanceFromPointToLine(intersection, p2, p3);
            farSideDistance = GeometryTools.distanceFromPointToLine(intersection, p1, p4);

            if (farSideDistance >= radius)
            {
               if (radius > bestInCircleRadius)
               {
                  bestInCircleRadius = radius;
                  bestInCirclePoint.set(intersection);
               }
            }
         }

         intersection = getInCirclePoint(p2, p3, p4, p1);

         if (intersection != null)
         {
            radius = GeometryTools.distanceFromPointToLine(intersection, p3, p4);
            farSideDistance = GeometryTools.distanceFromPointToLine(intersection, p2, p1);

            if (farSideDistance >= radius)
            {
               if (radius > bestInCircleRadius)
               {
                  bestInCircleRadius = radius;
                  bestInCirclePoint.set(intersection);
               }
            }
         }

         intersection = getInCirclePoint(p3, p4, p1, p2);

         if (intersection != null)
         {
            radius = GeometryTools.distanceFromPointToLine(intersection, p4, p1);
            farSideDistance = GeometryTools.distanceFromPointToLine(intersection, p3, p2);

            if (farSideDistance >= radius)
            {
               if (radius > bestInCircleRadius)
               {
                  bestInCircleRadius = radius;
                  bestInCirclePoint.set(intersection);
               }
            }
         }

         intersection = getInCirclePoint(p4, p1, p2, p3);

         if (intersection != null)
         {
            radius = GeometryTools.distanceFromPointToLine(intersection, p1, p2);
            farSideDistance = GeometryTools.distanceFromPointToLine(intersection, p4, p3);

            if (farSideDistance >= radius)
            {
               if (radius > bestInCircleRadius)
               {
                  bestInCircleRadius = radius;
                  bestInCirclePoint.set(intersection);
               }
            }
         }

         if ((Double.isNaN(bestInCirclePoint.x)) || (Double.isNaN(bestInCirclePoint.y)))
         {
            throw new RuntimeException("NaN in InCircle Point for SupportPolygon: " + this);
         }
      }

      return bestInCircleRadius;
   }

   private int getNumberOfFootstepsOnTopOfEachOther()
   {
      int total = 0;
      RobotQuadrant[] quadrants = RobotQuadrant.values;
      for (int i = 0; i < quadrants.length; i++)
      {
         FramePoint footStep = footsteps.get(quadrants[i]);
         if(footStep != null)
         {
            for (int j = i + 1; j < quadrants.length; j++)
            {
               FramePoint otherFootStep = footsteps.get(quadrants[j]);
               if(footStep != null && footStep.epsilonEquals(otherFootStep, 0.001))
               {
                  total++;
               }
            }
         }
      }
      return total;
   }

   /**
    * getInCircleRadiusIgnoreSwingLeg
    *
    * This method returns the best (largest radius) incircle radius of the polygon excluding circles that are not inscribed.
    * If the swing leg is null it includes all legs
    * If the swing leg is not null, there must be three specified non-null legs
    * 2 or more feet cannot be in the same place, and legs cannot be crossed.
    *
    * @deprecated
    * @return double best incircle radius of polygon
    */
   public double getInCircleRadius3Legs()
   {
      if (!this.getReferenceFrame().isWorldFrame())
         throw new RuntimeException("SupportPolygon:getInCircleAllLegs() requires the SupportPolygon to be in a world frame!");

      Point2d intersection = getInCirclePoint3Legs();

      ArrayList<RobotQuadrant> nonSwingLegs = new ArrayList<RobotQuadrant>();
      for (RobotQuadrant robotQuadrant: RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(robotQuadrant);
         if (useThisLeg(footstep))
            nonSwingLegs.add(robotQuadrant);
      }

      if (nonSwingLegs.size() != 3)
         throw new RuntimeException("Stance: getInCircleRadiusIgnoreSwingLeg cannot be applied to stance with < 3 nonSwingLegs\n" + this);

      FramePoint b = getFootstep(nonSwingLegs.get(1));
      FramePoint c = getFootstep(nonSwingLegs.get(2));

      Point2d p2 = new Point2d(b.getX(), b.getY());
      Point2d p3 = new Point2d(c.getX(), c.getY());

      // IF block added by CPerez
      // Jan-02-2007 Checking if the center point is not null
      // if it is null the returned radius is 0.0.
      double radius = 0.0;
      if (intersection != null)
      {
         radius = GeometryTools.distanceFromPointToLine(intersection, p2, p3);
      }

      // End of added IF block

      return radius;
   }

   /**
    * getInCirclePointIgnoreSwingLeg
    *
    * This method returns the best (largest radius) incircle point of the polygon excluding circles that are not inscribed.
    * If the swing leg is null it includes all legs
    * If the swing leg is not null, there must be three specified non-null legs
    * 2 or more feet cannot be in the same place, and legs cannot be crossed.
    *
    * @deprecated
    * @return Point2d  best incircle point for polygon
    */
   public Point2d getInCirclePoint3Legs()
   {
      if (!this.getReferenceFrame().isWorldFrame())
         throw new RuntimeException("SupportPolygon:getInCircleAllLegs() requires the SupportPolygon to be in a world frame!");

      ArrayList<RobotQuadrant> nonSwingLegs = new ArrayList<RobotQuadrant>();
      for (RobotQuadrant robotQuadrant: RobotQuadrant.values)
      {
         if (useThisLeg(robotQuadrant))
            nonSwingLegs.add(robotQuadrant);
      }

      if (nonSwingLegs.size() != 3)
         throw new RuntimeException("Stance: getInCircleRadiusIgnoreSwingLeg cannot be applied to stance with < 3 nonSwingLegs");

      FramePoint a = getFootstep(nonSwingLegs.get(0));
      FramePoint b = getFootstep(nonSwingLegs.get(1));
      FramePoint c = getFootstep(nonSwingLegs.get(2));

      Point2d p1 = new Point2d(a.getX(), a.getY());
      Point2d p2 = new Point2d(b.getX(), b.getY());
      Point2d p3 = new Point2d(c.getX(), c.getY());
      Point2d p4 = new Point2d(p1);

      Point2d intersection = getInCirclePoint(p1, p2, p3, p4);

      return intersection;
   }

   /**
    * getInCirclePoint
    *
    * his method assumes the points are in a specific order (U-shape).
    * It returns the InCricle Point based on the two angles formed by the three line segments
    *
    * @param p1 Point2d point defining the three line segments (must be in order)
    * @param p2 Point2d point defining the three line segments (must be in order)
    * @param p3 Point2d point defining the three line segments (must be in order)
    * @param p4 Point2d point defining the three line segments (must be in order)
    * @return Point2d incirlce point
    */
   public static Point2d getInCirclePoint(Point2d p1, Point2d p2, Point2d p3, Point2d p4)
   {
      // get first two line segments
      Vector2d v21 = new Vector2d();
      v21.sub(p1, p2);
      Vector2d v23 = new Vector2d();
      v23.sub(p3, p2);

      // normalize and subtract to get vector that bisects the first angle
      v21.normalize();
      v23.normalize();
      Vector2d v2p = new Vector2d(v21);
      v2p.add(v23);
      v2p.normalize();

      // get second two line segments
      Vector2d v34 = new Vector2d();
      v34.sub(p4, p3);
      Vector2d v32 = new Vector2d();
      v32.sub(p2, p3);

      // normalize and subtract to get vector that bisects the second angle
      v34.normalize();
      v32.normalize();
      Vector2d v3p = new Vector2d(v34);
      v3p.add(v32);
      v3p.normalize();

      // find intersection point of the two bisecting vectors
      Point2d intersection = GeometryTools.getIntersectionBetweenTwoLines(p2, v2p, p3, v3p);

      if ((Double.isNaN(intersection.x)) || (Double.isNaN(intersection.y)))
         return null;

//    {
//       System.err.println("p1 = " + p1);
//       System.err.println("p2 = " + p2);
//       System.err.println("p3 = " + p3);
//       System.err.println("p4 = " + p4);
//
//       System.err.println("v21 = " + v21);
//       System.err.println("v23 = " + v23);
//       System.err.println("v2p = " + v2p);
//       System.err.println("v34 = " + v34);
//       System.err.println("v32 = " + v32);
//       System.err.println("v3p = " + v3p);
//
//       throw new RuntimeException("NaN in InCircle Point for SupportPolygon: " + this);
//    }

      return intersection;
   }

   /**
    * distanceInsideIncircle
    *
    * @param point Point2d
    * @param removeSwingLeg boolean
    * @return double
    */
   public double distanceInsideIncircle(Point2d point)
   {
      Point2d inCirclePoint = new Point2d();
      double inCircleRadius;
      int numberOfLegs = getNumberOfLegs();
      if (numberOfLegs == 3)
      {
         inCirclePoint = getInCirclePoint3Legs();
         inCircleRadius = getInCircleRadius3Legs();
      }
      else if (numberOfLegs == 4)
      {
         inCircleRadius = getInCircle(inCirclePoint);
      }
      else
      {
         return Double.NaN;

//       throw new RuntimeException("Polygon.distanceInsideIncircle(): Unhandled case: numberOfLegs == " + numberOfLegs);
      }

      double distanceToInCircleCenter = point.distance(inCirclePoint);

      return (inCircleRadius - distanceToInCircleCenter);
   }

   public boolean useThisLeg(FramePoint footstep)
   {
      if (footstep == null)
         return false;
      else
         return true;
   }
   
   public boolean useThisLeg(RobotQuadrant robotQuadrant)
   {
      if (footsteps.get(robotQuadrant) == null)
         return false;
      else
         return true;
   }

   /**
    * getCentroidFramePoint
    *
    * @param removeSwingLeg boolean
    * @return FramePoint
    */
   public FramePoint getCentroidFramePoint()
   {
      FramePoint centroid = new FramePoint(this.getReferenceFrame());
      getCentroid(centroid);

      return centroid;
   }

//   /**
//    * getLastNonNullFootstep
//    *
//    * @param removeSwingLeg boolean
//    * @return int
//    */
//   public int getLastNonNullFootstep()
//   {
//      for (int i = this.footsteps.length - 1; i >= 0; i--)
//      {
//         if (useThisLeg(this.footsteps[i]))
//         {
//            return i;
//         }
//      }
//
//      throw new RuntimeException("Polygon: getLastNonNullFootstep(): no last non null footstep");
//   }
//
//   /**
//    * getFirstNonNullFootstep
//    *
//    * @param removeSwingLeg boolean
//    * @return int
//    */
//   public int getFirstNonNullFootstep()
//   {
//      for (int i = 0; i < this.footsteps.length; i++)
//      {
//         if (useThisLeg(this.footsteps[i]))
//         {
//            return i;
//         }
//      }
//
//      throw new RuntimeException("Polygon: getFirstNonNullFootstep(): no first non null footstep");
//   }
//
//   /**
//    * getFirstNonNullFootstep
//    *
//    * @param removeSwingLeg boolean
//    * @return int
//    */
//   public int getFirstNullFootstep()
//   {
//      for (int i = 0; i < this.footsteps.length; i++)
//      {
//         if (!useThisLeg(this.footsteps[i]))
//         {
//            return i;
//         }
//      }
//
//      throw new RuntimeException("Polygon: getFirstNullFootstep(): no first null footstep");
//   }

   /**
    * Returns the distance from the centroid (balancing point of the polygon)
    * to the closest edge of the polygon.   If this distance is small, then the
    * polygon represents a poor choice for a balanced foot configuration. If it is
    * negative, then the polygon is statically unstable.
    *
    * @return double
    */
   public double centroidDistanceInside()
   {
      FramePoint centroid = getCentroidFramePoint();
      Point2d center = new Point2d(centroid.getX(), centroid.getY());

      return distanceInside(center);
   }

   /**
    * Computes a nominal yaw angle. If triangle support, then the angle from the front to back support foot on the same side.
    * If a quad, then the average of the two front to back angles.
    *
    * @return double
    */
   public double getNominalYaw()
   {
      FramePoint flFootstep = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint frFootstep = this.getFootstep(RobotQuadrant.FRONT_RIGHT);
      FramePoint hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      FramePoint hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

      Vector2d vector1 = null, vector2 = null;

      if ((flFootstep != null) && (hlFootstep != null))
      {
         vector1 = new Vector2d(flFootstep.getX() - hlFootstep.getX(), flFootstep.getY() - hlFootstep.getY());
      }

      if ((frFootstep != null) && (hrFootstep != null))
      {
         vector2 = new Vector2d(frFootstep.getX() - hrFootstep.getX(), frFootstep.getY() - hrFootstep.getY());
      }

      if ((vector1 != null) && (vector2 != null))
      {
         vector1.add(vector2);

         return Math.atan2(vector1.y, vector1.x);
      }

      else if (vector1 != null)
      {
         return Math.atan2(vector1.y, vector1.x);
      }
      else if (vector2 != null)
      {
         return Math.atan2(vector2.y, vector2.x);
      }
      else
      {
         return 0.0;
      }
   }

   public double getNominalYawHindLegs()
   {
      FramePoint hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      FramePoint hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

      double ret = Math.atan2(hrFootstep.getX() - hlFootstep.getX(), -(hrFootstep.getY() - hlFootstep.getY()));

      return ret;
   }

   /**
    * Computes a nominal pitch angle. If triangle support, then the angle from the front to back support foot on the same side.
    * If a quad, then the average of the two front to back angles.
    * @return double
    */
   public double getNominalPitch()
   {
      FramePoint flFootstep = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint frFootstep = this.getFootstep(RobotQuadrant.FRONT_RIGHT);
      FramePoint hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      FramePoint hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

      Vector3d vector1 = null, vector2 = null;

      if ((useThisLeg(flFootstep) && useThisLeg(hlFootstep)))
      {
         vector1 = new Vector3d(flFootstep.getX() - hlFootstep.getX(), flFootstep.getY() - hlFootstep.getY(), flFootstep.getZ() - hlFootstep.getZ());
      }

      if (useThisLeg(frFootstep) && useThisLeg(hrFootstep))
      {
         vector2 = new Vector3d(frFootstep.getX() - hrFootstep.getX(), frFootstep.getY() - hrFootstep.getY(), frFootstep.getZ() - hrFootstep.getZ());
      }

      if ((vector1 != null) && (vector2 != null))
      {
         vector1.add(vector2);
         double length = vector1.length();
         if (length < 1e-3)
            return 0.0;    // Avoid division by zero and NaN...

         return -Math.asin(vector1.z / length);
      }

      else if (vector1 != null)
      {
         double length = vector1.length();
         if (length < 1e-3)
            return 0.0;

         return -Math.asin(vector1.z / length);
      }
      else if (vector2 != null)
      {
         double length = vector2.length();
         if (length < 1e-3)
            return 0.0;

         return -Math.asin(vector2.z / length);
      }
      else
      {
         return 0.0;
      }
   }


   /**
    * Computes a nominal roll angle. If triangle support, then the angle from the front to back support foot on the same side.
    * If a quad, then the average of the two front to back angles.
    * @return double
    */
   public double getNominalRoll()
   {
      FramePoint flFootstep = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint frFootstep = this.getFootstep(RobotQuadrant.FRONT_RIGHT);
      FramePoint hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      FramePoint hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

      Vector3d vector1 = null, vector2 = null;

      if ((useThisLeg(flFootstep) && useThisLeg(frFootstep)))
      {
         vector1 = new Vector3d(flFootstep.getX() - frFootstep.getX(), flFootstep.getY() - frFootstep.getY(), flFootstep.getZ() - frFootstep.getZ());
      }

      if (useThisLeg(hlFootstep) && useThisLeg(hrFootstep))
      {
         vector2 = new Vector3d(hlFootstep.getX() - hrFootstep.getX(), hlFootstep.getY() - hrFootstep.getY(), hlFootstep.getZ() - hrFootstep.getZ());
      }

      if ((vector1 != null) && (vector2 != null))
      {
         vector1.add(vector2);
         double length = vector1.length();
         if (length < 1e-3)
            return 0.0;    // Avoid division by zero and NaN...

         return Math.asin(vector1.z / length);
      }

      else if (vector1 != null)
      {
         double length = vector1.length();
         if (length < 1e-3)
            return 0.0;

         return Math.asin(vector1.z / length);
      }
      else if (vector2 != null)
      {
         double length = vector2.length();
         if (length < 1e-3)
            return 0.0;

         return Math.asin(vector2.z / length);
      }
      else
      {
         return 0.0;
      }
   }


   /**
    * Returns pitch of the polygon in radians.  Negative values indicate the
    * polygon's "front" is higher than the "back": average front foot positions
    * are higher than then average back foot positions
    *
    * @return double
    */
   public double getPitchInRadians()
   {
      FramePoint averageFrontPosition = averageFrontFootPosition();
      FramePoint averageHindPosition = averageHindFootPosition();

      return -Math.atan2(averageFrontPosition.getZ() - averageHindPosition.getZ(), averageFrontPosition.getX() - averageHindPosition.getX());
   }

   /**
    * Returns the average position of the vertices in the polygon
    * corresponding to front foot locations.
    *
    * @return Point3d
    */
   public FramePoint averageFrontFootPosition()
   {
      int footDownCounter = 0;
      FramePoint footPosition = new FramePoint(getReferenceFrame());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(robotQuadrant);
         if (useThisLeg(footstep))
         {
            if (robotQuadrant.isQuadrantInFront())
            {
               footPosition.add(footstep);
               footDownCounter++;
            }
         }
      }

      footPosition.scale(1.0 / ((double) footDownCounter));

      return footPosition;
   }

   /**
    * Returns the average position of the vertices in the polygon
    * corresponding to hind foot locations.
    *
    * @return Point3d
    */
   public FramePoint averageHindFootPosition()
   {
      int footDownCounter = 0;
      FramePoint footPosition = new FramePoint(getReferenceFrame());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(robotQuadrant);
         if (useThisLeg(footstep))
         {
            if (robotQuadrant.isQuadrantInHind())
            {
               footPosition.add(footstep);
               footDownCounter++;
            }
         }
      }

      footPosition.scale(1.0 / ((double) footDownCounter));

      return footPosition;
   }

   /**
    * Rotates the feet about the Centroid, keeping the z heights.
    *
    * @return SupportPolygon
    */
   public void yawAboutCentroid(double yaw)
   {
      FramePoint centroid = getCentroidFramePoint();

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(quadrant);
         if (useThisLeg(footstep))
         {
            FramePoint rotatedPoint = footstep.yawAboutPoint(centroid, yaw);
            footstep.set(rotatedPoint);
         }
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
      int numberOfLegs = getNumberOfLegs();
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

      RobotQuadrant indexStart = getLastNonNullQuadrant();
      RobotQuadrant indexEnd = getFirstNonNullQuadrant();

      // indexStart must be greater than indexEnd
      if (indexStart.ordinal() < indexEnd.ordinal())
      {
         throw new RuntimeException("SupportPolygon.distanceInside(): indexStart must be > indexEnd");
      }

      FramePoint pointStart = this.footsteps.get(indexStart);
      FramePoint pointEnd = this.footsteps.get(indexEnd);

      // FramePoint pointStart = footsteps.get(footsteps.size() - 1).getPositionFramePoint();
      // FramePoint pointEnd = footsteps.get(0).getPositionFramePoint();

      if ((!pointStart.getReferenceFrame().isWorldFrame()) || (!pointEnd.getReferenceFrame().isWorldFrame()))
      {
         throw new RuntimeException("SupportPolygon.distanceInside() should only be called from a Polygon in a World Frame!!");
      }

      Point2d lineStart = projectXY(pointStart);
      Point2d lineEnd = projectXY(pointEnd);
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
         FramePoint footstep = this.footsteps.get(RobotQuadrant.getQuadrantNameFromOrdinal(i));
         if (useThisLeg(footstep))
         {
            counter++;
            lineEnd = projectXY(footstep);

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

   public boolean hasSameFootsteps(QuadrupedSupportPolygon polyTwo)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint thisFootstep = footsteps.get(robotQuadrant);
         FramePoint otherFootstep = polyTwo.getFootstep(robotQuadrant);
         if(!thisFootstep.epsilonEquals(otherFootstep, 0.005))
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
      if (this.getNumberOfLegs() != 3)
         throw new IllegalArgumentException("This supportPolygon must contain exactly three legs not " + this.getNumberOfLegs());

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(!useThisLeg(quadrant))
         {
            Point2d A = projectXY(this.getFootstep(quadrant.getAcrossBodyQuadrant()));
            Point2d B = projectXY(this.getFootstep(quadrant.getSameSideQuadrant()));
            Point2d C = projectXY(this.getFootstep(quadrant.getDiagonalOppositeQuadrant()));
            Point2d bisectingPoint = GeometryTools.getTriangleBisector(A, B, C);

            return new FramePoint(this.getReferenceFrame(), bisectingPoint.x, bisectingPoint.y, 0.0);
         }
      }
      return null;
   }
   
   
   
   /**
    *  getStanceWidthFrontLegs
    * 
    *  @return double
    */
   public double getStanceLength(RobotSide robotSide)
   {
      if (this.getNumberOfLegs() != 4)
      {
         throw new RuntimeException("Need 4 legs for SupportPolygon.getStanceWidthFrontLegs()");
      }

      FramePoint frontFootstep = footsteps.get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide));
      FramePoint endFootstep = footsteps.get(RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide));

      return frontFootstep.distance(endFootstep);
   }
   
   /**
    *  getStanceWidthFrontLegs
    * 
    *  @return double
    */
   public double getStanceWidthFrontLegs()
   {
      if (this.getNumberOfLegs() != 4)
      {
         throw new RuntimeException("Need 4 legs for SupportPolygon.getStanceWidthFrontLegs()");
      }

      double heading = this.getNominalYaw();
      Vector2d perpendicularHeadingVecetor = new Vector2d(-Math.sin(heading), Math.cos(heading));

      FramePoint frontLeft = footsteps.get(RobotQuadrant.FRONT_LEFT);
      FramePoint frontRight = footsteps.get(RobotQuadrant.FRONT_RIGHT);

      frontLeft.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      frontRight.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double vectorX = frontLeft.getX() - frontRight.getX();
      double vectorY = frontLeft.getY() - frontRight.getY();

      Vector2d rightToLeft = new Vector2d(vectorX, vectorY);

      return Math.abs(rightToLeft.dot(perpendicularHeadingVecetor));
   }

   /**
    * getStanceWidthHindLegs
    *
    * @return double
    */
   public double getStanceWidthHindLegs()
   {
      if (this.getNumberOfLegs() != 4)
      {
         throw new RuntimeException("Need 4 legs for SupportPolygon.getStanceWidthHindLegs()");
      }

      double heading = this.getNominalYaw();
      Vector2d perpendicularHeadingVecetor = new Vector2d(-Math.sin(heading), Math.cos(heading));

      FramePoint hindLeft = footsteps.get(RobotQuadrant.HIND_LEFT);
      FramePoint hindRight = footsteps.get(RobotQuadrant.HIND_RIGHT);

      hindLeft.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      hindRight.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double vectorX = hindLeft.getX() - hindRight.getX();
      double vectorY = hindLeft.getY() - hindRight.getY();

      Vector2d rightToLeft = new Vector2d(vectorX, vectorY);

      return Math.abs(rightToLeft.dot(perpendicularHeadingVecetor));
   }


   public boolean epsilonEquals(QuadrupedSupportPolygon polyTwo)
   {
      if (polyTwo == null)
         return false;
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint thisFootstep = this.getFootstep(quadrant);
         FramePoint otherFootstep = polyTwo.getFootstep(quadrant);
         if(!thisFootstep.epsilonEquals(otherFootstep, 0.005))
         {
            return false;
         }
         
      }

      return true;
   }

   public String toString()
   {
      String string = "";

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint footstep = footsteps.get(quadrant);
         if (footstep != null)
            string += " " + quadrant.getCamelCaseNameForStartOfExpression() +  " " + footstep.toString() + " ";
      }

      return string;
   }


   /**
    * Returns the intersection points of the polygon and the given line segment.
    * The first returned FramePoint will be the point nearest the segmentStartFramePoint and the second returned FramePoint
    * will be the point nearest the segmentEndFramePoint. If both segment FramePoints are inside the polygon, both returned
    * FramePoints will be non-null. If both segment FramePoints are outside the Polygon, the one corresponding to the nearest will be non-null.
    * If one is inside and one is outside, such that the segment fully intersect the polygon, then
    * the one corresponding to the one inside will be non-null.
    * The z value of the returned points will be set so that the points are along the line between the feet points. That will make these points be
    * the tipping points on the polygon.
    *
    * @param segmentStartFramePoint FramePoint
    * @param segmentEndFramePoint FramePoint
    *
    * @return FramePoint[]
    */
   public FramePoint[] getIntersectingPointsOfSegment(FramePoint segmentStartFramePoint, FramePoint segmentEndFramePoint)
   {
      double EPSILON = 1e-6;

      this.getReferenceFrame().checkReferenceFrameMatch(segmentStartFramePoint.getReferenceFrame());
      segmentStartFramePoint.getReferenceFrame().checkReferenceFrameMatch(segmentEndFramePoint.getReferenceFrame());

      if (!segmentStartFramePoint.getReferenceFrame().isWorldFrame())
      {
         throw new RuntimeException("!segmentStart.getReferenceFrame().isWorldFrame()");
      }

      Point2d segmentStart = projectXY(segmentStartFramePoint);
      Point2d segmentEnd = projectXY(segmentEndFramePoint);

      RobotQuadrant indexStart = getLastNonNullQuadrant();
      RobotQuadrant indexEnd = getFirstNonNullQuadrant();

      // indexStart must be greater than indexEnd
      if (indexStart.ordinal() < indexEnd.ordinal())
      {
         throw new RuntimeException("SupportPolygon.getIntersectingPointsOfSegment(): indexStart must be > indexEnd");
      }

      FramePoint feetPointStart = this.footsteps.get(indexStart);
      FramePoint feetPointEnd = this.footsteps.get(indexEnd);

      Point2d lineStart = projectXY(feetPointStart);
      Point2d lineEnd = projectXY(feetPointEnd);

      double[] percentages = GeometryTools.getLineSegmentPercentagesIfIntersecting(segmentStart, segmentEnd, lineStart, lineEnd);

      FramePoint intersectionNearStart = null, intersectionNearEnd = null;

      if ((percentages != null) && ((-EPSILON < percentages[1]) && (percentages[1] < 1.0 + EPSILON)))
      {
         if (percentages[0] <= 0.0)
         {
            intersectionNearStart = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
            intersectionNearStart.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
         }
         else if (percentages[0] >= 1.0)
         {
            intersectionNearEnd = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
            intersectionNearEnd.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
         }
         else
         {
            // The line segment straddles the support polygon. Return the one that is inside:
            if (this.isInside(segmentStart))
            {
               intersectionNearStart = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
               intersectionNearStart.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
               intersectionNearEnd = null;

               return new FramePoint[] {intersectionNearStart, intersectionNearEnd};
            }
            else
            {
               intersectionNearStart = null;
               intersectionNearEnd = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
               intersectionNearEnd.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);

               return new FramePoint[] {intersectionNearStart, intersectionNearEnd};
            }
         }
      }

      for (int i = indexEnd.ordinal() + 1; i <= indexStart.ordinal(); i++)
      {
         feetPointStart = feetPointEnd;
         lineStart = lineEnd;

         FramePoint footstep = this.footsteps.get(RobotQuadrant.getQuadrantNameFromOrdinal(i));
         if (useThisLeg(footstep))
         {
            feetPointEnd = footstep;
            lineEnd = projectXY(feetPointEnd);

            percentages = GeometryTools.getLineSegmentPercentagesIfIntersecting(segmentStart, segmentEnd, lineStart, lineEnd);

            if ((percentages != null) && ((-EPSILON < percentages[1]) && (percentages[1] < 1.0 + EPSILON)))
            {
               if (percentages[0] <= 0.0)
               {
                  FramePoint newIntersectionNearStart = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
                  newIntersectionNearStart.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
                  if (intersectionNearStart != null)
                  {
                     double distanceOne = segmentStartFramePoint.getXYPlaneDistance(intersectionNearStart);
                     double distanceTwo = segmentStartFramePoint.getXYPlaneDistance(newIntersectionNearStart);
                     if (distanceTwo < distanceOne)
                        intersectionNearStart = newIntersectionNearStart;
                  }
                  else
                  {
                     intersectionNearStart = newIntersectionNearStart;
                  }
               }
               else if (percentages[0] >= 1.0)
               {
                  FramePoint newIntersectionNearEnd = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
                  newIntersectionNearEnd.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);

                  if (intersectionNearEnd != null)
                  {
                     double distanceOne = segmentEndFramePoint.getXYPlaneDistance(intersectionNearEnd);
                     double distanceTwo = segmentEndFramePoint.getXYPlaneDistance(newIntersectionNearEnd);

                     if (distanceTwo < distanceOne)
                        intersectionNearEnd = newIntersectionNearEnd;
                  }
                  else
                  {
                     intersectionNearEnd = newIntersectionNearEnd;
                  }
               }
               else
               {
                  // The line segment straddles the support polygon. Return the one that is inside:
                  if (this.isInside(segmentStart))
                  {
                     intersectionNearStart = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
                     intersectionNearStart.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
                     intersectionNearEnd = null;

                     return new FramePoint[] {intersectionNearStart, intersectionNearEnd};
                  }
                  else
                  {
                     intersectionNearStart = null;
                     intersectionNearEnd = new FramePoint(feetPointStart.getReferenceFrame());    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);
                     intersectionNearEnd.interpolate(feetPointStart, feetPointEnd, percentages[1]);    // FramePoint.morph(segmentStartFramePoint, segmentEndFramePoint, percentages[0]);

                     return new FramePoint[] {intersectionNearStart, intersectionNearEnd};
                  }
               }

            }
         }
      }

      return new FramePoint[] {intersectionNearStart, intersectionNearEnd};
   }
   
   public QuadrupedSupportPolygon deleteLegCopy(RobotQuadrant legName)
   {
      QuadrupedSupportPolygon newPolygon = new QuadrupedSupportPolygon(this);
      newPolygon.deleteLeg(legName);
      return newPolygon;
   }

   public void deleteLeg(RobotQuadrant legName)
   {
      footsteps.remove(legName);
   }

   /**
    *  This method returns the common support polygon bewteen this and the supplied supportPolygon
    *
    *  *** Assumes regular gait 
    *
    * @param nextSupportPolygon SupportPolygon
    *        1) must contain only three kegs
    *        2) exactly two footsteps must be the same
    * @param specifiedLegName LegName is the legname to assign to the intersection footstep
    *        3) must be one of the swinging (same side) leg names
    * @return SupportPolygon That is common to both (can be null for opposite side swing legs)
    *        This should be the two matching feet and the intersection of the two tror lines
    */
   public QuadrupedSupportPolygon getCommonSupportPolygon(QuadrupedSupportPolygon nextSupportPolygon, RobotQuadrant specifiedLegName)
   {
      // verify both have exactly three legs
      if (this.getNumberOfLegs() != 3)
         throw new IllegalArgumentException("This supportPolygon must contain exactly three legs not " + this.getNumberOfLegs());
      if (nextSupportPolygon.getNumberOfLegs() != 3)
         throw new IllegalArgumentException("Next supportPolygon must contain exactly three legs not " + nextSupportPolygon.getNumberOfLegs());

      // return null if swing legs are not same side *** Assumes regular gait ***
      RobotQuadrant currentSwingLeg = this.getFirstNullQuadrant();
      RobotQuadrant nextSwingLeg = nextSupportPolygon.getFirstNullQuadrant();
      if (nextSwingLeg != currentSwingLeg.getSameSideQuadrant())
         return null;

      // verify exactly two legs epsilon match
      if (this.getNumberOfMatchinfFootsteps(nextSupportPolygon) != 2)
         throw new IllegalArgumentException("There must be exactly two similar foosteps not " + this.getNumberOfMatchinfFootsteps(nextSupportPolygon));

      // verify specified swing leg name is one of the swinging (same side) leg names
      if ((specifiedLegName != currentSwingLeg) && (specifiedLegName != nextSwingLeg))
         throw new IllegalArgumentException("The specified swingleg must be one of the swinging (same side) leg names");

      // if swing legs are the same side find intersection of two trot lines
      // this is the first trot line
      Point2d point1 = new Point2d(this.getFootstep(nextSwingLeg).getX(), this.getFootstep(nextSwingLeg).getY());
      Point2d point1B = new Point2d(this.getFootstep(nextSwingLeg.getDiagonalOppositeQuadrant()).getX(),
                                    this.getFootstep(nextSwingLeg.getDiagonalOppositeQuadrant()).getY());
      point1B.sub(point1);
      Vector2d vector1 = new Vector2d(point1B);

      // this is the second trot line
      Point2d point2 = new Point2d(nextSupportPolygon.getFootstep(currentSwingLeg).getX(), nextSupportPolygon.getFootstep(currentSwingLeg).getY());
      Point2d point2B = new Point2d(nextSupportPolygon.getFootstep(currentSwingLeg.getDiagonalOppositeQuadrant()).getX(),
                                    nextSupportPolygon.getFootstep(currentSwingLeg.getDiagonalOppositeQuadrant()).getY());
      point2B.sub(point2);
      Vector2d vector2 = new Vector2d(point2B);

      // this is the intersection
      Point2d intersection2d = GeometryTools.getIntersectionBetweenTwoLines(point1, vector1, point2, vector2);

      // this is a footstep based on the intersection
      FramePoint position = new FramePoint(this.getReferenceFrame(), intersection2d.x, intersection2d.y, 0.0);
      
      QuadrantDependentList<FramePoint> commonFeetPositions = new QuadrantDependentList<FramePoint>();
      commonFeetPositions.put(specifiedLegName, position);
      
      commonFeetPositions.put(currentSwingLeg.getAcrossBodyQuadrant(), getFootstep(currentSwingLeg.getAcrossBodyQuadrant()));
      commonFeetPositions.put(currentSwingLeg.getDiagonalOppositeQuadrant(), getFootstep(currentSwingLeg.getDiagonalOppositeQuadrant()));

      return new QuadrupedSupportPolygon(commonFeetPositions);
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
    * @param specifiedLegName LegName is the legname to assign to the intersection footstep
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
   public QuadrupedSupportPolygon getShrunkenCommonSupportPolygon(QuadrupedSupportPolygon nextSupportPolygon, RobotQuadrant specifiedLegName, double frontDistance,
           double sideDistance, double hindDistance)
   {
      // get common polygon
      QuadrupedSupportPolygon commonSupportPolygon = getCommonSupportPolygon(nextSupportPolygon, specifiedLegName);

      // if it is null return null
      if (commonSupportPolygon == null)
         return null;

      // find legs invovled
      RobotQuadrant swingLeg = commonSupportPolygon.getFirstNullQuadrant();
      RobotQuadrant oppositeFront, oppositeHind;
      if (swingLeg.isQuadrantInFront())
      {
         oppositeFront = swingLeg.getAcrossBodyQuadrant();
         oppositeHind = swingLeg.getDiagonalOppositeQuadrant();
      }
      else
      {
         oppositeFront = swingLeg.getDiagonalOppositeQuadrant();
         oppositeHind = swingLeg.getAcrossBodyQuadrant();
      }

      // define the starting sides
      FramePoint swingLegSameSideFootstep = new FramePoint(commonSupportPolygon.getFootstep(swingLeg.getSameSideQuadrant()));
      FramePoint oppositeFrontFootstep = new FramePoint(commonSupportPolygon.getFootstep(oppositeFront));
      FramePoint oppositeHindFootstep = new FramePoint(commonSupportPolygon.getFootstep(oppositeHind));
      Point2d point1 = QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep);
      Point2d point2 = QuadrupedSupportPolygon.projectXY(oppositeFrontFootstep);
      Vector2d frontVector;
      if (swingLeg.isQuadrantOnLeftSide())
      {
         point1.sub(point2);
         frontVector = new Vector2d(point1);
      }
      else
      {
         point2.sub(point1);
         frontVector = new Vector2d(point2);
      }

      point2 = QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep);
      point1 = QuadrupedSupportPolygon.projectXY(oppositeHindFootstep);
      Vector2d hindVector;
      if (swingLeg.isQuadrantOnLeftSide())
      {
         point1.sub(point2);
         hindVector = new Vector2d(point1);
      }
      else
      {
         point2.sub(point1);
         hindVector = new Vector2d(point2);
      }

      point1 = QuadrupedSupportPolygon.projectXY(oppositeFrontFootstep);
      point2 = QuadrupedSupportPolygon.projectXY(oppositeHindFootstep);
      Vector2d sideVector;
      if (swingLeg.isQuadrantOnLeftSide())
      {
         point1.sub(point2);
         sideVector = new Vector2d(point1);
      }
      else
      {
         point2.sub(point1);
         sideVector = new Vector2d(point2);
      }

      // first shrink the front side
      // find normal perpendicular to side
      Vector2d directionOfShrinkage = GeometryTools.getPerpendicularVector(frontVector);
      directionOfShrinkage.normalize();

      // scale by desired distance
      directionOfShrinkage.scale(frontDistance);

      // apply to each foot
      point1 = QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep);
      point2 = QuadrupedSupportPolygon.projectXY(oppositeFrontFootstep);
      point1.add(directionOfShrinkage);
      point2.add(directionOfShrinkage);

      // find intersection of new side with each existing side
      Vector2d point1ToPoint2 = new Vector2d(point1);
      point1ToPoint2.sub(point2);
      Point2d newSwingLegSameSidePoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
                                            QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep), hindVector);
      Point2d oppositeFrontPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
                                      QuadrupedSupportPolygon.projectXY(oppositeFrontFootstep), sideVector);

      // update foot positions
      FramePoint position = new FramePoint(commonSupportPolygon.getReferenceFrame(), newSwingLegSameSidePoint.x, newSwingLegSameSidePoint.y, 0.0);
//      swingLegSameSideFootstep = new CartesianFramePoint(position, swingLeg.getSameSideQuadrant());
      swingLegSameSideFootstep = new FramePoint(position);
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeFrontPoint.x, oppositeFrontPoint.y, 0.0);
//      oppositeFrontFootstep = new CartesianFramePoint(position, oppositeFront);
      oppositeFrontFootstep = new FramePoint(position);

      // now shrink the hind side
      // find normal perpendicular to side
      directionOfShrinkage = GeometryTools.getPerpendicularVector(hindVector);
      directionOfShrinkage.normalize();

      // scale by desired distance
      directionOfShrinkage.scale(hindDistance);

      // apply to each foot
      point1 = QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep);
      point2 = QuadrupedSupportPolygon.projectXY(oppositeHindFootstep);
      point1.add(directionOfShrinkage);
      point2.add(directionOfShrinkage);

      // find intersection of new side with each existing side
      point1ToPoint2 = new Vector2d(point1);
      point1ToPoint2.sub(point2);
      newSwingLegSameSidePoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
              QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep), frontVector);
      Point2d oppositeHindPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
                                     QuadrupedSupportPolygon.projectXY(oppositeFrontFootstep), sideVector);

      // update foot positions
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), newSwingLegSameSidePoint.x, newSwingLegSameSidePoint.y, 0.0);
//      swingLegSameSideFootstep = new CartesianFramePoint(position, swingLeg.getSameSideQuadrant());
      swingLegSameSideFootstep = new FramePoint(position);
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeHindPoint.x, oppositeHindPoint.y, 0.0);
//      oppositeHindFootstep = new CartesianFramePoint(position, oppositeHind);
      oppositeHindFootstep = new FramePoint(position);

      // now shrink the remaining side
      // find normal perpendicular to side
      directionOfShrinkage = GeometryTools.getPerpendicularVector(sideVector);
      directionOfShrinkage.normalize();

      // scale by desired distance
      directionOfShrinkage.scale(sideDistance);

      // apply to each foot
      point1 = QuadrupedSupportPolygon.projectXY(oppositeFrontFootstep);
      point2 = QuadrupedSupportPolygon.projectXY(oppositeHindFootstep);
      point1.add(directionOfShrinkage);
      point2.add(directionOfShrinkage);

      // find intersection of new side with each existing side
      point1ToPoint2 = new Vector2d(point1);
      point1ToPoint2.sub(point2);
      oppositeFrontPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
              QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep), frontVector);
      oppositeHindPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
              QuadrupedSupportPolygon.projectXY(swingLegSameSideFootstep), hindVector);

      // update foot positions
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeFrontPoint.x, oppositeFrontPoint.y, 0.0);
//      oppositeFrontFootstep = new CartesianFramePoint(position, oppositeFront);
      oppositeFrontFootstep = new FramePoint(position);
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeHindPoint.x, oppositeHindPoint.y, 0.0);
//      oppositeHindFootstep = new CartesianFramePoint(position, oppositeHind);
      oppositeHindFootstep = new FramePoint(position);

      // now add new footsteps to shrunken polygon
      QuadrantDependentList<FramePoint> commonFootsteps = new QuadrantDependentList<FramePoint>();
//      ArrayList<FramePoint> commonFootsteps = new ArrayList<FramePoint>();
      commonFootsteps.put(swingLeg.getSameSideQuadrant(), swingLegSameSideFootstep);
      commonFootsteps.put(oppositeFront, oppositeFrontFootstep);
      commonFootsteps.put(oppositeHind, oppositeHindFootstep);

      return new QuadrupedSupportPolygon(commonFootsteps);
   }




   /**
    * If this and the given supportPolygon differ only by one footstep, this returns the leg that differs.
    * Else it returns null
    * @param supportPolygonEnd SupportPolygon
    * @return LegName
    */
   public RobotQuadrant getSwingLegFromHereToNextPolygon(QuadrupedSupportPolygon nextSupportPolygon)
   {
      RobotQuadrant swingLeg = null;

      // First Check using ==
      for (RobotQuadrant legName : RobotQuadrant.values())
      {
         if (getFootstep(legName) != nextSupportPolygon.getFootstep(legName))
         {
            if (swingLeg != null)
            {
               swingLeg = null;

               break;
            }

            swingLeg = legName;
         }
      }

      if (swingLeg != null)
         return swingLeg;

      // If that doesn't give an answer, then check using isEpsilonEqualTo
      for (RobotQuadrant legName : RobotQuadrant.values())
      {
         if (!getFootstep(legName).epsilonEquals(nextSupportPolygon.getFootstep(legName), 1e-5))
         {
            if (swingLeg != null)
            {
               return null;
            }

            swingLeg = legName;
         }
      }

      return swingLeg;
   }

   public double distanceInsideTrotLine(Point2d point)
   {
      if (this.getNumberOfLegs() == 4)
         return 0.0;

      RobotQuadrant swingLeg = getFirstNullQuadrant();

      FramePoint pointStart = footsteps.get(swingLeg.getAcrossBodyQuadrant());
      FramePoint pointEnd = footsteps.get(swingLeg.getSameSideQuadrant());

      Point2d lineStart = projectXY(pointStart);
      Point2d lineEnd = projectXY(pointEnd);

      boolean inside = GeometryTools.isPointOnLeftSideOfLine(point, lineStart, lineEnd);

      if ((swingLeg == RobotQuadrant.HIND_LEFT) || (swingLeg == RobotQuadrant.FRONT_RIGHT))
      {
         inside = !inside;
      }

      double distance = GeometryTools.distanceFromPointToLine(point, lineStart, lineEnd);

      if (!inside)
         distance = -distance;

      return distance;
   }


   /**
    * getTrotLineMiddle
    *
    * @return FramePoint
    */
   public FramePoint getTrotLineMiddle()
   {
      RobotQuadrant swingLeg = getFirstNullQuadrant();

      FramePoint footstep1 = footsteps.get(swingLeg.getAcrossBodyQuadrant());
      FramePoint footstep2 = footsteps.get(swingLeg.getSameSideQuadrant());

      FramePoint ret = new FramePoint(footstep1);
      ret.add(footstep2);

      ret.scale(0.5);

      return ret;
   }

   public FrameVector get2DVectorPerpendicularToTrotLinePointingInside()
   {
      RobotQuadrant swingLeg = getFirstNullQuadrant();

      FramePoint footstep1 = footsteps.get(swingLeg.getAcrossBodyQuadrant());
      FramePoint footstep2 = footsteps.get(swingLeg.getSameSideQuadrant());

      FrameVector diffVector = new FrameVector(footstep2);
      diffVector.sub(footstep1);

      diffVector.setZ(0.0);
      double x = diffVector.getX();
      double y = diffVector.getY();

      if (swingLeg == RobotQuadrant.HIND_LEFT)
      {
         diffVector.setX(y);
         diffVector.setY(-x);
         diffVector.normalize();
      }

      else if (swingLeg == RobotQuadrant.HIND_RIGHT)
      {
         diffVector.setX(-y);
         diffVector.setY(x);
         diffVector.normalize();
      }

      else
      {
         throw new RuntimeException("Not implemented for fronts!");
      }



      return diffVector;
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
      if (getNumberOfLegs() == 4)
      {
         xFL = footsteps.get(RobotQuadrant.FRONT_LEFT).getX();
         yFL = footsteps.get(RobotQuadrant.FRONT_LEFT).getY();
                       
         xFR = footsteps.get(RobotQuadrant.FRONT_RIGHT).getX();
         yFR = footsteps.get(RobotQuadrant.FRONT_RIGHT).getY();
                       
         xHR = footsteps.get(RobotQuadrant.HIND_RIGHT).getX();
         yHR = footsteps.get(RobotQuadrant.HIND_RIGHT).getY();
                       
         xHL = footsteps.get(RobotQuadrant.HIND_LEFT).getX();
         yHL = footsteps.get(RobotQuadrant.HIND_LEFT).getY();
      }
      else
      {
         xFL = (footsteps.get(RobotQuadrant.FRONT_LEFT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.FRONT_LEFT).getX();
         yFL = (footsteps.get(RobotQuadrant.FRONT_LEFT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.FRONT_LEFT).getY();
         
         xFR = (footsteps.get(RobotQuadrant.FRONT_RIGHT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.FRONT_RIGHT).getX();
         yFR = (footsteps.get(RobotQuadrant.FRONT_RIGHT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.FRONT_RIGHT).getY();
                
         xHR = (footsteps.get(RobotQuadrant.HIND_RIGHT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.HIND_RIGHT).getX();
         yHR = (footsteps.get(RobotQuadrant.HIND_RIGHT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.HIND_RIGHT).getY();
                
         xHL = (footsteps.get(RobotQuadrant.HIND_LEFT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.HIND_LEFT).getX();
         yHL = (footsteps.get(RobotQuadrant.HIND_LEFT) == null) ? Double.NaN : footsteps.get(RobotQuadrant.HIND_LEFT).getY();
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
    * replaceFootstepCopy: this replaces the stored footsep with the passed in one.
    * If the stored footstep is null, it will throw an exception.
    *
    * @param footstep Footstep
    * @return Polygon
    */
   public QuadrupedSupportPolygon replaceFootstepCopy(RobotQuadrant quadrant, FramePoint footstep)
   {
      QuadrupedSupportPolygon newPolygon = new QuadrupedSupportPolygon(this);
      newPolygon.setFootstep(quadrant, footstep);
      return newPolygon;
   }

   /**
    *  getDiagonalIntersection returns a point that has a value of the intersection
    *  of the diagonals in a quad stance.
    * 
    *  @return Point3d  A point that is the diagonal intersection of the quad stance
    *                   null if stance is not a quad
    */
   public FramePoint getDiagonalIntersection()
   {
      // Copied from http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
      // Uses the P = P1 + alpha (P2-P1) method rather than y = mx+b and thus doesn't have problems with vertical lines.

      // Footstep[] footstepArray = this.getFootsteps();

      FramePoint frontLeftFootstep = footsteps.get(RobotQuadrant.FRONT_LEFT);
      FramePoint frontRightFootstep = footsteps.get(RobotQuadrant.FRONT_RIGHT);
      FramePoint hindRightFootstep = footsteps.get(RobotQuadrant.HIND_RIGHT);
      FramePoint hindLeftFootstep = footsteps.get(RobotQuadrant.HIND_LEFT);

      if ((frontLeftFootstep == null) || (frontRightFootstep == null) || (hindRightFootstep == null) || (hindLeftFootstep == null))
      {
         return null;
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

      return new FramePoint(getReferenceFrame(), xp, yp, 0.0);
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
      if (this.getNumberOfLegs() != 2)
         return false;

      RobotQuadrant firstLeg = getFirstNonNullQuadrant();
      RobotQuadrant diagonalLeg = firstLeg.getDiagonalOppositeQuadrant();
      
      if (footsteps.get(diagonalLeg) == null)
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
      if (!this.isValidTrotPolygon())
         throw new RuntimeException("SupportPolygon is not a ValidTrotPolygon");

      RobotQuadrant firstLeg = getFirstNonNullQuadrant();
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
      if (!this.isValidTrotPolygon())
         throw new RuntimeException("SupportPolygon is not a ValidTrotPolygon");

      RobotQuadrant firstLeg = getFirstNonNullQuadrant();
      if(firstLeg.isQuadrantOnRightSide())
      {
         return firstLeg;
      }
      
      return firstLeg.getDiagonalOppositeQuadrant();
   }


   public FramePoint getAverageFrontPoint()
   {
      FramePoint frontLeft = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint frontRight = this.getFootstep(RobotQuadrant.FRONT_RIGHT);

      if ((frontLeft != null) && (frontRight != null))
      {
         FramePoint ret = new FramePoint(frontLeft);
         ret.add(frontRight);
         ret.scale(0.5);

         return ret;
      }

      else if (frontLeft != null)
         return new FramePoint(frontLeft);
      else if (frontRight != null)
         return new FramePoint(frontRight);

      else
         throw new RuntimeException("SupportPolygon.getAverageFrontPoint(): Both front legs are null!");
   }


   public FramePoint getAverageHindPoint()
   {
      FramePoint hindLeft = this.getFootstep(RobotQuadrant.HIND_LEFT);
      FramePoint hindRight = this.getFootstep(RobotQuadrant.HIND_RIGHT);

      if ((hindLeft != null) && (hindRight != null))
      {
         FramePoint ret = new FramePoint(hindLeft);
         ret.add(hindRight);
         ret.scale(0.5);

         return ret;
      }

      else if (hindLeft != null)
         return new FramePoint(hindLeft);
      else if (hindRight != null)
         return  new FramePoint(hindRight);

      else
         throw new RuntimeException("SupportPolygon.getAverageFrontPoint(): Both hind legs are null!");

   }
   
   public static void main(String[] args)
   {
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();
      QuadrantDependentList<FramePoint> footsteps = new QuadrantDependentList<FramePoint>();
      
      
      FramePoint framePoint;

      framePoint = new FramePoint(frame, 0.0, 0.0, 0.0);
      footsteps.put(RobotQuadrant.FRONT_LEFT, new FramePoint(framePoint));

      framePoint = new FramePoint(frame, 1.0, 0.0, 0.0);
      footsteps.put(RobotQuadrant.FRONT_RIGHT, new FramePoint(framePoint));

      framePoint = new FramePoint(frame, 1.0, -1.0, 0.0);
      footsteps.put(RobotQuadrant.HIND_RIGHT, new FramePoint(framePoint));

      QuadrupedSupportPolygon polygon = new QuadrupedSupportPolygon(footsteps);

      Point2d point = new Point2d(0.5, 0.25);
      System.out.println("DI=" + polygon.distanceInside(point));
   }

   public boolean getTangentTangentRadiusCircleCenter(RobotQuadrant robotQuadrantToAnchorTo, double radius, Point2d centerToPack)
   {
      if(useThisLeg(robotQuadrantToAnchorTo) && getNumberOfLegs() == 3)
      {
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
         bisectorVector.scale(radiusOffsetAlongBisector / bisectorVector.length());
         
         vertex2d.add(bisectorVector);
         centerToPack.set(vertex2d);
         return true;
      }
      
      return false;
      
      
   }
}
