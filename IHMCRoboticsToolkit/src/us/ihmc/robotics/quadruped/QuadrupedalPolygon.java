package us.ihmc.robotics.quadruped;

import java.io.Serializable;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
public class QuadrupedalPolygon implements Serializable
{
   protected final QuadrupedFootstep[] footstepArray = new QuadrupedFootstep[4];

   public QuadrupedalPolygon()
   {
   }

   public static QuadrupedalPolygon createPolygon(ReferenceFrame referenceFrame, double[][] footPositions)
   {
      FramePoint[] footPositionFramePoints = new FramePoint[4];

      for (RobotQuadrant legName : RobotQuadrant.values())
      {
         if (footPositions[legName.ordinal()] != null)
         {
            FramePoint footPosition = new FramePoint(referenceFrame, footPositions[legName.ordinal()]);

            footPositionFramePoints[legName.ordinal()] = footPosition;
         }
      }

      return createPolygon(referenceFrame, footPositionFramePoints);
   }


   public static QuadrupedalPolygon createPolygon(ReferenceFrame referenceFrame, FramePoint[] footPositions)
   {
      QuadrupedFootstep[] footsteps = new QuadrupedFootstep[4];

      for (RobotQuadrant legName : RobotQuadrant.values())
      {
         FramePoint footPosition = footPositions[legName.ordinal()];

         if (footPosition != null)
         {
            CartesianQuadrupedFootstep footstep = new CartesianQuadrupedFootstep(footPosition, legName);
            footsteps[legName.ordinal()] = footstep;
         }
      }

      return new QuadrupedalPolygon(footsteps);
   }

   /**
    * Constructs a polygon from an arraylist of footsteps.  The footsteps must
    * be in a particular order to guarantee that we have a convex polygon.
    *
    * @param footsteps ArrayList
    */
   public QuadrupedalPolygon(ArrayList<QuadrupedFootstep> footsteps)
   {
      // first check for duplicate legNames
      ArrayList<RobotQuadrant> legNames = RobotQuadrant.getAllQuadrants();
      for (QuadrupedFootstep footstep : footsteps)
      {
         RobotQuadrant legName = footstep.getLegName();

         if (legNames.contains(legName))
         {
            legNames.remove(legName);
         }
         else
         {
            throw new RuntimeException("Duplicate LegName in footsteps array");
         }
      }

      // now populate the footstepArray
      for (QuadrupedFootstep footstep : footsteps)
      {
         this.footstepArray[footstep.getLegName().ordinal()] = footstep;
      }
   }

   public void printOutPolygon(String string)
   {
      System.out.print(string + " ");

      for (QuadrupedFootstep footstep : this.getFootsteps())
      {
         System.out.print(footstep.getLegName() + ": " + footstep.getPositionFramePointCopy() + "  , ");
      }

      System.out.println("\n");
   }


   /**
    * Constructs a polygon from four footsteps.  The footsteps must
    * not have repeat leg names.
    *
    * @param footstep0 Footstep
    * @param footstep1 Footstep
    * @param footstep2 Footstep
    * @param footstep3 Footstep
    */
   public QuadrupedalPolygon(QuadrupedFootstep footstep0, QuadrupedFootstep footstep1, QuadrupedFootstep footstep2, QuadrupedFootstep footstep3)
   {
      // first check for duplicate legNames
      ArrayList<RobotQuadrant> legNames = RobotQuadrant.getAllQuadrants();

      // footstep0:
      RobotQuadrant legName = footstep0.getLegName();

      if (legNames.contains(legName))
      {
         legNames.remove(legName);
      }
      else
      {
         throw new RuntimeException("Duplicate LegName in footsteps array");
      }

      this.footstepArray[footstep0.getLegName().ordinal()] = footstep0;

      // footstep1:
      legName = footstep1.getLegName();

      if (legNames.contains(legName))
      {
         legNames.remove(legName);
      }
      else
      {
         throw new RuntimeException("Duplicate LegName in footsteps array");
      }

      this.footstepArray[footstep1.getLegName().ordinal()] = footstep1;

      // footstep2:
      legName = footstep2.getLegName();

      if (legNames.contains(legName))
      {
         legNames.remove(legName);
      }
      else
      {
         throw new RuntimeException("Duplicate LegName in footsteps array");
      }

      this.footstepArray[footstep2.getLegName().ordinal()] = footstep2;

      // footstep3:
      legName = footstep3.getLegName();

      if (legNames.contains(legName))
      {
         legNames.remove(legName);
      }
      else
      {
         throw new RuntimeException("Duplicate LegName in footsteps array");
      }

      this.footstepArray[footstep3.getLegName().ordinal()] = footstep3;
   }

// /**
//  * Constructs a polygon from an arraylist of footsteps.  The footsteps must
//  * be in a particular order to guarantee that we have a convex polygon.
//  *
//  * @param footsteps ArrayList
//  */
// public Polygon(ArrayList<Footstep> footsteps)
// {
//    int previousLegIndex = -1;
//    for (Footstep footstep : footsteps)
//    {
//       if (footstep != null)
//       {
//          if (footstep.getLegName().ordinal() <= previousLegIndex)
//          {
//             throw new RuntimeException("Footsteps must be listed in LegName order.");
//          }
//          this.footstepArray[footstep.getLegName().ordinal()] = footstep;
//          previousLegIndex = footstep.getLegName().ordinal();
//       }
//    }
// }



   public QuadrupedalPolygon(QuadrupedFootstep[] supportPolygon)
   {
      // need to check that the index matches the ordinal of the leg name
      for (int i = 0; i < footstepArray.length; i++)
      {
         if (supportPolygon[i] != null)
            if (i != supportPolygon[i].getLegName().ordinal())
               throw new RuntimeException("array of footsteps must be in LegName order");
         footstepArray[i] = supportPolygon[i];
      }
   }

   public QuadrupedalPolygon(QuadrupedalPolygon polygon)
   {
      for (int i = 0; i < footstepArray.length; i++)
      {
         footstepArray[i] = polygon.footstepArray[i];
      }
   }

   /**
    * Combines the (disjoint) foot vertices from two polygons into a
    * single polygon.  A check is made to make sure that the polygons
    * don't define the same foot.
    *
    * @param firstPolygon Polygon
    * @param secondPolygon Polygon
    */

   public QuadrupedalPolygon(QuadrupedalPolygon firstPolygon, QuadrupedalPolygon secondPolygon)
   {
      if (firstPolygon == null)
      {
         for (int i = 0; i < footstepArray.length; i++)
         {
            footstepArray[i] = secondPolygon.footstepArray[i];
         }

         return;
      }

      else if (secondPolygon == null)
      {
         for (int i = 0; i < footstepArray.length; i++)
         {
            footstepArray[i] = firstPolygon.footstepArray[i];
         }

         return;
      }

      for (int i = 0; i < footstepArray.length; i++)
      {
         // Check that both polygons don't define a particular foot.
         if (useThisLeg(firstPolygon.footstepArray[i]) && useThisLeg(secondPolygon.footstepArray[i]))
         {
            throw new RuntimeException("Polygon.Polygon: Both polygons defined footstep " + i);
         }
         else if (useThisLeg(firstPolygon.footstepArray[i]))
         {
            footstepArray[i] = firstPolygon.footstepArray[i];
         }
         else if (useThisLeg(secondPolygon.footstepArray[i]))
         {
            footstepArray[i] = secondPolygon.footstepArray[i];
         }
      }
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
         FramePoint point1 = this.getFootstep(legPairs[i][0]).getPositionFramePointCopy();
         FramePoint point2 = this.getFootstep(legPairs[i][1]).getPositionFramePointCopy();

         Vector2d edgeVector = new Vector2d(point2.getX(), point2.getY());
         edgeVector.sub(new Point2d(point1.getX(), point1.getY()));

         edgeNormals[i] = GeometryTools.getPerpendicularVector(edgeVector);
      }

      return edgeNormals;
   }

   public ReferenceFrame getReferenceFrame()
   {
      // Return the reference frame of the first non-null footstep.
      for (QuadrupedFootstep footstep : footstepArray)
      {
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
   public QuadrupedalPolygon changeFrameCopy(ReferenceFrame referenceFrame)
   {
      QuadrupedFootstep[] newFootsteps = new QuadrupedFootstep[4];

      for (int i = 0; i < 4; i++)
      {
         if (this.footstepArray[i] != null)
         {
            newFootsteps[i] = this.footstepArray[i].changeFrameCopy(referenceFrame);
         }
      }

      return new QuadrupedalPolygon(newFootsteps);
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
            RobotQuadrant firstLeg = RobotQuadrant.getQuadrantNameFromOrdinal(this.getFirstNonNullFootstep());
            RobotQuadrant lastLeg = RobotQuadrant.getQuadrantNameFromOrdinal(this.getLastNonNullFootstep());
            legPairs = new RobotQuadrant[][]
            {
               {firstLeg, lastLeg}
            };

            break;

         case 3 :

            RobotQuadrant swingLeg = null;
            for (QuadrupedFootstep footstep : footstepArray)
            {
               if (footstep == null)
               {
                  swingLeg = footstep.getLegName();
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

   public QuadrupedFootstep getFootstep(RobotQuadrant legName)
   {
      return footstepArray[legName.ordinal()];
   }

   public QuadrupedFootstep[] getFootstepArrayCopy()
   {
      QuadrupedFootstep[] ret = new QuadrupedFootstep[4];

      for (int i = 0; i < 4; i++)
      {
         ret[i] = footstepArray[i];
      }

      return ret;
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
      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
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
   public int getNumberOfMatchinfFootsteps(QuadrupedalPolygon polygon)
   {
      // check the number of null legs
      int numberOfMatching = 0;
      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            QuadrupedFootstep polygonFootstep = polygon.footstepArray[footstep.getLegName().ordinal()];
            if (polygonFootstep != null)
            {
               if (footstep.getPositionFramePointCopy().epsilonEquals(polygonFootstep.getPositionFramePointCopy(), 0.0005))
               {
                  numberOfMatching++;
               }
            }
         }
      }

      return numberOfMatching;
   }


   /**
    * getFootsteps returns an list of the footsteps that make up the polygon
    *
    * @return ArrayList  A list of Footsteps that make up the polygon
    */
   public ArrayList<QuadrupedFootstep> getFootsteps()
   {
      ArrayList<QuadrupedFootstep> ret = new ArrayList<QuadrupedFootstep>();
      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            ret.add(footstep);
         }
      }

      return ret;
   }


   public ArrayList<RobotQuadrant> getLegs()
   {
      ArrayList<RobotQuadrant> ret = new ArrayList<RobotQuadrant>();

      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            ret.add(footstep.getLegName());
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
      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            centroid.add(footstep.getPositionFramePointCopy());
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
   public QuadrupedalPolygon getShrunkenPolygon(double shrinkDistance)
   {
      if (getNumberOfLegs() != 3)
      {
         throw new RuntimeException("This method works only for triangle polygons.");
      }

      ArrayList<QuadrupedFootstep> footsteps = getFootsteps();
      for (QuadrupedFootstep footstep : footsteps)
      {
         if (!(footstep instanceof CartesianQuadrupedFootstep))
         {
            throw new RuntimeException("This method works only for Cartesian footsteps.");
         }
      }

      // Cap the shrink amount by half the radius of the largest inscribed circle.
      shrinkDistance = Math.min(shrinkDistance, 0.5 * this.getInCircleRadius3Legs());

      ArrayList<QuadrupedFootstep> newFeet = new ArrayList<QuadrupedFootstep>();

      // Foot 0
      FramePoint footPoint = footsteps.get(0).getPositionFramePointCopy();
      FramePoint point1 = footsteps.get(1).getPositionFramePointCopy();
      FramePoint point2 = footsteps.get(2).getPositionFramePointCopy();
      Vector2d edgeVector = new Vector2d(point2.getX(), point2.getY());
      edgeVector.sub(new Point2d(point1.getX(), point1.getY()));
      Vector3d edgeNormal = new Vector3d(-edgeVector.y, edgeVector.x, 0.0);
      edgeNormal.normalize();
      edgeNormal.scale(shrinkDistance);
      footPoint.add(new FrameVector(footPoint.getReferenceFrame(), edgeNormal));
      newFeet.add(new CartesianQuadrupedFootstep(footPoint, footsteps.get(0).getLegName()));

      // Foot 1
      footPoint = footsteps.get(1).getPositionFramePointCopy();
      point1 = footsteps.get(2).getPositionFramePointCopy();
      point2 = footsteps.get(0).getPositionFramePointCopy();
      edgeVector = new Vector2d(point2.getX(), point2.getY());
      edgeVector.sub(new Point2d(point1.getX(), point1.getY()));
      edgeNormal = new Vector3d(-edgeVector.y, edgeVector.x, 0.0);
      edgeNormal.normalize();
      edgeNormal.scale(shrinkDistance);
      footPoint.add(new FrameVector(footPoint.getReferenceFrame(), edgeNormal));
      newFeet.add(new CartesianQuadrupedFootstep(footPoint, footsteps.get(1).getLegName()));

      // Foot 2
      footPoint = footsteps.get(2).getPositionFramePointCopy();
      point1 = footsteps.get(0).getPositionFramePointCopy();
      point2 = footsteps.get(1).getPositionFramePointCopy();
      edgeVector = new Vector2d(point2.getX(), point2.getY());
      edgeVector.sub(new Point2d(point1.getX(), point1.getY()));
      edgeNormal = new Vector3d(-edgeVector.y, edgeVector.x, 0.0);
      edgeNormal.normalize();
      edgeNormal.scale(shrinkDistance);
      footPoint.add(new FrameVector(footPoint.getReferenceFrame(), edgeNormal));
      newFeet.add(new CartesianQuadrupedFootstep(footPoint, footsteps.get(2).getLegName()));

      return new QuadrupedalPolygon(newFeet);
   }

   /**
    * getLowestFootstep
    *
    * @param removeSwingLeg boolean
    * @return Footstep
    */
   public QuadrupedFootstep getLowestFootstep()
   {
      double minZ = Double.POSITIVE_INFINITY;
      QuadrupedFootstep lowest = null;

      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            double footZ = footstep.getPositionFramePointCopy().getZ();
            if (footZ < minZ)
            {
               minZ = footZ;
               lowest = footstep;
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
   public QuadrupedFootstep getHighestFootstep()
   {
      double maxZ = Double.NEGATIVE_INFINITY;
      QuadrupedFootstep highest = null;

      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            double footZ = footstep.getPositionFramePointCopy().getZ();
            if (footZ > maxZ)
            {
               maxZ = footZ;
               highest = footstep;
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

      for (int i = 0; i < 4; i++)
      {
         QuadrupedFootstep footstep = this.footstepArray[i];
         if (useThisLeg(footstep))
         {
            FramePoint position = footstep.getPositionFramePointCopy();
            if (position.getX() < min.x)
            {
               min.x = position.getX();
            }

            if (position.getY() < min.y)
            {
               min.y = position.getY();
            }

            if (position.getX() > max.x)
            {
               max.x = position.getX();
            }

            if (position.getY() > max.y)
            {
               max.y = position.getY();
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

      int indexStart = getLastNonNullFootstep();
      int indexEnd = getFirstNonNullFootstep();

      // indexStart must be greater than indexEnd
      if (indexStart < indexEnd)
      {
         throw new RuntimeException("SupportPolygon.distanceInside(): indexStart must be > indexEnd");
      }

      FramePoint pointStart = this.footstepArray[indexStart].getPositionFramePointCopy();
      FramePoint pointEnd = this.footstepArray[indexEnd].getPositionFramePointCopy();

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
      for (int i = indexEnd + 1; i <= indexStart; i++)
      {
         lineStart = lineEnd;
         QuadrupedFootstep footstep = this.footstepArray[i];
         if (useThisLeg(footstep))
         {
            lineEnd = projectXY(footstep.getPositionFramePointCopy());

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

      if (numberOfLegs == 3)
      {
         bestInCircleRadius = this.getInCircleRadius3Legs();
         intersection = this.getInCirclePoint3Legs();
         bestInCirclePoint.set(intersection);
      }

      else if (numberOfLegs == 4)
      {
         QuadrupedFootstep a = getFootstep(RobotQuadrant.FRONT_LEFT);
         QuadrupedFootstep b = getFootstep(RobotQuadrant.FRONT_RIGHT);
         QuadrupedFootstep c = getFootstep(RobotQuadrant.HIND_RIGHT);
         QuadrupedFootstep d = getFootstep(RobotQuadrant.HIND_LEFT);

         Point2d p1 = new Point2d(a.getPositionFramePointCopy().getX(), a.getPositionFramePointCopy().getY());
         Point2d p2 = new Point2d(b.getPositionFramePointCopy().getX(), b.getPositionFramePointCopy().getY());
         Point2d p3 = new Point2d(c.getPositionFramePointCopy().getX(), c.getPositionFramePointCopy().getY());
         Point2d p4 = new Point2d(d.getPositionFramePointCopy().getX(), d.getPositionFramePointCopy().getY());

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
      for (QuadrupedFootstep footstep : footstepArray)
      {
         if (useThisLeg(footstep))
            nonSwingLegs.add(footstep.getLegName());
      }

      if (nonSwingLegs.size() != 3)
         throw new RuntimeException("Stance: getInCircleRadiusIgnoreSwingLeg cannot be applied to stance with < 3 nonSwingLegs\n" + this);

      QuadrupedFootstep b = getFootstep(nonSwingLegs.get(1));
      QuadrupedFootstep c = getFootstep(nonSwingLegs.get(2));

      Point2d p2 = new Point2d(b.getPositionFramePointCopy().getX(), b.getPositionFramePointCopy().getY());
      Point2d p3 = new Point2d(c.getPositionFramePointCopy().getX(), c.getPositionFramePointCopy().getY());

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
      for (QuadrupedFootstep footstep : footstepArray)
      {
         if (useThisLeg(footstep))
            nonSwingLegs.add(footstep.getLegName());
      }

      if (nonSwingLegs.size() != 3)
         throw new RuntimeException("Stance: getInCircleRadiusIgnoreSwingLeg cannot be applied to stance with < 3 nonSwingLegs");

      QuadrupedFootstep a = getFootstep(nonSwingLegs.get(0));
      QuadrupedFootstep b = getFootstep(nonSwingLegs.get(1));
      QuadrupedFootstep c = getFootstep(nonSwingLegs.get(2));

      Point2d p1 = new Point2d(a.getPositionFramePointCopy().getX(), a.getPositionFramePointCopy().getY());
      Point2d p2 = new Point2d(b.getPositionFramePointCopy().getX(), b.getPositionFramePointCopy().getY());
      Point2d p3 = new Point2d(c.getPositionFramePointCopy().getX(), c.getPositionFramePointCopy().getY());
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

   public boolean useThisLeg(QuadrupedFootstep footstep)
   {
      if (footstep == null)
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

   /**
    * getLastNonNullFootstep
    *
    * @param removeSwingLeg boolean
    * @return int
    */
   public int getLastNonNullFootstep()
   {
      for (int i = this.footstepArray.length - 1; i >= 0; i--)
      {
         if (useThisLeg(this.footstepArray[i]))
         {
            return i;
         }
      }

      throw new RuntimeException("Polygon: getLastNonNullFootstep(): no last non null footstep");
   }

   /**
    * getFirstNonNullFootstep
    *
    * @param removeSwingLeg boolean
    * @return int
    */
   public int getFirstNonNullFootstep()
   {
      for (int i = 0; i < this.footstepArray.length; i++)
      {
         if (useThisLeg(this.footstepArray[i]))
         {
            return i;
         }
      }

      throw new RuntimeException("Polygon: getFirstNonNullFootstep(): no first non null footstep");
   }

   /**
    * getFirstNonNullFootstep
    *
    * @param removeSwingLeg boolean
    * @return int
    */
   public int getFirstNullFootstep()
   {
      for (int i = 0; i < this.footstepArray.length; i++)
      {
         if (!useThisLeg(this.footstepArray[i]))
         {
            return i;
         }
      }

      throw new RuntimeException("Polygon: getFirstNullFootstep(): no first null footstep");
   }

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
      QuadrupedFootstep flFootstep = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      QuadrupedFootstep frFootstep = this.getFootstep(RobotQuadrant.FRONT_RIGHT);
      QuadrupedFootstep hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      QuadrupedFootstep hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

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
      QuadrupedFootstep hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      QuadrupedFootstep hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

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
      QuadrupedFootstep flFootstep = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      QuadrupedFootstep frFootstep = this.getFootstep(RobotQuadrant.FRONT_RIGHT);
      QuadrupedFootstep hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      QuadrupedFootstep hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

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
      QuadrupedFootstep flFootstep = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      QuadrupedFootstep frFootstep = this.getFootstep(RobotQuadrant.FRONT_RIGHT);
      QuadrupedFootstep hrFootstep = this.getFootstep(RobotQuadrant.HIND_RIGHT);
      QuadrupedFootstep hlFootstep = this.getFootstep(RobotQuadrant.HIND_LEFT);

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

      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            if (footstep.getLegName().isQuadrantInFront())
            {
               footPosition.add(footstep.getPositionFramePointCopy());
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

      for (QuadrupedFootstep footstep : this.footstepArray)
      {
         if (useThisLeg(footstep))
         {
            if (footstep.getLegName().isQuadrantInHind())
            {
               footPosition.add(footstep.getPositionFramePointCopy());
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
   public QuadrupedalPolygon yawAboutCentroid(double yaw)
   {
      FramePoint centroid = getCentroidFramePoint();

      QuadrupedFootstep[] newFootsteps = new QuadrupedFootstep[4];

      for (int i = 0; i < 4; i++)
      {
         if (useThisLeg(this.footstepArray[i]))
         {
            newFootsteps[i] = this.footstepArray[i].yawAboutPointCopy(centroid, yaw);
         }
         else
         {
            newFootsteps[i] = null;
         }
      }

      return new QuadrupedalPolygon(newFootsteps);
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

      int indexStart = getLastNonNullFootstep();
      int indexEnd = getFirstNonNullFootstep();

      // indexStart must be greater than indexEnd
      if (indexStart < indexEnd)
      {
         throw new RuntimeException("SupportPolygon.distanceInside(): indexStart must be > indexEnd");
      }

      FramePoint pointStart = this.footstepArray[indexStart].getPositionFramePointCopy();
      FramePoint pointEnd = this.footstepArray[indexEnd].getPositionFramePointCopy();

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

      for (int i = indexEnd + 1; i <= indexStart; i++)
      {
         lineStart = lineEnd;
         QuadrupedFootstep footstep = this.footstepArray[i];
         if (useThisLeg(footstep))
         {
            counter++;
            lineEnd = projectXY(footstep.getPositionFramePointCopy());

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

   public boolean hasSameFootsteps(QuadrupedalPolygon polyTwo)
   {
      if (footstepArray[RobotQuadrant.FRONT_LEFT_ORDINAL] != polyTwo.footstepArray[RobotQuadrant.FRONT_LEFT_ORDINAL])
         return false;
      if (footstepArray[RobotQuadrant.FRONT_RIGHT_ORDINAL] != polyTwo.footstepArray[RobotQuadrant.FRONT_RIGHT_ORDINAL])
         return false;
      if (footstepArray[RobotQuadrant.HIND_RIGHT_ORDINAL] != polyTwo.footstepArray[RobotQuadrant.HIND_RIGHT_ORDINAL])
         return false;
      if (footstepArray[RobotQuadrant.HIND_LEFT_ORDINAL] != polyTwo.footstepArray[RobotQuadrant.HIND_LEFT_ORDINAL])
         return false;

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

      RobotQuadrant swingLeg = RobotQuadrant.getQuadrantNameFromOrdinal(getFirstNullFootstep());
      Point2d A = projectXY(this.getFootstep(swingLeg.getAcrossBodyQuadrant()).getPositionFramePointCopy());
      Point2d B = projectXY(this.getFootstep(swingLeg.getSameSideQuadrant()).getPositionFramePointCopy());
      Point2d C = projectXY(this.getFootstep(swingLeg.getDiagonalOppositeQuadrant()).getPositionFramePointCopy());
      Point2d bisectingPoint = GeometryTools.getTriangleBisector(A, B, C);

      return new FramePoint(this.getReferenceFrame(), bisectingPoint.x, bisectingPoint.y, 0.0);
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

      QuadrupedFootstep frontFootstep = footstepArray[RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide).ordinal()];
      QuadrupedFootstep endFootstep = footstepArray[RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide).ordinal()];

      return frontFootstep.distanceToFootstepInXY(endFootstep);
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

      QuadrupedFootstep frontLeft = footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()];
      QuadrupedFootstep frontRight = footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()];

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

      QuadrupedFootstep hindLeft = footstepArray[RobotQuadrant.HIND_LEFT.ordinal()];
      QuadrupedFootstep hindRight = footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()];

      hindLeft.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      hindRight.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double vectorX = hindLeft.getX() - hindRight.getX();
      double vectorY = hindLeft.getY() - hindRight.getY();

      Vector2d rightToLeft = new Vector2d(vectorX, vectorY);

      return Math.abs(rightToLeft.dot(perpendicularHeadingVecetor));
   }


   public boolean epsilonEquals(QuadrupedalPolygon polyTwo)
   {
      if (polyTwo == null)
         return false;

      if (footstepArray[RobotQuadrant.FRONT_LEFT_ORDINAL] != null)
      {
         if (polyTwo.footstepArray[RobotQuadrant.FRONT_LEFT_ORDINAL] == null)
            return false;
         if (!footstepArray[RobotQuadrant.FRONT_LEFT_ORDINAL].getPositionFramePointCopy().epsilonEquals(
                 polyTwo.footstepArray[RobotQuadrant.FRONT_LEFT_ORDINAL].getPositionFramePointCopy(), 0.0005))
            return false;
      }

      if (footstepArray[RobotQuadrant.FRONT_RIGHT_ORDINAL] != null)
      {
         if (polyTwo.footstepArray[RobotQuadrant.FRONT_RIGHT_ORDINAL] == null)
            return false;
         if (!footstepArray[RobotQuadrant.FRONT_RIGHT_ORDINAL].getPositionFramePointCopy().epsilonEquals(
                 polyTwo.footstepArray[RobotQuadrant.FRONT_RIGHT_ORDINAL].getPositionFramePointCopy(), 0.0005))
            return false;
      }

      if (footstepArray[RobotQuadrant.HIND_RIGHT_ORDINAL] != null)
      {
         if (polyTwo.footstepArray[RobotQuadrant.HIND_RIGHT_ORDINAL] == null)
            return false;
         if (!footstepArray[RobotQuadrant.HIND_RIGHT_ORDINAL].getPositionFramePointCopy().epsilonEquals(
                 polyTwo.footstepArray[RobotQuadrant.HIND_RIGHT_ORDINAL].getPositionFramePointCopy(), 0.0005))
            return false;
      }

      if (footstepArray[RobotQuadrant.HIND_LEFT_ORDINAL] != null)
      {
         if (polyTwo.footstepArray[RobotQuadrant.HIND_LEFT_ORDINAL] == null)
            return false;
         if (!footstepArray[RobotQuadrant.HIND_LEFT_ORDINAL].getPositionFramePointCopy().epsilonEquals(
                 polyTwo.footstepArray[RobotQuadrant.HIND_LEFT_ORDINAL].getPositionFramePointCopy(), 0.0005))
            return false;
      }

      return true;
   }

   public String toString()
   {
      String string = "";

      for (QuadrupedFootstep footstep : footstepArray)
      {
         if (footstep != null)
            string += footstep.toString() + " ";
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

      int indexStart = getLastNonNullFootstep();
      int indexEnd = getFirstNonNullFootstep();

      // indexStart must be greater than indexEnd
      if (indexStart < indexEnd)
      {
         throw new RuntimeException("SupportPolygon.getIntersectingPointsOfSegment(): indexStart must be > indexEnd");
      }

      FramePoint feetPointStart = this.footstepArray[indexStart].getPositionFramePointCopy();
      FramePoint feetPointEnd = this.footstepArray[indexEnd].getPositionFramePointCopy();

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

      for (int i = indexEnd + 1; i <= indexStart; i++)
      {
         feetPointStart = feetPointEnd;
         lineStart = lineEnd;

         QuadrupedFootstep footstep = this.footstepArray[i];
         if (useThisLeg(footstep))
         {
            feetPointEnd = footstep.getPositionFramePointCopy();
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
   
   public QuadrupedFootstep getFootstep(int index)
   {
      return footstepArray[index];
   }

   public static void main(String[] args)
   {
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();

      FramePoint framePoint;

      framePoint = new FramePoint(frame, 0.0, 0.0, 0.0);
      QuadrupedFootstep footstepFL = new CartesianQuadrupedFootstep(framePoint, RobotQuadrant.FRONT_LEFT);

      framePoint = new FramePoint(frame, 1.0, 0.0, 0.0);
      QuadrupedFootstep footstepFR = new CartesianQuadrupedFootstep(framePoint, RobotQuadrant.FRONT_RIGHT);

      framePoint = new FramePoint(frame, 1.0, -1.0, 0.0);
      QuadrupedFootstep footstepHR = new CartesianQuadrupedFootstep(framePoint, RobotQuadrant.HIND_RIGHT);

      ArrayList<QuadrupedFootstep> footsteps = new ArrayList<QuadrupedFootstep>();
      footsteps.add(footstepFL);
      footsteps.add(footstepFR);

//    footsteps.add(footstepHR);

      QuadrupedalPolygon polygon = new QuadrupedalPolygon(footsteps);

      Point2d point = new Point2d(0.5, 0.25);
      System.out.println("DI=" + polygon.distanceInside(point));
   }
}
