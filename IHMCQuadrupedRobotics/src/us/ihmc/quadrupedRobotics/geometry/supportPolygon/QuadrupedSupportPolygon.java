package us.ihmc.quadrupedRobotics.geometry.supportPolygon;

import static us.ihmc.robotics.robotSide.RobotQuadrant.*;

import java.io.Serializable;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RecyclingQuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class QuadrupedSupportPolygon implements Serializable
{
   private static final long serialVersionUID = 4247638266737494462L;
   
   private final RecyclingQuadrantDependentList<FramePoint> footsteps = new RecyclingQuadrantDependentList<>(FramePoint.class);
   
   private final FrameConvexPolygon2d tempFrameConvexPolygon2d = new FrameConvexPolygon2d();
   
   private final FramePoint temporaryFramePoint = new FramePoint();
   private final FrameVector tempPlaneNormalInWorld = new FrameVector();
   
   private final FrameVector[] tempVectorsForInCirclePoint = new FrameVector[] {new FrameVector(), new FrameVector(), new FrameVector(), new FrameVector()};
   private final FramePoint[] tempPointListForInCirclePoint = new FramePoint[4];
   private final FramePoint tempInCircleCenter = new FramePoint();
   
   private final FramePoint tempIntersection = new FramePoint();
   
   private final FrameVector[] tempVectorsForCommonSupportPolygon = new FrameVector[] {new FrameVector(), new FrameVector()};
   private final Point2D[] tempPointsForCornerCircle = new Point2D[] {new Point2D(), new Point2D(), new Point2D(), new Point2D()};
   private final Vector2D tempVectorForCornerCircle = new Vector2D();
   
   private final FrameLineSegment2d tempLineSegment2d = new FrameLineSegment2d();
   private final FramePoint2d tempFramePoint2dOne = new FramePoint2d();
   private final FramePoint2d tempFramePoint2dTwo = new FramePoint2d();
   private final FrameLine2d tempFrameLine2d = new FrameLine2d();

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
         setFootstep(robotQuadrant, polygon.getFootstep(robotQuadrant));
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
   
   public FramePoint reviveFootstep(RobotQuadrant robotQuadrant)
   {      
      return footsteps.add(robotQuadrant);
   }
   
   public void set(QuadrupedSupportPolygon polygon)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         setFootstep(robotQuadrant, polygon.getFootstep(robotQuadrant));
      }
   }

   public void setFootstep(RobotQuadrant robotQuadrant, FramePoint footstep)
   {
      if (footstep == null)
      {
         footsteps.remove(robotQuadrant);
      }
      else
      {
         footsteps.add(robotQuadrant).setIncludingFrame(footstep);
      }
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
   public void translate(Vector3D translateBy)
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
    * Translates the polygon forward a distance or backward when distance is negative.
    */
   public void translateForward(double forwardDistance)
   {
      FramePoint frontMidPoint = tempInCircleCenter;
      FramePoint hindMidPoint = tempIntersection;
      FrameVector forwardVector = tempVectorsForCommonSupportPolygon[0];
      
      getFrontMidpoint(frontMidPoint);
      getHindMidpoint(hindMidPoint);
      
      forwardVector.sub(frontMidPoint, hindMidPoint);
      forwardVector.normalize();
      forwardVector.scale(forwardDistance);
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         getFootstep(robotQuadrant).add(forwardVector);
      }
   }
   
   /**
    * Translates the polygon sideways to the right a distance or to the left when distance is negative.
    */
   public void translateSideways(double rightwaysDistance)
   {
      FramePoint rightMidpoint = tempInCircleCenter;
      FramePoint leftMidpoint = tempIntersection;
      FrameVector rightwaysVector = tempVectorsForCommonSupportPolygon[0];
      
      getRightMidpoint(rightMidpoint);
      getLeftMidpoint(leftMidpoint);
      
      rightwaysVector.sub(rightMidpoint, leftMidpoint);
      rightwaysVector.normalize();
      rightwaysVector.scale(rightwaysDistance);
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         getFootstep(robotQuadrant).add(rightwaysVector);
      }
   }

   /**
    * Rotates the feet about the Centroid, keeping the z heights.
    *
    * @return SupportPolygon
    */
   public void yawAboutCentroid(double yaw)
   {
      getCentroid(temporaryFramePoint);
   
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
      updateTempFrameConvexPolygon();
      yoFrameConvexPolygon2d.setFrameConvexPolygon2d(tempFrameConvexPolygon2d);
   }

   private void updateTempFrameConvexPolygon()
   {
      tempFrameConvexPolygon2d.clear();
      tempFrameConvexPolygon2d.changeFrame(getReferenceFrame());
      for (RobotQuadrant supportingQuadrant : getSupportingQuadrantsInOrder())
      {
         tempFrameConvexPolygon2d.addVertexByProjectionOntoXYPlane(getFootstep(supportingQuadrant));
      }
      tempFrameConvexPolygon2d.update();
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
   
   public void snapPointToClosestEdgeOfPolygonIfOutside2d(YoFramePoint2d pointToSnap)
   {
      FramePoint2d snapped = snapPointToClosestEdgeOfPolygonIfOutside2d(pointToSnap.getX(), pointToSnap.getY());
      pointToSnap.set(snapped.getX(), snapped.getY());
   }
   
   public void snapPointToClosestEdgeOfPolygonIfOutside2d(YoFramePoint pointToSnap)
   {
      FramePoint2d snapped = snapPointToClosestEdgeOfPolygonIfOutside2d(pointToSnap.getX(), pointToSnap.getY());
      pointToSnap.set(snapped.getX(), snapped.getY(), 0.0);
   }
   
   private FramePoint2d snapPointToClosestEdgeOfPolygonIfOutside2d(double x, double y)
   {
      if (size() > 0 && !isInside(x, y))
      {
         updateTempFrameConvexPolygon();

         tempFramePoint2dOne.set(x, y);
         tempFrameConvexPolygon2d.getClosestEdge(tempLineSegment2d, tempFramePoint2dOne);
         tempLineSegment2d.getClosestPointOnLineSegment(tempFramePoint2dTwo, tempFramePoint2dOne);
         
         return tempFramePoint2dTwo;
      }
      else
      {
         tempFramePoint2dTwo.set(x, y);
         return tempFramePoint2dTwo;
      }
   }
   
   public void snapPointToEdgeTowardsInnerPointIfOutside(YoFramePoint pointToSnap, YoFramePoint innerPoint)
   {
      if (size() > 0 && !isInside(pointToSnap.getFrameTuple()))
      {
         updateTempFrameConvexPolygon();

         tempLineSegment2d.set(ReferenceFrame.getWorldFrame(), innerPoint.getX(), innerPoint.getY(), pointToSnap.getX(), pointToSnap.getY());
         FramePoint2d[] intersectionWith = tempFrameConvexPolygon2d.intersectionWith(tempLineSegment2d);
         if (intersectionWith == null || intersectionWith.length < 1)
         {
            tempFrameLine2d.set(ReferenceFrame.getWorldFrame(), innerPoint.getX(), innerPoint.getY(), pointToSnap.getX(), pointToSnap.getY());
            intersectionWith = tempFrameConvexPolygon2d.intersectionWith(tempFrameLine2d);
         }
         if (intersectionWith != null)
         {
            pointToSnap.setX(intersectionWith[0].getX());
            pointToSnap.setY(intersectionWith[0].getY());
         }
      }
   }


   /**
    * if you pick up a leg and want equal weight distribution, pretend that there are two legs on top of eachother
    * @param centroidToPack
    */
   public void getCentroidEqualWeightingEnds(FramePoint centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      int size = size();
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if(getFootstep(robotQuadrant) != null)
         {
            centroidToPack.add(getFootstep(robotQuadrant));
         }
         else
         {
            FramePoint acrossBodyFootstep = getFootstep(robotQuadrant.getAcrossBodyQuadrant());
            if(acrossBodyFootstep != null)
            {
               centroidToPack.add(acrossBodyFootstep);
               size++;
            }
         }
      }
      
      centroidToPack.scale(1.0 / size);
   }
   
   public void getCentroid(FramePoint centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         centroidToPack.add(getFootstep(robotQuadrant));
      }
      
      centroidToPack.scale(1.0 / size());
   }
   
   /**
    * gets the Centroid of the supplied quadrants and sets the z value to the average between the lowest foot in 
    * the front and hind in world frame. If no Hinds or no Fronts are supplied it will use the lowest foot given.
    * If no quadrants are supplied the result will be Zeros in world.
    */
   public void getCentroidAveragingLowestZHeightsAcrossEnds(FramePoint centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      double frontZ = Double.MAX_VALUE;
      double hindZ = Double.MAX_VALUE;
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         FramePoint footstep = getFootstep(robotQuadrant);
         centroidToPack.add(footstep);
         
         double currentFootZ = footstep.getZ();
         if(robotQuadrant.isQuadrantInFront() && currentFootZ < frontZ)
         {
            frontZ = currentFootZ;
         }
         else if(robotQuadrant.isQuadrantInHind() && currentFootZ < hindZ)
         {
            hindZ = currentFootZ;
         }
      }

      centroidToPack.scale(1.0 / size());
      
      double averageZ = 0.0;
      if(hindZ < Double.MAX_VALUE)
      {
         averageZ += hindZ;
      }
      if(frontZ < Double.MAX_VALUE)
      {
         averageZ += frontZ;
      }
      averageZ /= 2.0;
      centroidToPack.setZ(averageZ);
   }
   
   /**
    * gets the weighted Centroid of support polyon and sets the z value to the average between the lowest foot in 
    * the front and hind in world frame. If no Hinds or no Fronts are supplied it will use the lowest foot given.
    * If no quadrants are available the result will be Zeros in world.
    */
   public void getCentroidWithEqualWeightedEndsAveragingLowestZHeightsAcrossEnds(FramePoint centroidToPack)
   {
      centroidToPack.setToZero(ReferenceFrame.getWorldFrame());
      getCentroidEqualWeightingEnds(centroidToPack);
      
      double frontZ = Double.MAX_VALUE;
      double hindZ = Double.MAX_VALUE;
      
      for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
      {
         FramePoint footstep = getFootstep(robotQuadrant);
         double currentFootZ = footstep.getZ();
         if(robotQuadrant.isQuadrantInFront() && currentFootZ < frontZ)
         {
            frontZ = currentFootZ;
         }
         else if(robotQuadrant.isQuadrantInHind() && currentFootZ < hindZ)
         {
            hindZ = currentFootZ;
         }
      }
      
      double averageZ = 0.0;
      if(hindZ < Double.MAX_VALUE)
      {
         averageZ += hindZ;
      }
      if(frontZ < Double.MAX_VALUE)
      {
         averageZ += frontZ;
      }
      averageZ /= 2.0;
      centroidToPack.setZ(averageZ);
   }
   
   private final FramePoint tempFramePointForCentroids = new FramePoint();
   public void getCentroidFramePoseAveragingLowestZHeightsAcrossEnds(FramePose framePose)
   {
      double nominalPitch = getNominalPitch();
      double nominalRoll = getNominalRoll();
      double nominalYaw = getNominalYaw();
      
      getCentroidAveragingLowestZHeightsAcrossEnds(tempFramePointForCentroids);
      framePose.setYawPitchRoll(nominalYaw, nominalPitch, nominalRoll);
      framePose.setPosition(tempFramePointForCentroids);
   }
   
   public void getWeightedCentroidFramePoseAveragingLowestZHeightsAcrossEnds(FramePose framePose)
   {
      double nominalPitch = getNominalPitch();
      double nominalRoll = getNominalRoll();
      double nominalYaw = getNominalYaw();
      
      getCentroidWithEqualWeightedEndsAveragingLowestZHeightsAcrossEnds(tempFramePointForCentroids);
      framePose.setYawPitchRoll(nominalYaw, nominalPitch, nominalRoll);
      framePose.setPosition(tempFramePointForCentroids);
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
   public void getBounds(Point2D minToPack, Point2D maxToPack)
   {
      minToPack.setX(Double.POSITIVE_INFINITY);
      minToPack.setY(Double.POSITIVE_INFINITY);
      maxToPack.setX(Double.NEGATIVE_INFINITY);
      maxToPack.setY(Double.NEGATIVE_INFINITY);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footstep = getFootstep(robotQuadrant);
         if (containsFootstep(footstep))
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
    * ignores Z values for the polygon. The test works by computing the minimum
    * distance from each polygon line segment to the point.
    *
    * @param point Point2d
    * @return boolean
    */
   public boolean isInside(FramePoint point)
   {
      return isInside(point.getX(), point.getY());
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
      return isInside(point.getX(), point.getY());
   }
   
   private boolean isInside(double x, double y)
   {
      if (getDistanceInside2d(x, y) > 0.0)
      {
         return true;
      }

      return false;
   }
   
   public double getDistanceInside2d(FramePoint2d point)
   {
      return getDistanceInside2d(point.getX(), point.getY());
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
   public double getDistanceInside2d(FramePoint point)
   {
      return getDistanceInside2d(point.getX(), point.getY());
   }
   
   private double getDistanceInside2d(double x, double y)
   {
      if (size() == 1)
      {
         FramePoint footstep = getFootstep(getFirstSupportingQuadrant());
         return -EuclidGeometryTools.distanceBetweenPoint2Ds(x, y, footstep.getX(), footstep.getY());
      }
      else if (size() == 2)
      {
         FramePoint pointOne = getFootstep(getFirstSupportingQuadrant());
         FramePoint pointTwo = getFootstep(getLastSupportingQuadrant());
         return -Math.abs(EuclidGeometryTools.distanceFromPoint2DToLine2D(x, y, pointOne.getX(), pointOne.getY(), pointTwo.getX() - pointOne.getX(), pointTwo.getY() - pointOne.getY()));
      }
      else
      {
         double closestDistance = Double.POSITIVE_INFINITY;
         
         for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
         {
            FramePoint pointOne = getFootstep(robotQuadrant);
            FramePoint pointTwo = getFootstep(getNextClockwiseSupportingQuadrant(robotQuadrant));
            
            double distance = getDistanceToSideOfSegment(x, y, pointOne, pointTwo);
            if (distance < closestDistance)
            {
               closestDistance = distance;
            }
         }
         
         return closestDistance;
      }
   }

   /**
    * Returns the distance the given (x,y) value is outside the polygon.  This is defined
    * as the minimum distance from the point to each line segment on the polygon.
    * If the point is inside the polygon, the distance will be the negative of the minimum distance
    * from the point to each line segment on the polygon (i.e. -getDistanceInside2d). This test ignores Z values for the polygon.
    *
    * @param point Point2d
    * @return boolean
    */
   public double getDistanceOutside2d(FramePoint2d point)
   {
      return getDistanceOutside2d(point.getX(), point.getY());
   }

   /**
    * @see #getDistanceInside2d(FramePoint2d)
    */
   public double getDistanceOutside2d(FramePoint point)
   {
      return getDistanceOutside2d(point.getX(), point.getY());
   }

   /**
    * @see #getDistanceInside2d(FramePoint2d)
    */
   private double getDistanceOutside2d(double x, double y)
   {
      if (size() == 1)
      {
         FramePoint footstep = getFootstep(getFirstSupportingQuadrant());
         return EuclidGeometryTools.distanceBetweenPoint2Ds(x, y, footstep.getX(), footstep.getY());
      }
      else if (size() == 2)
      {
         FramePoint pointOne = getFootstep(getFirstSupportingQuadrant());
         FramePoint pointTwo = getFootstep(getLastSupportingQuadrant());
         return Math.abs(getDistanceToSideOfSegment(x, y, pointOne, pointTwo));
      }
      else
      {
         double closestDistanceOutside = Double.POSITIVE_INFINITY;
         double closestDistanceInside = getDistanceInside2d(x, y);
         if (closestDistanceInside >= 0)
         {
            return -closestDistanceInside;
         }
         for (RobotQuadrant robotQuadrant : getSupportingQuadrantsInOrder())
         {
            FramePoint pointOne = getFootstep(robotQuadrant);
            FramePoint pointTwo = getFootstep(getNextClockwiseSupportingQuadrant(robotQuadrant));
            double distance = Math.abs(getDistanceToSideOfSegment(x, y, pointOne, pointTwo));
            if (distance < closestDistanceOutside)
            {
               closestDistanceOutside = distance;
            }
         }
         return closestDistanceOutside;
      }
   }

   private double getDistanceToSideOfSegment(double x, double y, FramePoint pointOne, FramePoint pointTwo)
   {
      double x1 = pointOne.getX();
      double y1 = pointOne.getY();
      
      double x2 = pointTwo.getX();
      double y2 = pointTwo.getY();
      
      double numerator = (y2 - y1) * x - (x2 - x1) * y + x2*y1 - y2*x1;
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
      GeometryTools.getIntersectionBetweenTwoLines2d(tempPointListForInCirclePoint[1], v2p, tempPointListForInCirclePoint[2], v3p, intersectionToPack);
   }

   /**
    * Get the distance of the point inside of the in circle.
    *
    * @param point
    * @return distance
    */
   public double getDistanceInsideInCircle2d(FramePoint point)
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
      getMidpoint(RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, framePointToPack);
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
      getMidpoint(RobotQuadrant.HIND_LEFT, RobotQuadrant.HIND_RIGHT, framePointToPack);
   }
   
   /**
    * Returns the average position of the vertices in the polygon
    * corresponding to left foot locations. NaN is hind feet aren't
    * supporting.
    *
    * @params framePointToPack
    */
   public void getLeftMidpoint(FramePoint framePointToPack)
   {
      getMidpoint(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, framePointToPack);
   }
   
   /**
    * Returns the average position of the vertices in the polygon
    * corresponding to right foot locations. NaN is hind feet aren't
    * supporting.
    *
    * @params framePointToPack
    */
   public void getRightMidpoint(FramePoint framePointToPack)
   {
      getMidpoint(RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_RIGHT, framePointToPack);
   }
   
   private void getMidpoint(RobotQuadrant quadrant1, RobotQuadrant quadrant2, FramePoint framePointToPack)
   {
      if (containsFootstep(quadrant1) && containsFootstep(quadrant2))
      {
         framePointToPack.setToZero();
         framePointToPack.add(getFootstep(quadrant1));
         framePointToPack.add(getFootstep(quadrant2));
         framePointToPack.scale(0.5);
      }
      else if (containsFootstep(quadrant1))
      {
         framePointToPack.set(getFootstep(quadrant1));
      }
      else if (containsFootstep(quadrant2))
      {
         framePointToPack.set(getFootstep(quadrant2));
      }
      else
      {
         throw new UndefinedOperationException("Polygon must contain a footstep at " + quadrant1 + " or " + quadrant2);
      }
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
    *  getStanceWidthFrontLegs
    * 
    *  @return double
    */
   public double getStanceLength(RobotSide robotSide)
   {
      if (containsFootstep(RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide))
       && containsFootstep(RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide)))
      {
         FramePoint frontFootstep = getFootstep(RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide));
         FramePoint hindFootstep = getFootstep(RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide));

         return frontFootstep.distance(hindFootstep);
      }
      else
      {
         throw new UndefinedOperationException("Polygon must contain both legs on this side: " + robotSide);
      }
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
      
      FrameVector direction1 = tempVectorsForCommonSupportPolygon[0];
      direction1.sub(getFootstep(compareSwingLeg.getDiagonalOppositeQuadrant()), getFootstep(compareSwingLeg));
      
      FrameVector direction2 = tempVectorsForCommonSupportPolygon[1];
      direction2.sub(polygonToCompare.getFootstep(thisSwingLeg.getDiagonalOppositeQuadrant()), polygonToCompare.getFootstep(thisSwingLeg));
      
      commonPolygonToPack.clear();
      FramePoint intersection = commonPolygonToPack.reviveFootstep(quadrantToAssignToIntersection);
      GeometryTools.getIntersectionBetweenTwoLines2d(getFootstep(compareSwingLeg), direction1, polygonToCompare.getFootstep(thisSwingLeg), direction2, intersection);
      
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
         QuadrupedSupportPolygon tempCommonSupportPolygon, RobotQuadrant quadrantForIntersection, double frontDistance, double sideDistance,
         double hindDistance)
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
         
         GeometryTools.getIntersectionBetweenTwoLines2d(shrunkenNextEdgeFoot, shrunkenShrinkEdgeFoot, originalPreviousEdgeFoot, originalShrinkEdgeFoot, shrunkenShrinkEdgeFoot);
         GeometryTools.getIntersectionBetweenTwoLines2d(shrunkenShrinkEdgeFoot, shrunkenNextEdgeFoot, originalPreviousEdgeFoot, originalNextEdgeFoot, shrunkenNextEdgeFoot);
      }
      else if (size() == 2)
      {
         RobotQuadrant nextEdgeQuadrant = getNextClockwiseSupportingQuadrant(sideToShrink);
         RobotQuadrant shrinkQuadrant = getNextClockwiseSupportingQuadrant(nextEdgeQuadrant);
         
         FramePoint shrinkFootstep = getFootstep(shrinkQuadrant);
         FramePoint shrinkTowardsFootstep = getFootstep(nextEdgeQuadrant);
         
         FrameVector shrinkVector = tempVectorsForInCirclePoint[0];
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
   
   /**
    * Gets distance from P1 to trotLine specified by front quadrant.
    */
   public double getDistanceFromP1ToTrotLineInDirection2d(RobotQuadrant trotQuadrant, FramePoint p1, FramePoint p2)
   {
      boolean intersectionExists = GeometryTools.getIntersectionBetweenTwoLines2d(p1, p2, getFootstep(trotQuadrant), getFootstep(trotQuadrant.getDiagonalOppositeQuadrant()), tempIntersection);

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
    * This only works on triangles and must have size of 3.
    * 
    * @param cornerToPutCircle
    * @param cornerCircleRadius
    * @param centerToPack
    * @return false if the center passed the centroid
    */
   public boolean getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant cornerToPutCircle, double cornerCircleRadius, FramePoint2d centerToPack)
   {
      if (size() == 3)
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
         throw new UndefinedOperationException("Triangle size must be 3. size = " + size());
      }
   }
   
   /**
    * Gets the center point of a circle of radius in the corner of a polygon.
    * 
    * This polygon must have size of 3 or 4.
    * 
    * @param cornerToPutCircle
    * @param radius
    * @param centerToPack
    */
   public void getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant cornerToPutCircle, double cornerCircleRadius, FramePoint2d centerToPack)
   {
      if (containsFootstep(cornerToPutCircle))
      {
         if (size() >= 3)
         {
            // Corner and A and B form a V with corner as the vertex
            FramePoint cornerPoint = getFootstep(cornerToPutCircle);
            FramePoint pointA = getFootstep(getNextClockwiseSupportingQuadrant(cornerToPutCircle));
            FramePoint pointB = getFootstep(getNextCounterClockwiseSupportingQuadrant(cornerToPutCircle));
   
            double cornerToA = cornerPoint.distance(pointA);
            double cornerToB = cornerPoint.distance(pointB);
            double aToB = pointA.distance(pointB);
   
            double theta = EuclidGeometryTools.unknownTriangleAngleByLawOfCosine(cornerToA, cornerToB, aToB);
            
            Point2D tempCorner = tempPointsForCornerCircle[0];
            Point2D tempA = tempPointsForCornerCircle[1];
            Point2D tempB = tempPointsForCornerCircle[2];
            
            cornerPoint.getPoint2d(tempCorner);
            pointA.getPoint2d(tempA);
            pointB.getPoint2d(tempB);
   
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
            throw new UndefinedOperationException("Polygon size must be 3 or 4. size = " + size());
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
