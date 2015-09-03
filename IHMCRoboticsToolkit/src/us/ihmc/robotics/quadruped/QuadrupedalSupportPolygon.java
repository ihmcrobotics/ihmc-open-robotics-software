package us.ihmc.robotics.quadruped;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

@SuppressWarnings("all")
public class QuadrupedalSupportPolygon extends QuadrupedalPolygon
{
   public QuadrupedalSupportPolygon(QuadrupedalPolygon polygon)
   {
      super(polygon);
   }

   public QuadrupedalSupportPolygon(QuadrupedFootstep[] supportPolygonFootsteps)
   {
      super(supportPolygonFootsteps);
   }

   public QuadrupedalSupportPolygon(ArrayList<QuadrupedFootstep> supportPolygonFootsteps)
   {
      super(supportPolygonFootsteps);
   }

   public QuadrupedalSupportPolygon(QuadrupedFootstep footstep0, QuadrupedFootstep footstep1, QuadrupedFootstep footstep2, QuadrupedFootstep footstep3)
   {
      super(footstep0, footstep1, footstep2, footstep3);
   }


   public QuadrupedalSupportPolygon deleteLegCopy(RobotQuadrant legName)
   {
      QuadrupedFootstep[] footstepArray = getFootstepArrayCopy();
      footstepArray[legName.ordinal()] = null;

      return new QuadrupedalSupportPolygon(footstepArray);
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
   public QuadrupedalSupportPolygon getCommonSupportPolygon(QuadrupedalSupportPolygon nextSupportPolygon, RobotQuadrant specifiedLegName)
   {
      // verify both have exactly three legs
      if (this.getNumberOfLegs() != 3)
         throw new IllegalArgumentException("This supportPolygon must contain exactly three legs not " + this.getNumberOfLegs());
      if (nextSupportPolygon.getNumberOfLegs() != 3)
         throw new IllegalArgumentException("Next supportPolygon must contain exactly three legs not " + nextSupportPolygon.getNumberOfLegs());

      // return null if swing legs are not same side *** Assumes regular gait ***
      RobotQuadrant currentSwingLeg = RobotQuadrant.getQuadrantNameFromOrdinal(this.getFirstNullFootstep());
      RobotQuadrant nextSwingLeg = currentSwingLeg.getNextRegularGaitSwingQuadrant();
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
      QuadrupedFootstep intersectionFootstep = new CartesianQuadrupedFootstep(position, specifiedLegName);

      // add now intersection and two matching legs
      ArrayList<QuadrupedFootstep> commonFootsteps = new ArrayList<QuadrupedFootstep>();
      commonFootsteps.add(intersectionFootstep);
      commonFootsteps.add(this.getFootstep(currentSwingLeg.getAcrossBodyQuadrant()));
      commonFootsteps.add(this.getFootstep(currentSwingLeg.getDiagonalOppositeQuadrant()));

      return new QuadrupedalSupportPolygon(commonFootsteps);
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
   public QuadrupedalSupportPolygon getShrunkenCommonSupportPolygon(QuadrupedalSupportPolygon nextSupportPolygon, RobotQuadrant specifiedLegName, double frontDistance,
           double sideDistance, double hindDistance)
   {
      // get common polygon
      QuadrupedalSupportPolygon commonSupportPolygon = getCommonSupportPolygon(nextSupportPolygon, specifiedLegName);

      // if it is null return null
      if (commonSupportPolygon == null)
         return null;

      // find legs invovled
      RobotQuadrant swingLeg = RobotQuadrant.getQuadrantNameFromOrdinal(commonSupportPolygon.getFirstNullFootstep());
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
      QuadrupedFootstep swingLegSameSideFootstep = commonSupportPolygon.getFootstep(swingLeg.getSameSideQuadrant());
      QuadrupedFootstep oppositeFrontFootstep = commonSupportPolygon.getFootstep(oppositeFront);
      QuadrupedFootstep oppositeHindFootstep = commonSupportPolygon.getFootstep(oppositeHind);
      Point2d point1 = QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy());
      Point2d point2 = QuadrupedalSupportPolygon.projectXY(oppositeFrontFootstep.getPositionFramePointCopy());
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

      point2 = QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy());
      point1 = QuadrupedalSupportPolygon.projectXY(oppositeHindFootstep.getPositionFramePointCopy());
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

      point1 = QuadrupedalSupportPolygon.projectXY(oppositeFrontFootstep.getPositionFramePointCopy());
      point2 = QuadrupedalSupportPolygon.projectXY(oppositeHindFootstep.getPositionFramePointCopy());
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
      point1 = QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy());
      point2 = QuadrupedalSupportPolygon.projectXY(oppositeFrontFootstep.getPositionFramePointCopy());
      point1.add(directionOfShrinkage);
      point2.add(directionOfShrinkage);

      // find intersection of new side with each existing side
      Vector2d point1ToPoint2 = new Vector2d(point1);
      point1ToPoint2.sub(point2);
      Point2d newSwingLegSameSidePoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
                                            QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy()), hindVector);
      Point2d oppositeFrontPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
                                      QuadrupedalSupportPolygon.projectXY(oppositeFrontFootstep.getPositionFramePointCopy()), sideVector);

      // update foot positions
      FramePoint position = new FramePoint(commonSupportPolygon.getReferenceFrame(), newSwingLegSameSidePoint.x, newSwingLegSameSidePoint.y, 0.0);
      swingLegSameSideFootstep = new CartesianQuadrupedFootstep(position, swingLeg.getSameSideQuadrant());
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeFrontPoint.x, oppositeFrontPoint.y, 0.0);
      oppositeFrontFootstep = new CartesianQuadrupedFootstep(position, oppositeFront);

      // now shrink the hind side
      // find normal perpendicular to side
      directionOfShrinkage = GeometryTools.getPerpendicularVector(hindVector);
      directionOfShrinkage.normalize();

      // scale by desired distance
      directionOfShrinkage.scale(hindDistance);

      // apply to each foot
      point1 = QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy());
      point2 = QuadrupedalSupportPolygon.projectXY(oppositeHindFootstep.getPositionFramePointCopy());
      point1.add(directionOfShrinkage);
      point2.add(directionOfShrinkage);

      // find intersection of new side with each existing side
      point1ToPoint2 = new Vector2d(point1);
      point1ToPoint2.sub(point2);
      newSwingLegSameSidePoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
              QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy()), frontVector);
      Point2d oppositeHindPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
                                     QuadrupedalSupportPolygon.projectXY(oppositeFrontFootstep.getPositionFramePointCopy()), sideVector);

      // update foot positions
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), newSwingLegSameSidePoint.x, newSwingLegSameSidePoint.y, 0.0);
      swingLegSameSideFootstep = new CartesianQuadrupedFootstep(position, swingLeg.getSameSideQuadrant());
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeHindPoint.x, oppositeHindPoint.y, 0.0);
      oppositeHindFootstep = new CartesianQuadrupedFootstep(position, oppositeHind);

      // now shrink the remaining side
      // find normal perpendicular to side
      directionOfShrinkage = GeometryTools.getPerpendicularVector(sideVector);
      directionOfShrinkage.normalize();

      // scale by desired distance
      directionOfShrinkage.scale(sideDistance);

      // apply to each foot
      point1 = QuadrupedalSupportPolygon.projectXY(oppositeFrontFootstep.getPositionFramePointCopy());
      point2 = QuadrupedalSupportPolygon.projectXY(oppositeHindFootstep.getPositionFramePointCopy());
      point1.add(directionOfShrinkage);
      point2.add(directionOfShrinkage);

      // find intersection of new side with each existing side
      point1ToPoint2 = new Vector2d(point1);
      point1ToPoint2.sub(point2);
      oppositeFrontPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
              QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy()), frontVector);
      oppositeHindPoint = GeometryTools.getIntersectionBetweenTwoLines(point1, point1ToPoint2,
              QuadrupedalSupportPolygon.projectXY(swingLegSameSideFootstep.getPositionFramePointCopy()), hindVector);

      // update foot positions
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeFrontPoint.x, oppositeFrontPoint.y, 0.0);
      oppositeFrontFootstep = new CartesianQuadrupedFootstep(position, oppositeFront);
      position = new FramePoint(commonSupportPolygon.getReferenceFrame(), oppositeHindPoint.x, oppositeHindPoint.y, 0.0);
      oppositeHindFootstep = new CartesianQuadrupedFootstep(position, oppositeHind);

      // now add new footsteps to shrunken polygon
      ArrayList<QuadrupedFootstep> commonFootsteps = new ArrayList<QuadrupedFootstep>();
      commonFootsteps.add(swingLegSameSideFootstep);
      commonFootsteps.add(oppositeFrontFootstep);
      commonFootsteps.add(oppositeHindFootstep);

      return new QuadrupedalSupportPolygon(commonFootsteps);
   }




   /**
    * If this and the given supportPolygon differ only by one footstep, this returns the leg that differs.
    * Else it returns null
    * @param supportPolygonEnd SupportPolygon
    * @return LegName
    */
   public RobotQuadrant getSwingLegFromHereToNextPolygon(QuadrupedalSupportPolygon nextSupportPolygon)
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
         if (!getFootstep(legName).isEpsilonEqualTo(nextSupportPolygon.getFootstep(legName), 1e-5))
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

      int swingLegIndex = this.getFirstNullFootstep();
      RobotQuadrant swingLeg = RobotQuadrant.getQuadrantNameFromOrdinal(swingLegIndex);

      QuadrupedFootstep footstep1 = footstepArray[swingLeg.getAcrossBodyQuadrant().ordinal()];
      QuadrupedFootstep footstep2 = footstepArray[swingLeg.getSameSideQuadrant().ordinal()];

      FramePoint pointStart = footstep1.getPositionFramePointCopy();
      FramePoint pointEnd = footstep2.getPositionFramePointCopy();

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
      int swingLegIndex = this.getFirstNullFootstep();
      RobotQuadrant swingLeg = RobotQuadrant.getQuadrantNameFromOrdinal(swingLegIndex);

      QuadrupedFootstep footstep1 = footstepArray[swingLeg.getAcrossBodyQuadrant().ordinal()];
      QuadrupedFootstep footstep2 = footstepArray[swingLeg.getSameSideQuadrant().ordinal()];

      FramePoint ret = footstep1.getPositionFramePointCopy();
      ret.add(footstep2.getPositionFramePointCopy());

      ret.scale(0.5);

      return ret;
   }

   public FrameVector get2DVectorPerpendicularToTrotLinePointingInside()
   {
      int swingLegIndex = this.getFirstNullFootstep();
      RobotQuadrant swingLeg = RobotQuadrant.getQuadrantNameFromOrdinal(swingLegIndex);

      QuadrupedFootstep footstep1 = footstepArray[swingLeg.getAcrossBodyQuadrant().ordinal()];
      QuadrupedFootstep footstep2 = footstepArray[swingLeg.getSameSideQuadrant().ordinal()];

      FrameVector diffVector = new FrameVector(footstep2.getPositionFramePointCopy());
      diffVector.sub(footstep1.getPositionFramePointCopy());

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
         xFL = footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()].getX();
         yFL = footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()].getY();

         xFR = footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()].getX();
         yFR = footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()].getY();

         xHR = footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()].getX();
         yHR = footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()].getY();

         xHL = footstepArray[RobotQuadrant.HIND_LEFT.ordinal()].getX();
         yHL = footstepArray[RobotQuadrant.HIND_LEFT.ordinal()].getY();
      }
      else
      {
         xFL = (footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()].getX();
         yFL = (footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()].getY();

         xFR = (footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()].getX();
         yFR = (footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()].getY();

         xHR = (footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()].getX();
         yHR = (footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()].getY();

         xHL = (footstepArray[RobotQuadrant.HIND_LEFT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.HIND_LEFT.ordinal()].getX();
         yHL = (footstepArray[RobotQuadrant.HIND_LEFT.ordinal()] == null) ? Double.NaN : footstepArray[RobotQuadrant.HIND_LEFT.ordinal()].getY();
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
   public QuadrupedalSupportPolygon replaceFootstepCopy(QuadrupedFootstep footstep)
   {
      QuadrupedFootstep[] newFootsteps = new QuadrupedFootstep[4];

      for (int i = 0; i < 4; i++)
      {
         if (i == footstep.getLegName().ordinal())
         {
            // check to make sure that the stored footstep is not null
            if (this.footstepArray[i] == null)
               throw new RuntimeException("You cannot replaceFootstepCopy for a stored null footstep: " + footstep);
            else
               newFootsteps[i] = footstep;
         }
         else
         {
            newFootsteps[i] = this.footstepArray[i];
         }
      }

      return new QuadrupedalSupportPolygon(newFootsteps);
   }


   public QuadrupedalSupportPolygon changeFrameCopy(ReferenceFrame referenceFrame)
   {
      ArrayList<QuadrupedFootstep> footsteps = this.getFootsteps();
      ArrayList<QuadrupedFootstep> newFootsteps = new ArrayList<QuadrupedFootstep>();

      for (QuadrupedFootstep footstep : footsteps)
      {
         newFootsteps.add(footstep.changeFrameCopy(referenceFrame));
      }

      return new QuadrupedalSupportPolygon(newFootsteps);
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

      QuadrupedFootstep frontLeftFootstep = footstepArray[RobotQuadrant.FRONT_LEFT.ordinal()];
      QuadrupedFootstep frontRightFootstep = footstepArray[RobotQuadrant.FRONT_RIGHT.ordinal()];
      QuadrupedFootstep hindRightFootstep = footstepArray[RobotQuadrant.HIND_RIGHT.ordinal()];
      QuadrupedFootstep hindLeftFootstep = footstepArray[RobotQuadrant.HIND_LEFT.ordinal()];

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
      if (this.getFootsteps().size() != 2)
         return false;

      if (this.getFootsteps().get(0).getLegName().getDiagonalOppositeQuadrant() != this.getFootsteps().get(1).getLegName())
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

      ArrayList<QuadrupedFootstep> footsteps = this.getFootsteps();

      for (QuadrupedFootstep footstep : footsteps)
      {
         if (footstep.getLegName().isQuadrantOnLeftSide())
            return footstep.getLegName();
      }

      throw new RuntimeException("There should have been a left side leg in the supportPolygon");
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

      ArrayList<QuadrupedFootstep> footsteps = this.getFootsteps();

      for (QuadrupedFootstep footstep : footsteps)
      {
         if (footstep.getLegName().isQuadrantOnRightSide())
            return footstep.getLegName();
      }

      throw new RuntimeException("There should have been a right side leg in the supportPolygon");
   }


   public FramePoint getAverageFrontPoint()
   {
      QuadrupedFootstep frontLeft = this.getFootstep(RobotQuadrant.FRONT_LEFT);
      QuadrupedFootstep frontRight = this.getFootstep(RobotQuadrant.FRONT_RIGHT);

      if ((frontLeft != null) && (frontRight != null))
      {
         FramePoint ret = frontLeft.getPositionFramePointCopy();
         ret.add(frontRight.getPositionFramePointCopy());
         ret.scale(0.5);

         return ret;
      }

      else if (frontLeft != null)
         return frontLeft.getPositionFramePointCopy();
      else if (frontRight != null)
         return frontRight.getPositionFramePointCopy();

      else
         throw new RuntimeException("SupportPolygon.getAverageFrontPoint(): Both front legs are null!");
   }


   public FramePoint getAverageHindPoint()
   {
      QuadrupedFootstep hindLeft = this.getFootstep(RobotQuadrant.HIND_LEFT);
      QuadrupedFootstep hindRight = this.getFootstep(RobotQuadrant.HIND_RIGHT);

      if ((hindLeft != null) && (hindRight != null))
      {
         FramePoint ret = hindLeft.getPositionFramePointCopy();
         ret.add(hindRight.getPositionFramePointCopy());
         ret.scale(0.5);

         return ret;
      }

      else if (hindLeft != null)
         return hindLeft.getPositionFramePointCopy();
      else if (hindRight != null)
         return hindRight.getPositionFramePointCopy();

      else
         throw new RuntimeException("SupportPolygon.getAverageFrontPoint(): Both hind legs are null!");

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

   public void printOutSupportPolygon(String nameSuffix)
   {
      QuadrupedalSupportPolygon supportPolygon = this.changeFrameCopy(ReferenceFrame.getWorldFrame());

      System.out.println("\n      Point3d[] feetPositions" + nameSuffix + " = new Point3d[]{");

      for (RobotQuadrant legName : RobotQuadrant.values())
      {
         QuadrupedFootstep footstep = supportPolygon.getFootstep(legName);
         if (footstep != null)
         {
            Point3d point3d = supportPolygon.getFootstep(legName).getPositionFramePointCopy().getPointCopy();
            System.out.print("          new Point3d(" + point3d.x + ", " + point3d.y + ", " + point3d.z + ")");

            if (footstep.getLegName().ordinal() != 3)
            {
               System.out.println(",");
            }
         }
      }

      System.out.println("\n      };");

      System.out.print("      SupportPolygon supportPolygon" + nameSuffix + " = new SupportPolygon(LittleDogFrames.getWorldFrame(), feetPositions" + nameSuffix
                       + ");");

   }
}
