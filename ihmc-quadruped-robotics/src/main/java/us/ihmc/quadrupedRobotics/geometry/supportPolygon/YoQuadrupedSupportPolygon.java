package us.ihmc.quadrupedRobotics.geometry.supportPolygon;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class YoQuadrupedSupportPolygon
{
   private final String namePrefix;
   
   private final QuadrantDependentList<YoBoolean> containsStorage = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> yoFootsteps = new QuadrantDependentList<>();
   
   private final QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
   
   public YoQuadrupedSupportPolygon(String namePrefix, YoVariableRegistry yoVariableRegistry)
   {
      this.namePrefix = namePrefix;
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         containsStorage.set(robotQuadrant, new YoBoolean(namePrefix + "Contains" + robotQuadrant.getPascalCaseName(), yoVariableRegistry));
         yoFootsteps.set(robotQuadrant, new YoFramePoint(namePrefix + "Footstep" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), yoVariableRegistry));
      }
   }
   
   public void clear()
   {
      quadrupedSupportPolygon.clear();
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }

   public boolean areLegsCrossing()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.areLegsCrossing();
   }
   
   public boolean containsFootstep(RobotQuadrant robotQuadrant)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.containsFootstep(robotQuadrant);
   }
   
   public boolean containsSameQuadrants(QuadrupedSupportPolygon polygonToCompare)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.containsSameQuadrants(polygonToCompare);
   }
   
   public void getBounds(Point2D minToPack, Point2D maxToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getBounds(minToPack, maxToPack);
   }
   
   public RobotQuadrant getClosestFootstep(FramePoint3D pointToCompare)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getClosestFootstep(pointToCompare);
   }
   
   public void getCommonTriangle2d(QuadrupedSupportPolygon polygonToCompare, QuadrupedSupportPolygon commonPolygonToPack, RobotQuadrant quadrantToAssignToIntersection)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getCommonTriangle2d(polygonToCompare, commonPolygonToPack, quadrantToAssignToIntersection);
   }
   
   public double getDistanceInside2d(FramePoint2DReadOnly point)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getDistanceInside2d(point);
   }
   
   public double getDistanceInside2d(FramePoint3D point)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getDistanceInside2d(point);
   }
   
   public double getDistanceInsideInCircle2d(FramePoint3D point)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getDistanceInsideInCircle2d(point);
   }
   
   public RobotQuadrant getFirstNonSupportingQuadrant()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getFirstNonSupportingQuadrant();
   }
   
   public RobotQuadrant getFirstSupportingQuadrant()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getFirstSupportingQuadrant();
   }
   
   public FramePoint3D getFootstep(RobotQuadrant robotQuadrant)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getFootstep(robotQuadrant);
   }
   
   public void getFrontMidpoint(FramePoint3D framePointToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getFrontMidpoint(framePointToPack);
   }
   
   public void getLeftMidpoint(FramePoint3D framePointToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getLeftMidpoint(framePointToPack);
   }
   
   public void getRightMidpoint(FramePoint3D framePointToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getRightMidpoint(framePointToPack);
   }
   
   public void getHindMidpoint(FramePoint3D framePointToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getHindMidpoint(framePointToPack);
   }

   public RobotQuadrant getHighestFootstep()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getHighestFootstep();
   }
   
   public double getInCircle2d(FramePoint3D inCircleCenterToPack)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getInCircle2d(inCircleCenterToPack);
   }
   
   public double getInCircleRadius2d()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getInCircleRadius2d();
   }
   
   public void getInCirclePoint2d(FramePoint3D intersectionToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getInCirclePoint2d(intersectionToPack);
   }
   
   public RobotQuadrant getLastNonSupportingQuadrant()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getLastNonSupportingQuadrant();
   }
   
   public RobotQuadrant getLastSupportingQuadrant()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getLastSupportingQuadrant();
   }
   
   public RobotQuadrant getLeftTrotLeg()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getLeftTrotLeg();
   }
   
   public RobotQuadrant[][] getLegPairs()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getLegPairs();
   }
   
   public RobotQuadrant getLowestFootstep()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getLowestFootstep();
   }
   
   public double getLowestFootstepZHeight()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getLowestFootstepZHeight();
   }
   
   public double getNominalPitch()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getNominalPitch();
   }
   
   public double getNominalRoll()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getNominalRoll();
   }
   
   public double getNominalYaw()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getNominalYaw();
   }
   
   public double getNominalYawHindLegs()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getNominalYawHindLegs();
   }
   
   public int getNumberOfEqualFootsteps(QuadrupedSupportPolygon polygonToCompare)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getNumberOfEqualFootsteps(polygonToCompare);
   }
   
   public ReferenceFrame getReferenceFrame()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getReferenceFrame();
   }
   
   public RobotQuadrant getRightTrotLeg()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getRightTrotLeg();
   }
   
   public void getShrunkenCommonTriangle2d(QuadrupedSupportPolygon nextSupportPolygon, QuadrupedSupportPolygon shrunkenCommonPolygonToPack,
         QuadrupedSupportPolygon tempCommonSupportPolygon, RobotQuadrant quadrantForIntersection, double frontDistance, double sideDistance,
         double hindDistance)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getShrunkenCommonTriangle2d(nextSupportPolygon, shrunkenCommonPolygonToPack, tempCommonSupportPolygon, quadrantForIntersection, frontDistance,
                                        sideDistance, hindDistance);
   }
   
   public void getShrunkenPolygon2d(QuadrupedSupportPolygon shrunkenPolygonToPack, RobotQuadrant sideToShrink, double distance)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getShrunkenPolygon2d(shrunkenPolygonToPack, sideToShrink, distance);
   }
   
   public double getStanceLength(RobotSide robotSide)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getStanceLength(robotSide);
   }
   
   public RobotQuadrant[] getSupportingQuadrantsInOrder()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getSupportingQuadrantsInOrder();
   }
   
   public RobotQuadrant getWhichFootstepHasMoved(QuadrupedSupportPolygon nextPolygon)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getWhichFootstepHasMoved(nextPolygon);
   }
   
   public boolean isInside(FramePoint3D point)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.isInside(point);
   }
   
   public boolean isInside(FramePoint2D point)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.isInside(point);
   }
   
   public boolean isValidTrotPolygon()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.isValidTrotPolygon();
   }
   
   public void packYoFrameConvexPolygon2d(YoFrameConvexPolygon2d yoFrameConvexPolygon2d)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.packYoFrameConvexPolygon2d(yoFrameConvexPolygon2d);
   }
   
   public void removeFootstep(RobotQuadrant robotQuadrant)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.removeFootstep(robotQuadrant);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant cornerToPutCircle, double cornerCircleRadius, FramePoint2D centerToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getCenterOfCircleOfRadiusInCornerOfPolygon(cornerToPutCircle, cornerCircleRadius, centerToPack);
   }
   
   public boolean getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant cornerToPutCircle, double cornerCircleRadius,
         FramePoint2D centerToPack)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(cornerToPutCircle, cornerCircleRadius, centerToPack);
   }
   
   public RobotQuadrant getNextClockwiseSupportingQuadrant(RobotQuadrant robotQuadrant)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getNextClockwiseSupportingQuadrant(robotQuadrant);
   }
   
   public RobotQuadrant getNextCounterClockwiseSupportingQuadrant(RobotQuadrant robotQuadrant)
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.getNextCounterClockwiseSupportingQuadrant(robotQuadrant);
   }
   
   public void printOutPolygon(String string)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.printOutPolygon(string);
   }
   
   public void reviveFootstep(RobotQuadrant robotQuadrant)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.reviveFootstep(robotQuadrant);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void setFootstep(RobotQuadrant robotQuadrant, FramePoint3DReadOnly footstep)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.setFootstep(robotQuadrant, footstep);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void shrinkPolygon2d(double distance)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.shrinkPolygon2d(distance);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void shrinkPolygon2d(RobotQuadrant robotQuadrant, double distance)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.shrinkPolygon2d(robotQuadrant, distance);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }

   public int size()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon.size();
   }
   
   public void translateSideways(double rightwaysDistance)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.translateSideways(rightwaysDistance);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void translateForward(double forwardDistance)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.translateForward(forwardDistance);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void translate(double x, double y, double z)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.translate(x, y, z);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void translate(Vector3D translateBy)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.translate(translateBy);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void yawAboutCentroid(double yaw)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.yawAboutCentroid(yaw);
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void getCentroid2d(FramePoint2D centroidToPack2d)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getCentroid2d(centroidToPack2d);
   }
   
   public void getCentroid2d(YoFramePoint2d centroidToPack)
   {
      getCentroid(tempCentroid);
      centroidToPack.set(tempCentroid);
   }

   public void getCentroid(FramePoint3D centroidToPack)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.getCentroid(centroidToPack);
   }

   private final FramePoint3D tempCentroid = new FramePoint3D();

   public void getCentroid(YoFramePoint centroidToPack)
   {
      getCentroid(tempCentroid);
      centroidToPack.set(tempCentroid);
   }

   public void snapPointToClosestEdgeOfPolygonIfOutside2d(YoFramePoint2d pointToSnap)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.snapPointToClosestEdgeOfPolygonIfOutside2d(pointToSnap);
   }

   public void snapPointToClosestEdgeOfPolygonIfOutside2d(YoFramePoint pointToSnap)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.snapPointToClosestEdgeOfPolygonIfOutside2d(pointToSnap);
   }

   public void snapPointToEdgeTowardsInnerPointIfOutside(YoFramePoint pointToSnap, YoFramePoint innerPoint)
   {
      putYoValuesIntoSupportPolygon();
      quadrupedSupportPolygon.snapPointToEdgeTowardsInnerPointIfOutside(pointToSnap,  innerPoint);
   }
   
   public void setWithoutChecks(QuadrupedSupportPolygon quadrupedSupportPolygon)
   {
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon);
   }
   
   public void setWithoutChecks(YoQuadrupedSupportPolygon quadrupedSupportPolygon)
   {
      getYoValuesFromSupportPolygon(quadrupedSupportPolygon.getQuadrupedSupportPolygon());
   }
   
   public QuadrupedSupportPolygon getQuadrupedSupportPolygon()
   {
      putYoValuesIntoSupportPolygon();
      return quadrupedSupportPolygon;
   }
   
   private void putYoValuesIntoSupportPolygon()
   {
      quadrupedSupportPolygon.clear();
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (containsStorage.get(robotQuadrant).getBooleanValue())
         {
            quadrupedSupportPolygon.setFootstep(robotQuadrant, yoFootsteps.get(robotQuadrant));
         }
      }
   }
   
   private void getYoValuesFromSupportPolygon(QuadrupedSupportPolygon quadrupedSupportPolygon)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         containsStorage.get(robotQuadrant).set(quadrupedSupportPolygon.containsFootstep(robotQuadrant));
      }
      
      for (RobotQuadrant robotQuadrant : quadrupedSupportPolygon.getSupportingQuadrantsInOrder())
      {
         yoFootsteps.get(robotQuadrant).set(quadrupedSupportPolygon.getFootstep(robotQuadrant));
      }
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }
}
