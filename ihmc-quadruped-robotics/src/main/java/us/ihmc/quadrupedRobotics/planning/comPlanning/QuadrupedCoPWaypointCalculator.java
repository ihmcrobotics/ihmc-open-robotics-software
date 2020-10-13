package us.ihmc.quadrupedRobotics.planning.comPlanning;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoPWaypointCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedCenterOfPressureTools;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerParameters;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class QuadrupedCoPWaypointCalculator implements CoPWaypointCalculator<QuadrupedTimedContactInterval>
{
   private final QuadrantDependentList<MutableDouble> normalizedContactPressures = new QuadrantDependentList<>(MutableDouble::new);

   private final FrameVector3D copOffsetFromStance = new FrameVector3D();

   private final DCMPlannerParameters plannerParameters;

   public QuadrupedCoPWaypointCalculator(DCMPlannerParameters plannerParameters)
   {
      this.plannerParameters = plannerParameters;
   }

   public void computeCoPWaypoint(FixedFramePoint3DBasics copWaypointToPack, QuadrupedTimedContactInterval timedContactPhase)
   {
      QuadrantDependentList<FramePoint3D> solePosition = timedContactPhase.getSolePosition();
      QuadrantDependentList<ContactState> contactState = timedContactPhase.getContactState();

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, timedContactPhase.getContactState());
      QuadrupedCenterOfPressureTools.computeCenterOfPressure(copWaypointToPack, timedContactPhase.getSolePosition(), normalizedContactPressures);

      computeCoPOffsetFromStance(contactState, solePosition, copOffsetFromStance);

      copWaypointToPack.add(copOffsetFromStance);

      constrainToPolygon(contactState, solePosition, copWaypointToPack);
   }

   private final Quaternion nominalOrientation = new Quaternion();

   private final FramePoint3D smallSideContact = new FramePoint3D();
   private final FramePoint3D smallEndContact = new FramePoint3D();
   private final FramePoint3D tempBigSideContact = new FramePoint3D();
   private final FramePoint3D tempBigEndContact = new FramePoint3D();

   private final PoseReferenceFrame stepFrame = new PoseReferenceFrame("stepFrame", ReferenceFrame.getWorldFrame());

   private final QuadrantDependentList<FramePoint3DReadOnly> tempList = new QuadrantDependentList<>();

   private void computeCoPOffsetFromStance(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions,
                                           FrameVector3D copOffsetToPack)
   {
      copOffsetToPack.setToZero();

      int numberOfLeftFeetInContact = 0;
      int numberOfRightFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;
      int numberOfHindFeetInContact = 0;
      int numberOfFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            numberOfFeetInContact++;
            if (robotQuadrant.isQuadrantInFront())
               numberOfFrontFeetInContact++;
            else
               numberOfHindFeetInContact++;
            if (robotQuadrant.isQuadrantOnLeftSide())
               numberOfLeftFeetInContact++;
            else
               numberOfRightFeetInContact++;
         }
      }

      if (numberOfFeetInContact != 3)
         return;

      RobotSide smallSide;
      RobotEnd smallEnd;

      if (numberOfLeftFeetInContact > numberOfRightFeetInContact)
         smallSide = RobotSide.RIGHT; // left side has more feet in contact
      else
         smallSide = RobotSide.LEFT;

      if (numberOfFrontFeetInContact > numberOfHindFeetInContact)
         smallEnd = RobotEnd.HIND; // front end has more feet in contact
      else
         smallEnd = RobotEnd.FRONT;

      RobotSide bigSide = smallSide.getOppositeSide();
      RobotEnd bigEnd = smallEnd.getOppositeEnd();

      RobotQuadrant smallEndQuadrant = RobotQuadrant.getQuadrant(smallEnd, bigSide);
      RobotQuadrant smallSideQuadrant = RobotQuadrant.getQuadrant(bigEnd, smallSide);

      double nominalYaw = getNominalYawForStance(contactState, solePositions);
      nominalOrientation.setToYawOrientation(nominalYaw);

      smallSideContact.setIncludingFrame(solePositions.get(smallSideQuadrant));
      smallEndContact.setIncludingFrame(solePositions.get(smallEndQuadrant));

      stepFrame.setPoseAndUpdate(smallEndContact, nominalOrientation);

      double averageLength = 0.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         tempBigEndContact.setIncludingFrame(solePositions.get(RobotQuadrant.getQuadrant(bigEnd, robotSide)));
         tempBigSideContact.changeFrame(stepFrame);
         averageLength += 0.5 * Math.abs(tempBigEndContact.getX());
      }

      stepFrame.setPoseAndUpdate(smallSideContact, nominalOrientation);

      double averageWidth = 0.0;
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         tempBigSideContact.setIncludingFrame(solePositions.get(RobotQuadrant.getQuadrant(robotEnd, bigSide)));
         tempBigSideContact.changeFrame(stepFrame);
         averageWidth += 0.5 * Math.abs(tempBigSideContact.getY());
      }

      double lengthShift = smallEnd.negateIfHindEnd(
            MathTools.clamp(plannerParameters.getStanceLengthCoPShiftFactor() * averageLength, plannerParameters.getMaxStanceLengthCoPShift()));
      double widthShift = smallSide
            .negateIfRightSide(MathTools.clamp(plannerParameters.getStanceWidthCoPShiftFactor() * averageWidth, plannerParameters.getMaxStanceWidthCoPShift()));
      copOffsetToPack.setIncludingFrame(stepFrame, lengthShift, widthShift, 0.0);
      copOffsetToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private final FramePoint2D tempFramePoint2D = new FramePoint2D();
   private final ConvexPolygon2D constraintPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D scaledConstraintPolygon = new ConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();

   private void constrainToPolygon(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions,
                                   FixedFramePoint3DBasics copToConstraint)
   {
      constraintPolygon.clear();
      int numberOfFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant).isLoadBearing())
         {
            numberOfFeetInContact++;
            constraintPolygon.addVertex(solePositions.get(robotQuadrant));
         }
      }
      constraintPolygon.update();

      tempFramePoint2D.set(copToConstraint);

      // don't need to do anything if it's already inside
      if (constraintPolygon.signedDistance(tempFramePoint2D) <= -plannerParameters.getSafeDistanceFromSupportPolygonEdges())
         return;

      if (numberOfFeetInContact > 2)
      {
         polygonScaler.scaleConvexPolygon(constraintPolygon, plannerParameters.getSafeDistanceFromSupportPolygonEdges(), scaledConstraintPolygon);
         scaledConstraintPolygon.orthogonalProjection(tempFramePoint2D);
         copToConstraint.set(tempFramePoint2D);
      }
      else
      {
         constraintPolygon.orthogonalProjection(tempFramePoint2D);
         copToConstraint.set(tempFramePoint2D);
      }
   }

   private double getNominalYawForStance(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePositions)
   {
      tempList.clear();
      int numberOfVertices = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant).isLoadBearing())
         {
            tempList.put(robotQuadrant, solePositions.get(robotQuadrant));
            numberOfVertices++;
         }
      }
      return QuadrupedSupportPolygon.getNominalYaw(tempList, numberOfVertices);
   }
}
