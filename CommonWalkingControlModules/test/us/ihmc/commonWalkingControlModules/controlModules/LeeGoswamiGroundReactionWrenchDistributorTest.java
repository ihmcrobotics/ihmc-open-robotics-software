package us.ihmc.commonWalkingControlModules.controlModules;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class LeeGoswamiGroundReactionWrenchDistributorTest
{
   private static final boolean VISUALIZE = false;

   @Test
   public void testSimpleWrenchDistribution()
   {
      ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();

      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      double mass = 100.0;
      double coefficientOfFriction = 1.0;

      PoseReferenceFrame centerOfMassFrame = new PoseReferenceFrame("com", ReferenceFrame.getWorldFrame());
      FramePoint centerOfMassPosition = new FramePoint(ReferenceFrame.getWorldFrame(), centerOfMassPoint3d);
      FramePose centerOfMassPose = new FramePose(centerOfMassPosition, new FrameOrientation(ReferenceFrame.getWorldFrame()));
      centerOfMassFrame.updatePose(centerOfMassPose);
      centerOfMassFrame.update();

      FrameVector gravitationalAcceleration = new FrameVector(centerOfMassFrame, 0.0, 0.0, -9.81);
      int nSupportVectors = 4;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");

      GroundReactionWrenchDistributorInterface distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, nSupportVectors, parentRegistry);

//    GroundReactionWrenchDistributorInterface distributor = new GeometricFlatGroundReactionWrenchDistributor(centerOfMassFrame, gravitationalAcceleration, mass, parentRegistry, null);

      double rotationalCoefficientOfFriction = 0.5;
      double footLength = 0.3;
      double footWidth = 0.15;
      Point3d leftMidfootLocation = new Point3d(0.0, 0.5, 0.0);
      SimplePlaneContactState leftFootContactState = new SimplePlaneContactState(footLength, footWidth, leftMidfootLocation);
      distributor.addContact(leftFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);

      Point3d rightMidfootLocation = new Point3d(0.0, -0.5, 0.0);
      SimplePlaneContactState rightFootContactState = new SimplePlaneContactState(footLength, footWidth, rightMidfootLocation);
      distributor.addContact(rightFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);

      contactStates.add(leftFootContactState);
      contactStates.add(rightFootContactState);

      Vector3d linearPart = new Vector3d(0.0, 0.0, mass * 9.81);
      Vector3d angularPart = new Vector3d(0.0, 0.0, 0.0);

      SpatialForceVector desiredNetSpatialForceVector = new SpatialForceVector(ReferenceFrame.getWorldFrame(), linearPart, angularPart);
      distributor.solve(desiredNetSpatialForceVector);

      if (VISUALIZE)
      {
         Robot robot = new Robot("null");

         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         int maxNumberOfFeet = 6;
         int maxNumberOfVertices = 10;
         GroundReactionWrenchDistributorVisuzalizer visualizer = new GroundReactionWrenchDistributorVisuzalizer(maxNumberOfFeet, maxNumberOfVertices,
                                                                    scs.getRootRegistry(), dynamicGraphicObjectsListRegistry);

         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         scs.startOnAThread();

         visualizer.update(distributor, centerOfMassFrame, contactStates, desiredNetSpatialForceVector);

         ThreadTools.sleepForever();
      }

      FrameVector leftForce = distributor.getForce(leftFootContactState);
      FrameVector rightForce = distributor.getForce(rightFootContactState);

      double epsilon = 1e-7;
      FrameVector expectedLeftForce = new FrameVector(gravitationalAcceleration);
      expectedLeftForce.scale(-0.5 * mass);

//    assertTrue(leftForce.epsilonEquals(expectedLeftForce, epsilon));

      FrameVector expectedRightForce = new FrameVector(gravitationalAcceleration);
      expectedRightForce.scale(-0.5 * mass);

//    assertTrue(rightForce.epsilonEquals(expectedRightForce, epsilon)); 

      verifyForceIsInsideFrictionCone(leftForce, leftFootContactState, coefficientOfFriction);
      verifyForceIsInsideFrictionCone(rightForce, rightFootContactState, coefficientOfFriction);

      FramePoint2d leftCenterOfPressure = distributor.getCenterOfPressure(leftFootContactState);
      FramePoint2d rightCenterOfPressure = distributor.getCenterOfPressure(rightFootContactState);

      FramePoint2d expectedLeftCenterOfPressure = new FramePoint2d(ReferenceFrame.getWorldFrame(), leftMidfootLocation.getX(), leftMidfootLocation.getY());
      FramePoint2d expectedRightCenterOfPressure = new FramePoint2d(ReferenceFrame.getWorldFrame(), rightMidfootLocation.getX(), rightMidfootLocation.getY());

      assertTrue(leftCenterOfPressure.epsilonEquals(expectedLeftCenterOfPressure, epsilon));
      assertTrue(rightCenterOfPressure.epsilonEquals(expectedRightCenterOfPressure, epsilon));

      verifyCenterOfPressureIsInsideFoot(leftCenterOfPressure, leftFootContactState);
      verifyCenterOfPressureIsInsideFoot(rightCenterOfPressure, rightFootContactState);


      verifyWrenchesSumToExpectedTotal(centerOfMassPosition, desiredNetSpatialForceVector, contactStates, distributor);

   }


   private void verifyWrenchesSumToExpectedTotal(FramePoint centerOfMassPosition, SpatialForceVector totalBodyWrench,
           ArrayList<PlaneContactState> contactStates, GroundReactionWrenchDistributorInterface distributor)
   {
      ReferenceFrame expressedInFrame = totalBodyWrench.getExpressedInFrame();

      FrameVector totalForce = new FrameVector(expressedInFrame);
      FrameVector totalMoment = new FrameVector(expressedInFrame);

      for (PlaneContactState planeContactState : contactStates)
      {
         FrameVector contactForce = distributor.getForce(planeContactState).changeFrameCopy(expressedInFrame);
         totalForce.add(contactForce);

         double normalTorqueMagnitude = distributor.getNormalTorque(planeContactState);
         FrameVector normalTorque = new FrameVector(planeContactState.getPlaneFrame(), 0.0, 0.0, normalTorqueMagnitude);

         FramePoint2d centerOfPressure2d = distributor.getCenterOfPressure(planeContactState);

         FramePoint centerOfPressure3d = new FramePoint(centerOfPressure2d.getReferenceFrame());
         centerOfPressure3d.setXY(centerOfPressure2d);
         centerOfPressure3d.changeFrame(expressedInFrame);

         FrameVector copToCoMVector = new FrameVector(centerOfMassPosition);
         copToCoMVector.changeFrame(expressedInFrame);
         copToCoMVector.sub(centerOfPressure3d);

         FrameVector crossProductTorque = new FrameVector(expressedInFrame);
         crossProductTorque.cross(copToCoMVector, contactForce);

         totalMoment.add(crossProductTorque);
         totalMoment.add(normalTorque.changeFrameCopy(expressedInFrame));
      }

      FrameVector groundReactionForce = totalBodyWrench.getLinearPartAsFrameVectorCopy();
      assertTrue(totalForce.epsilonEquals(groundReactionForce, 1e-7));

      FrameVector groundReactionMoment = totalBodyWrench.getAngularPartAsFrameVectorCopy();
      assertTrue(totalMoment.epsilonEquals(groundReactionMoment, 1e-7));
   }


   private static class SimplePlaneContactState implements PlaneContactState
   {
      private final ArrayList<FramePoint> contactPoints;
      private final ArrayList<FramePoint2d> contactPoints2d;

      public SimplePlaneContactState(double footLength, double footWidth, Point3d midfootLocation)
      {
         contactPoints = new ArrayList<FramePoint>();
         contactPoints2d = new ArrayList<FramePoint2d>();

         Point3d frontLeft = new Point3d(midfootLocation);
         frontLeft.setX(frontLeft.getX() + footLength / 2.0);
         frontLeft.setY(frontLeft.getY() + footWidth / 2.0);

         Point3d frontRight = new Point3d(midfootLocation);
         frontRight.setX(frontRight.getX() + footLength / 2.0);
         frontRight.setY(frontRight.getY() - footWidth / 2.0);

         Point3d backLeft = new Point3d(midfootLocation);
         backLeft.setX(backLeft.getX() - footLength / 2.0);
         backLeft.setY(backLeft.getY() + footWidth / 2.0);

         Point3d backRight = new Point3d(midfootLocation);
         backRight.setX(backRight.getX() - footLength / 2.0);
         backRight.setY(backRight.getY() - footWidth / 2.0);

         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), frontLeft));
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), frontRight));
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), backRight));
         contactPoints.add(new FramePoint(ReferenceFrame.getWorldFrame(), backLeft));

         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(frontLeft)));
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(frontRight)));
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(backRight)));
         contactPoints2d.add(new FramePoint2d(ReferenceFrame.getWorldFrame(), projectToXY(backLeft)));

      }

      private Point2d projectToXY(Point3d point)
      {
         return new Point2d(point.getX(), point.getY());
      }

      public boolean inContact()
      {
         return true;
      }

      public List<FramePoint> getContactPoints()
      {
         return contactPoints;
      }

      public List<FramePoint2d> getContactPoints2d()
      {
         return contactPoints2d;
      }

      public ReferenceFrame getBodyFrame()
      {
         return ReferenceFrame.getWorldFrame();
      }

      public ReferenceFrame getPlaneFrame()
      {
         return ReferenceFrame.getWorldFrame();
      }

   }


   ;

   private void verifyCenterOfPressureIsInsideFoot(FramePoint2d centerOfPressure, PlaneContactState planeContactState)
   {
      centerOfPressure.checkReferenceFrameMatch(planeContactState.getPlaneFrame());

      List<FramePoint2d> contactPoints = planeContactState.getContactPoints2d();

      FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(contactPoints);

      assertTrue(footPolygon.isPointInside(centerOfPressure, 1e-7));
   }

   private void verifyForceIsInsideFrictionCone(FrameVector forceVector, PlaneContactState planeContactState, double coefficientOfFriction)
   {
      forceVector = forceVector.changeFrameCopy(planeContactState.getPlaneFrame());

      double normalForce = forceVector.getZ();
      double parallelForce = Math.sqrt(forceVector.getX() * forceVector.getX() + forceVector.getY() * forceVector.getY());

      if (parallelForce > coefficientOfFriction * normalForce)
         fail("Outside of Friction Cone! forceVector = " + forceVector + ", planeContactState = " + planeContactState);

   }

}
