package us.ihmc.commonWalkingControlModules.controlModules;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
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
   private static boolean DEBUG = false;
   
   @Test
   public void testSimpleWrenchDistributionWithGeometricFlatGroundDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      GroundReactionWrenchDistributorInterface distributor = new GeometricFlatGroundReactionWrenchDistributor(parentRegistry, null);

      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      
      testSimpleWrenchDistribution(centerOfMassFrame, distributor, parentRegistry);
   }
   
   @Test
   public void testSimpleWrenchDistributionWithLeeGoswamiDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      int nSupportVectors = 4;

      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      
      GroundReactionWrenchDistributorInterface distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, nSupportVectors, parentRegistry);
      testSimpleWrenchDistribution(centerOfMassFrame, distributor, parentRegistry);
   }

   @Test
   public void testRandomFlatGroundExamplesWithGeometricFlatGroundDistributor()
   {
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      GroundReactionWrenchDistributorInterface distributor = new GeometricFlatGroundReactionWrenchDistributor(parentRegistry, null);

      boolean verifyForcesAreInsideFrictionCones = false;
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, centerOfMassFrame, distributor, parentRegistry);
   }
   
   @Test
   public void testRandomFlatGroundExamplesWithLeeGoswamiDistributor()
   {
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      
      int nSupportVectors = 4;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      GroundReactionWrenchDistributorInterface distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, nSupportVectors, parentRegistry);
      
      boolean verifyForcesAreInsideFrictionCones = false;
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, centerOfMassFrame, distributor, parentRegistry);
   }
   
   private void testSimpleWrenchDistribution(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributorInterface distributor, YoVariableRegistry parentRegistry)
   {
      ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();

      double coefficientOfFriction = 1.0;
      double rotationalCoefficientOfFriction = 0.1;
     
      double footLength = 0.3;
      double footWidth = 0.15;
      Point3d leftMidfootLocation = new Point3d(0.0, 0.5, 0.0);
      FlatGroundPlaneContactState leftFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, leftMidfootLocation);
      distributor.addContact(leftFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);

      Point3d rightMidfootLocation = new Point3d(0.0, -0.5, 0.0);
      FlatGroundPlaneContactState rightFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, rightMidfootLocation);
      distributor.addContact(rightFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);

      contactStates.add(leftFootContactState);
      contactStates.add(rightFootContactState);

      Vector3d linearPart = new Vector3d(50.0, 60.0, 1000.0);
      Vector3d angularPart = new Vector3d(10.0, 12.0, 13.0);

      SpatialForceVector desiredNetSpatialForceVector = new SpatialForceVector(centerOfMassFrame, linearPart, angularPart);
      distributor.solve(desiredNetSpatialForceVector);

      printIfDebug("leftForce = " + distributor.getForce(leftFootContactState));
      printIfDebug("rightForce = " + distributor.getForce(rightFootContactState));
      
      printIfDebug("leftNormalTorque = " + distributor.getNormalTorque(leftFootContactState));
      printIfDebug("rightNormalTorque = " + distributor.getNormalTorque(rightFootContactState));
      
      printIfDebug("leftCenterOfPressure = " + distributor.getCenterOfPressure(leftFootContactState));
      printIfDebug("rightCenterOfPressure = " + distributor.getCenterOfPressure(rightFootContactState));

      verifyForcesAreInsideFrictionCones(distributor, contactStates, coefficientOfFriction, rotationalCoefficientOfFriction);
      verifyCentersOfPressureAreInsideContactPolygons(distributor, contactStates);
      verifyWrenchesSumToExpectedTotal(centerOfMassFrame, desiredNetSpatialForceVector, contactStates, distributor);
      
      if (VISUALIZE)
      {
         Robot robot = new Robot("null");

         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         int maxNumberOfFeet = 2;
         GroundReactionWrenchDistributorVisuzalizer visualizer = new GroundReactionWrenchDistributorVisuzalizer(maxNumberOfFeet, scs.getRootRegistry(), dynamicGraphicObjectsListRegistry);

         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         addCoordinateSystem(scs);
         
         scs.startOnAThread();

         visualizer.update(scs, distributor, centerOfMassFrame, contactStates, desiredNetSpatialForceVector);

         ThreadTools.sleepForever();
      }
   }

   
   private void testRandomFlatGroundExamples(boolean verifyForcesAreInsideFrictionCones, ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributorInterface distributor, YoVariableRegistry parentRegistry)
   {
      Random random = new Random(1776L);
      ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
 
      double coefficientOfFriction = 0.87;
      double rotationalCoefficientOfFriction = 0.13;
      
      GroundReactionWrenchDistributorVisuzalizer visualizer = null;
      SimulationConstructionSet scs = null;
      
      if (VISUALIZE)
      {
         Robot robot = new Robot("null");

         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
         scs = new SimulationConstructionSet(robot);

         int maxNumberOfFeet = 2; //6;
         visualizer = new GroundReactionWrenchDistributorVisuzalizer(maxNumberOfFeet, scs.getRootRegistry(), dynamicGraphicObjectsListRegistry);

         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         
         addCoordinateSystem(scs);
         scs.startOnAThread(); 
      }
      
      int numberOfTests = 5;
      
      for (int i=0; i<numberOfTests; i++)
      {
         distributor.reset();
         contactStates.clear();
         
         FlatGroundPlaneContactState leftFootContactState = FlatGroundPlaneContactState.createRandomFlatGroundContactState(random, true);
         FlatGroundPlaneContactState rightFootContactState = FlatGroundPlaneContactState.createRandomFlatGroundContactState(random, false);

         contactStates.add(leftFootContactState);
         contactStates.add(rightFootContactState);
         distributor.addContact(leftFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);
         distributor.addContact(rightFootContactState, coefficientOfFriction, rotationalCoefficientOfFriction);

         SpatialForceVector desiredNetSpatialForceVector = generateRandomAchievableSpatialForceVector(random, centerOfMassFrame, contactStates, coefficientOfFriction, rotationalCoefficientOfFriction);
         distributor.solve(desiredNetSpatialForceVector);

         if (verifyForcesAreInsideFrictionCones)
         {
            verifyForcesAreInsideFrictionCones(distributor, contactStates, coefficientOfFriction, rotationalCoefficientOfFriction);
         }
         
         verifyCentersOfPressureAreInsideContactPolygons(distributor, contactStates);
         verifyWrenchesSumToExpectedTotal(centerOfMassFrame, desiredNetSpatialForceVector, contactStates, distributor);

         if (VISUALIZE)
         {
            visualizer.update(scs, distributor, centerOfMassFrame, contactStates, desiredNetSpatialForceVector);
         }
      }
      
      if (VISUALIZE)
      {
         ThreadTools.sleepForever();
      }
   }

   private void addCoordinateSystem(SimulationConstructionSet scs)
   {
      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(0.2);
      scs.addStaticLinkGraphics(coordinateSystem, Graphics3DNodeType.VISUALIZATION);
   }
   
   private PoseReferenceFrame createCenterOfMassFrame(Point3d centerOfMassPosition)
   {
      PoseReferenceFrame centerOfMassFrame = new PoseReferenceFrame("com", ReferenceFrame.getWorldFrame());
      FramePose centerOfMassPose = new FramePose(ReferenceFrame.getWorldFrame(), centerOfMassPosition, new Quat4d());
      centerOfMassFrame.updatePose(centerOfMassPose);
      centerOfMassFrame.update();
      return centerOfMassFrame;
   }

   private void printIfDebug(String string)
   {
      if (DEBUG ) System.out.println(string); 
   }

   private void verifyWrenchesSumToExpectedTotal(ReferenceFrame centerOfMassFrame, SpatialForceVector totalBodyWrench,
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

         FrameVector copToCoMVector = new FrameVector(centerOfMassFrame);
         copToCoMVector.changeFrame(expressedInFrame);
         copToCoMVector.sub(centerOfPressure3d);

         FrameVector crossProductTorque = new FrameVector(expressedInFrame);
         crossProductTorque.cross(contactForce, copToCoMVector);

         totalMoment.add(crossProductTorque);
         totalMoment.add(normalTorque.changeFrameCopy(expressedInFrame));
      }

      FrameVector totalBodyForce = totalBodyWrench.getLinearPartAsFrameVectorCopy();
      assertTrue(totalForce.epsilonEquals(totalBodyForce, 1e-7));

      FrameVector totalBodyMoment = totalBodyWrench.getAngularPartAsFrameVectorCopy();
      assertTrue("totalMoment = " + totalMoment + ", totalBodyMoment = " + totalBodyMoment, totalMoment.epsilonEquals(totalBodyMoment, 1e-7));
   }

   private void verifyForcesAreInsideFrictionCones(GroundReactionWrenchDistributorInterface distributor, ArrayList<PlaneContactState> contactStates,
         double coefficientOfFriction, double normalTorqueCoefficientOfFriction)
   {
      for (PlaneContactState contactState : contactStates)
      {
         FrameVector force = distributor.getForce(contactState);
         double normalTorqe = distributor.getNormalTorque(contactState);

         verifyForceIsInsideFrictionCone(force, normalTorqe, contactState, coefficientOfFriction, normalTorqueCoefficientOfFriction);
      }
   }
   
   private void verifyCentersOfPressureAreInsideContactPolygons(GroundReactionWrenchDistributorInterface distributor, ArrayList<PlaneContactState> contactStates)
   {
      for (PlaneContactState contactState : contactStates)
      {
         FramePoint2d centerOfPressure = distributor.getCenterOfPressure(contactState);
         verifyCenterOfPressureIsInsideFoot(centerOfPressure, contactState);
      }
   }
   
   private void verifyCenterOfPressureIsInsideFoot(FramePoint2d centerOfPressure, PlaneContactState planeContactState)
   {
      centerOfPressure.checkReferenceFrameMatch(planeContactState.getPlaneFrame());
      List<FramePoint2d> contactPoints = planeContactState.getContactPoints2d();
      FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(contactPoints);

      assertTrue(footPolygon.isPointInside(centerOfPressure, 1e-7));
   }

   private void verifyForceIsInsideFrictionCone(FrameVector forceVector, double normalTorque, PlaneContactState planeContactState, double coefficientOfFriction, double normalTorqueCoefficientOfFriction)
   {
      forceVector = forceVector.changeFrameCopy(planeContactState.getPlaneFrame());

      double normalForce = forceVector.getZ();
      double parallelForce = Math.sqrt(forceVector.getX() * forceVector.getX() + forceVector.getY() * forceVector.getY());

      if (parallelForce > coefficientOfFriction * normalForce)
         fail("Outside of Friction Cone! forceVector = " + forceVector + ", planeContactState = " + planeContactState);

      if (Math.abs(normalTorque) > normalTorqueCoefficientOfFriction * normalForce)
         fail("Too much normal torque! normalTorque = " + normalTorque + ", normalForce = " + normalForce + ", normalTorqueCoefficientOfFriction = " + normalTorqueCoefficientOfFriction);
   }

   private static class FlatGroundPlaneContactState implements PlaneContactState
   {
      private final ArrayList<FramePoint> contactPoints;
      private final ArrayList<FramePoint2d> contactPoints2d;

      public static FlatGroundPlaneContactState createRandomFlatGroundContactState(Random random, boolean leftSide)
      {
         double footLength = RandomTools.generateRandomDouble(random, 0.1, 0.3);
         double footWidth = RandomTools.generateRandomDouble(random, 0.1, 0.2);
         
         Point3d midfootLocation = RandomTools.generateRandomPoint(random, -1.0, 0.3, 0.0, 1.0, 1.0, 0.0);
         midfootLocation.setZ(0.0);
         
         if (!leftSide) midfootLocation.setY(-midfootLocation.getY());
         
         FlatGroundPlaneContactState flatGroundPlaneContactState = new FlatGroundPlaneContactState(footLength, footWidth, midfootLocation);
         return flatGroundPlaneContactState;
      }
      
      public FlatGroundPlaneContactState(double footLength, double footWidth, Point3d midfootLocation)
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

   private static SpatialForceVector generateRandomAchievableSpatialForceVector(Random random, ReferenceFrame centerOfMassFrame, ArrayList<PlaneContactState> contactStates, double coefficientOfFriction, double normalTorqueCoefficientOfFriction)
   {
      SpatialForceVector spatialForceVector = new SpatialForceVector(centerOfMassFrame);
      
      for (PlaneContactState contactState : contactStates)
      {
         ReferenceFrame contactPlaneFrame = contactState.getPlaneFrame();
       
         double normalForce = RandomTools.generateRandomDouble(random, 10.0, 100.0);
         double parallelForceMagnitude = random.nextDouble() * coefficientOfFriction * normalForce;
         
         Vector2d parallelForce2d = RandomTools.generateRandomVector2d(random, parallelForceMagnitude);
         Vector3d totalForce = new Vector3d(parallelForce2d.getX(), parallelForce2d.getY(), normalForce);
         
         double normalTorque = random.nextDouble() * normalTorqueCoefficientOfFriction * normalForce;
         Vector3d totalTorque = new Vector3d(0.0, 0.0, normalTorque);

         ReferenceFrame centerOfPressureFrame = generateRandomCenterOfPressureFrame(random, contactState, contactPlaneFrame);
         SpatialForceVector spatialForceVectorContributedByThisContact = new SpatialForceVector(centerOfPressureFrame, totalForce, totalTorque);
         
         spatialForceVectorContributedByThisContact.changeFrame(centerOfMassFrame);
         spatialForceVector.add(spatialForceVectorContributedByThisContact);
      }
      
      return spatialForceVector;
   }

   private static ReferenceFrame generateRandomCenterOfPressureFrame(Random random, PlaneContactState contactState, ReferenceFrame contactPlaneFrame)
   {
      PoseReferenceFrame centerOfPressureFrame = new PoseReferenceFrame("centerOfPressure", contactPlaneFrame);
      Point2d pointInsideContact = generateRandomPointInsideContact(random, contactState);
      Point3d centerOfPressurePosition = new Point3d(pointInsideContact.getX(), pointInsideContact.getY(), 0.0);
      FramePose framePose = new FramePose(contactPlaneFrame, centerOfPressurePosition, new Quat4d());
      centerOfPressureFrame.updatePose(framePose);
      centerOfPressureFrame.update();

      return centerOfPressureFrame;
   }

   private static Point2d generateRandomPointInsideContact(Random random, PlaneContactState contactState)
   {
      Point2d ret = new Point2d();
      double totalWeight = 0.0;

      List<FramePoint2d> contactPoints = contactState.getContactPoints2d();
      for (FramePoint2d contactPoint : contactPoints)
      {
         Point2d point2d = contactPoint.getPointCopy();
         double weight = random.nextDouble();
         point2d.scale(weight);
         ret.add(point2d);
         totalWeight += weight;
      }

      ret.scale(1.0/totalWeight);

      return ret;
   }

}
