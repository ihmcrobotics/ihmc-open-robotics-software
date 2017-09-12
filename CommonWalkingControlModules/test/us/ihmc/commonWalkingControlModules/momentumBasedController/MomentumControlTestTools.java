package us.ihmc.commonWalkingControlModules.momentumBasedController;

import static junit.framework.Assert.assertTrue;

import java.util.Collection;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialForceVectorTest;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 5/7/13
 */
public class MomentumControlTestTools
{
   public static void assertWrenchesSumUpToMomentumDot(Collection<Wrench> externalWrenches, SpatialForceVector desiredCentroidalMomentumRate, double gravityZ,
         double mass, ReferenceFrame centerOfMassFrame, double epsilon)
   {
      SpatialForceVector totalWrench = new SpatialForceVector(centerOfMassFrame);
      Wrench tempWrench = new Wrench();
      for (Wrench wrench : externalWrenches)
      {
         tempWrench.set(wrench);
         tempWrench.changeFrame(centerOfMassFrame);
         totalWrench.add(tempWrench);
      }

      Wrench gravitationalWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);
      gravitationalWrench.setLinearPartZ(-mass * gravityZ);
      totalWrench.add(gravitationalWrench);

      SpatialForceVectorTest.assertSpatialForceVectorEquals(desiredCentroidalMomentumRate, totalWrench, epsilon);
   }

   public static void assertWrenchesInFrictionCones(Map<RigidBody, Wrench> externalWrenches,
         Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, double coefficientOfFriction)
   {
      CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

      for (ContactablePlaneBody contactablePlaneBody : contactStates.keySet())
      {
         Wrench wrench = externalWrenches.get(contactablePlaneBody.getRigidBody());
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);
         ReferenceFrame planeFrame = contactState.getPlaneFrame();

         wrench.changeFrame(planeFrame);

         double fZ = wrench.getLinearPartZ();
         assertTrue(fZ > 0.0);

         double fT = Math.hypot(wrench.getLinearPartX(), wrench.getLinearPartY());
         assertTrue(fT / fZ < coefficientOfFriction);

         FramePoint2D cop = new FramePoint2D(planeFrame);
         centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, planeFrame);

         FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(contactState.getContactFramePoints2dInContactCopy());
         assertTrue(supportPolygon.isPointInside(cop));
      }
   }

   public static void assertRootJointWrenchZero(Map<RigidBody, Wrench> externalWrenches, SixDoFJoint rootJoint, double gravityZ, double epsilon)
   {
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(rootJoint.getPredecessor(), gravityZ);
      for (RigidBody rigidBody : externalWrenches.keySet())
      {
         Wrench externalWrench = externalWrenches.get(rigidBody);
         externalWrench.changeFrame(rigidBody.getBodyFixedFrame());
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrench);
      }
      inverseDynamicsCalculator.compute();
      Wrench wrench = new Wrench();
      rootJoint.getWrench(wrench);

      SpatialForceVector zeroWrench = new SpatialForceVector(wrench.getExpressedInFrame());
      SpatialForceVectorTest.assertSpatialForceVectorEquals(wrench, zeroWrench, epsilon);
   }
}
