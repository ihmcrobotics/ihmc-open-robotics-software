package us.ihmc.commonWalkingControlModules.momentumBasedController;

import static junit.framework.Assert.assertTrue;

import java.util.Collection;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.utilities.test.JUnitTools;

/**
 * @author twan
 *         Date: 5/7/13
 */
public class MomentumControlTestTools
{
   public static void assertWrenchesSumUpToMomentumDot(Collection<Wrench> externalWrenches, SpatialForceVector desiredCentroidalMomentumRate,
                                                       double gravityZ, double mass, ReferenceFrame centerOfMassFrame, double epsilon)
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

      JUnitTools.assertSpatialForceVectorEquals(desiredCentroidalMomentumRate, totalWrench, epsilon);
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

         FramePoint2d cop = new FramePoint2d(planeFrame);
         centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, planeFrame);

         FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(contactState.getContactFramePoints2dInContactCopy());
         assertTrue(supportPolygon.isPointInside(cop));
      }
   }

   public static void assertRootJointWrenchZero(Map<RigidBody, Wrench> externalWrenches, SixDoFJoint rootJoint, double gravityZ, double epsilon)
   {
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
      for (RigidBody rigidBody : externalWrenches.keySet())
      {
         Wrench externalWrench = externalWrenches.get(rigidBody);
         externalWrench.changeFrame(rigidBody.getBodyFixedFrame());
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrench);
      }
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();
      Wrench wrench = new Wrench();
      rootJoint.packWrench(wrench);

      SpatialForceVector zeroWrench = new SpatialForceVector(wrench.getExpressedInFrame());
      JUnitTools.assertSpatialForceVectorEquals(wrench, zeroWrench, epsilon);
   }
}
