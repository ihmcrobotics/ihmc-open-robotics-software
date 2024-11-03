package us.ihmc.commonWalkingControlModules.momentumBasedController;

import static us.ihmc.robotics.Assert.*;

import java.util.Collection;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;

/**
 * @author twan
 *         Date: 5/7/13
 */
public class MomentumControlTestTools
{
   public static void assertWrenchesSumUpToMomentumDot(Collection<Wrench> externalWrenches, SpatialForce desiredCentroidalMomentumRate, double gravityZ,
         double mass, ReferenceFrame centerOfMassFrame, double epsilon)
   {
      SpatialForce totalWrench = new SpatialForce(centerOfMassFrame);
      Wrench tempWrench = new Wrench();
      for (Wrench wrench : externalWrenches)
      {
         tempWrench.setIncludingFrame(wrench);
         tempWrench.changeFrame(centerOfMassFrame);
         totalWrench.add(tempWrench);
      }

      Wrench gravitationalWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);
      gravitationalWrench.setLinearPartZ(-mass * gravityZ);
      totalWrench.add(gravitationalWrench);

      MecanoTestTools.assertSpatialForceEquals(desiredCentroidalMomentumRate, totalWrench, epsilon);
   }

   public static void assertWrenchesInFrictionCones(Map<RigidBodyBasics, Wrench> externalWrenches,
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

         FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactState.getContactFramePoints2dInContactCopy()));
         assertTrue(supportPolygon.isPointInside(cop));
      }
   }

   public static void assertRootJointWrenchZero(Map<RigidBodyBasics, Wrench> externalWrenches, SixDoFJoint rootJoint, double gravityZ, double epsilon)
   {
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(rootJoint.getPredecessor());
      inverseDynamicsCalculator.setGravitionalAcceleration(-gravityZ);
      for (RigidBodyBasics rigidBody : externalWrenches.keySet())
      {
         Wrench externalWrench = externalWrenches.get(rigidBody);
         externalWrench.changeFrame(rigidBody.getBodyFixedFrame());
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrench);
      }
      inverseDynamicsCalculator.compute();
      Wrench wrench = new Wrench();
      wrench.setIncludingFrame(rootJoint.getJointWrench());

      SpatialForce zeroWrench = new SpatialForce(wrench.getReferenceFrame());
      MecanoTestTools.assertSpatialForceEquals(wrench, zeroWrench, epsilon);
   }
}
