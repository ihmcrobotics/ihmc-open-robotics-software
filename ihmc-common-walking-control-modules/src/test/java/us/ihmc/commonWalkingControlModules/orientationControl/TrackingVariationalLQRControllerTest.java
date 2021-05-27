package us.ihmc.commonWalkingControlModules.orientationControl;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class TrackingVariationalLQRControllerTest
{
   /*
   @Test
   public void testWithNoFeedback()
   {
      TrackingVariationalLQRController controller = new TrackingVariationalLQRController();

      int iters = 1000;
      Random random = new Random(1738L);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("", ReferenceFrame.getWorldFrame(), registry);
      MultipleWaypointsPositionTrajectoryGenerator torqueTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("", ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < iters; i++)
      {
         double duration = RandomNumbers.nextDouble(random, 0.1, 10.0);
         QuaternionReadOnly desiredOrientation = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3DReadOnly desiredAngularVelocity = EuclidCoreRandomTools.nextVector3D(random, new Vector3D(10.0, 10.0, 10.0));
         Vector3DReadOnly desiredTorque = EuclidCoreRandomTools.nextVector3D(random, new Vector3D(10.0, 10.0, 10.0));

         orientationTrajectory.clear();
         torqueTrajectory.clear();

         orientationTrajectory.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
         orientationTrajectory.appendWaypoint(duration, desiredOrientation, new Vector3D());
         orientationTrajectory.initialize();

         torqueTrajectory.appendWaypoint(0.0, new Point3D(), desiredTorque);
         torqueTrajectory.appendWaypoint(duration, new Point3D(), desiredTorque);
         torqueTrajectory.initialize();

         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();
         contactStateProvider.getTimeInterval().setInterval(0.0, duration);
         List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
         contactStateProviders.add(contactStateProvider);

         controller.setTrajectories(orientationTrajectory, torqueTrajectory, contactStateProviders);
         controller.compute(0.0, desiredOrientation, desiredAngularVelocity);

         Vector3DBasics feedbackTorque = new Vector3D();
         controller.getDesiredTorque(feedbackTorque);

         EuclidCoreTestTools.assertVector3DGeometricallyEquals(desiredTorque, feedbackTorque, 1e-5);
      }
   }

    */

   @Test
   public void testWithEasyOrientationError()
   {
      TrackingVariationalLQRController controller = new TrackingVariationalLQRController();

      int iters = 1000;
      Random random = new Random(1738L);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("", ReferenceFrame.getWorldFrame(), registry);
      MultipleWaypointsPositionTrajectoryGenerator torqueTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("", ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < iters; i++)
      {
         double duration = RandomNumbers.nextDouble(random, 0.1, 10.0);

         QuaternionReadOnly desiredOrientation = new Quaternion();
         Vector3DReadOnly desiredAngularVelocity = new Vector3D();
//         Vector3DReadOnly desiredTorque = EuclidCoreRandomTools.nextVector3D(random, new Vector3D(10.0, 10.0, 10.0));
         Vector3D desiredTorque = new Vector3D();

         QuaternionBasics pitchedOrientation = new Quaternion(desiredOrientation);
         QuaternionBasics yawedOrientation = new Quaternion(desiredOrientation);
         QuaternionBasics rolledOrientation = new Quaternion(desiredOrientation);

         double pitchRotation = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double yawRotation = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double rollRotation = RandomNumbers.nextDouble(random, Math.toRadians(10.0));

         pitchedOrientation.setToPitchOrientation(pitchRotation);
         yawedOrientation.setToYawOrientation(yawRotation);
         rolledOrientation.setToRollOrientation(rollRotation);

         orientationTrajectory.clear();
         torqueTrajectory.clear();

         orientationTrajectory.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
         orientationTrajectory.appendWaypoint(duration, desiredOrientation, new Vector3D());
         orientationTrajectory.initialize();

         torqueTrajectory.appendWaypoint(0.0, new Point3D(), desiredTorque);
         torqueTrajectory.appendWaypoint(duration, new Point3D(), desiredTorque);
         torqueTrajectory.initialize();


         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();
         contactStateProvider.getTimeInterval().setInterval(0.0, duration);
         List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
         contactStateProviders.add(contactStateProvider);

         controller.setTrajectories(orientationTrajectory, torqueTrajectory, contactStateProviders);
         Vector3DBasics feedbackTorque = new Vector3D();

         // check yaw only
         controller.compute(0.0, yawedOrientation, desiredAngularVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(-Math.signum(yawRotation), Math.signum(feedbackTorque.getZ()), 1e-5);

         // check pitch only
         controller.compute(0.0, pitchedOrientation, desiredAngularVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(-Math.signum(pitchRotation), Math.signum(feedbackTorque.getY()), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);

         // check roll only
         controller.compute(0.0, rolledOrientation, desiredAngularVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(-Math.signum(rollRotation), Math.signum(feedbackTorque.getX()), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);
      }
   }

   @Test
   public void testWithEasyAngularVelocityError()
   {
      TrackingVariationalLQRController controller = new TrackingVariationalLQRController();

      int iters = 1000;
      Random random = new Random(1738L);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("", ReferenceFrame.getWorldFrame(), registry);
      MultipleWaypointsPositionTrajectoryGenerator torqueTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("", ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < iters; i++)
      {
         double duration = RandomNumbers.nextDouble(random, 0.1, 10.0);

         QuaternionReadOnly desiredOrientation = new Quaternion();
         Vector3DReadOnly desiredAngularVelocity = new Vector3D();
         Vector3D desiredTorque = new Vector3D();

         Vector3D pitchedVelocity = new Vector3D();
         Vector3D yawedVelocity = new Vector3D();
         Vector3D rolledVelocity = new Vector3D();

         double pitchVelocity = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double yawVelocity = RandomNumbers.nextDouble(random, Math.toRadians(10.0));
         double rollVelocity = RandomNumbers.nextDouble(random, Math.toRadians(10.0));

         pitchedVelocity.setY(pitchVelocity);
         rolledVelocity.setX(rollVelocity);
         yawedVelocity.setZ(yawVelocity);

         orientationTrajectory.clear();
         torqueTrajectory.clear();

         orientationTrajectory.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
         orientationTrajectory.appendWaypoint(duration, desiredOrientation, new Vector3D());
         orientationTrajectory.initialize();

         torqueTrajectory.appendWaypoint(0.0, new Point3D(), desiredTorque);
         torqueTrajectory.appendWaypoint(duration, new Point3D(), desiredTorque);
         torqueTrajectory.initialize();


         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();
         contactStateProvider.getTimeInterval().setInterval(0.0, duration);
         List<SettableContactStateProvider> contactStateProviders = new ArrayList<>();
         contactStateProviders.add(contactStateProvider);

         controller.setTrajectories(orientationTrajectory, torqueTrajectory, contactStateProviders);
         Vector3DBasics feedbackTorque = new Vector3D();

         // check yaw only
         controller.compute(0.0, desiredOrientation, yawedVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(-Math.signum(yawVelocity), Math.signum(feedbackTorque.getZ()), 1e-5);

         // check pitch only
         controller.compute(0.0, desiredOrientation, pitchedVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(0.0, feedbackTorque.getX(), 1e-5);
         assertEquals(-Math.signum(pitchVelocity), Math.signum(feedbackTorque.getY()), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);

         // check roll only
         controller.compute(0.0, desiredOrientation, rolledVelocity);

         controller.getDesiredTorque(feedbackTorque);
         assertEquals(-Math.signum(rollVelocity), Math.signum(feedbackTorque.getX()), 1e-5);
         assertEquals(0.0, feedbackTorque.getY(), 1e-5);
         assertEquals(0.0, feedbackTorque.getZ(), 1e-5);
      }
   }
}
