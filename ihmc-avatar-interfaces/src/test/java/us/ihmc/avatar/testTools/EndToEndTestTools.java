package us.ihmc.avatar.testTools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryPointMessage;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSO3TrajectoryPoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class EndToEndTestTools
{
   public static final String FORMAT = EuclidCoreIOTools.getStringFormat(6, 4);

   /**
    * This method will assert the number of waypoints used by a {@link RigidBodyTaskspaceControlState} that is
    * controlling the {@link RigidBody} with given name. The number of waypoints checked is the total number
    * of trajectory points (in queue and in generator) of the controller.
    */
   public static void assertNumberOfPoints(String bodyName, int points, SimulationConstructionSet scs)
   {
      assertEquals("Unexpected number of waypoints:", points, findControllerNumberOfWaypoints(bodyName, scs));
   }

   public static void assertCurrentDesiredsMatchWaypoint(String bodyName, SO3TrajectoryPointMessage waypoint, SimulationConstructionSet scs, double epsilon)
   {
      assertCurrentDesiredsMatch(bodyName, waypoint.orientation, waypoint.angularVelocity, scs, epsilon);
   }

   public static void assertCurrentDesiredsMatch(String bodyName, QuaternionReadOnly expectedOrientation, Vector3D expectedAngularVelocity, SimulationConstructionSet scs, double epsilon)
   {
      Quaternion desiredOrientation = EndToEndTestTools.findControllerDesiredOrientation(bodyName, scs);
      Vector3D desiredAngularVelocity = EndToEndTestTools.findControllerDesiredAngularVelocity(bodyName, scs);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Orientation", expectedOrientation, desiredOrientation, epsilon, FORMAT);
      EuclidCoreTestTools.assertTuple3DEquals("Angular Velocity", expectedAngularVelocity, desiredAngularVelocity, epsilon, FORMAT);
   }

   public static void assertWaypointInGeneratorMatches(String bodyName, int index, SO3TrajectoryPointMessage waypoint, SimulationConstructionSet scs, double epsilon)
   {
      assertTrue("Index too high: " + index, index < RigidBodyTaskspaceControlState.maxPointsInGenerator);
      SimpleSO3TrajectoryPoint actualWaypoint = findOrientationTrajectoryPoint(bodyName, index, scs);
      assertEquals("Time", waypoint.getTime(), actualWaypoint.getTime(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Orientation", waypoint.orientation, actualWaypoint.getOrientationCopy(), epsilon, FORMAT);
      EuclidCoreTestTools.assertTuple3DEquals("Angular Velocity", waypoint.angularVelocity, actualWaypoint.getAngularVelocityCopy(), epsilon, FORMAT);
   }

   public static SimpleSO3TrajectoryPoint findOrientationTrajectoryPoint(String bodyName, int index, SimulationConstructionSet scs)
   {
      String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      String suffix = "AtWaypoint" + index;
      String timeName = bodyName + "Time";
      String orientationName = bodyName + "Orientation";
      String angularVelocityName = bodyName + "AngularVelocity";
      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setTime(scs.getVariable(orientationTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSO3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
      simpleSO3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      return simpleSO3TrajectoryPoint;
   }

   /**
    * Finds the number of waypoints in a {@link RigidBodyTaskspaceControlState} for the body with the
    * given name.
    */
   public static int findControllerNumberOfWaypoints(String bodyName, SimulationConstructionSet scs)
   {
      String namespace = RigidBodyControlState.createRegistryName(bodyName, RigidBodyControlMode.TASKSPACE);
      String variableName = bodyName + "TaskspaceNumberOfPoints";
      return (int) scs.getVariable(namespace, variableName).getValueAsLongBits();
   }

   /**
    * Finds the current desired orientation in the controller for the body with the given name.
    */
   public static Quaternion findControllerDesiredOrientation(String bodyName, SimulationConstructionSet scs)
   {
      return findQuat4d(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.ORIENTATION.getName(), scs);
   }

   /**
    * Finds the quaternion associated with the given namespace and name in scs.
    */
   public static Quaternion findQuat4d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findQuat4d(nameSpace, varname, "", scs);
   }

   /**
    * Finds the quaternion associated with the given namespace, prefix and suffix in scs.
    */
   public static Quaternion findQuat4d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      double x = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQxName(prefix, suffix)).getValueAsDouble();
      double y = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQyName(prefix, suffix)).getValueAsDouble();
      double z = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQzName(prefix, suffix)).getValueAsDouble();
      double s = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQsName(prefix, suffix)).getValueAsDouble();
      return new Quaternion(x, y, z, s);
   }

   public static Vector3D findControllerDesiredAngularVelocity(String bodyName, SimulationConstructionSet scs)
   {
      return findVector3d(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.ANGULAR_VELOCITY.getName(), scs);
   }

   public static Vector3D findVector3d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findVector3d(nameSpace, varname, "", scs);
   }

   public static Vector3D findVector3d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Vector3D(findTuple3d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Tuple3DBasics findTuple3d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      Tuple3DBasics tuple3d = new Point3D();
      tuple3d.setX(scs.getVariable(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix)).getValueAsDouble());
      tuple3d.setY(scs.getVariable(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix)).getValueAsDouble());
      tuple3d.setZ(scs.getVariable(nameSpace, YoFrameVariableNameTools.createZName(prefix, suffix)).getValueAsDouble());
      return tuple3d;
   }
}
