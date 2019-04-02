package us.ihmc.avatar.testTools;

import static org.junit.jupiter.api.Assertions.*;

import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointControlHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class EndToEndTestTools
{
   public static final String FORMAT = EuclidCoreIOTools.getStringFormat(6, 4);

   /**
    * This method will assert the number of waypoints used by a {@link RigidBodyTaskspaceControlState}
    * that is controlling the {@link RigidBodyBasics} with given name. The number of waypoints checked
    * is the total number of trajectory points (in queue and in generator) of the controller.
    */
   public static void assertTotalNumberOfWaypointsInTaskspaceManager(String bodyName, int expectedNumberOfPoints, YoVariableHolder yoVariableHolder)
   {
      assertTotalNumberOfWaypointsInTaskspaceManager(bodyName, "", expectedNumberOfPoints, yoVariableHolder);
   }

   public static void assertTotalNumberOfWaypointsInTaskspaceManager(String bodyName, String postfix, int expectedNumberOfPoints,
                                                                     YoVariableHolder yoVariableHolder)
   {
      assertEquals(expectedNumberOfPoints, findTotalNumberOfWaypointsInTaskspaceManager(bodyName, postfix, yoVariableHolder),
                   "Unexpected number of waypoints:");
   }

   public static void assertCurrentDesiredsMatchWaypoint(String bodyName, SO3TrajectoryPointMessage expectedWaypoint, double epsilon,
                                                         YoVariableHolder yoVariableHolder)
   {
      assertCurrentDesiredsMatch(bodyName, expectedWaypoint.getOrientation(), expectedWaypoint.getAngularVelocity(), epsilon, yoVariableHolder);
   }

   public static void assertCurrentDesiredsMatch(String bodyName, QuaternionReadOnly expectedOrientation, Vector3DReadOnly expectedAngularVelocity,
                                                 double epsilon, YoVariableHolder yoVariableHolder)
   {
      QuaternionReadOnly desiredOrientation = findFeedbackControllerDesiredOrientation(bodyName, yoVariableHolder);
      Vector3DReadOnly desiredAngularVelocity = findFeedbackControllerDesiredAngularVelocity(bodyName, yoVariableHolder);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Orientation", expectedOrientation, desiredOrientation, epsilon, FORMAT);
      EuclidCoreTestTools.assertTuple3DEquals("Angular Velocity", expectedAngularVelocity, desiredAngularVelocity, epsilon, FORMAT);
   }

   public static void assertWaypointInGeneratorMatches(String bodyName, int waypointIndexInController, SO3TrajectoryPointMessage expectedWaypoint,
                                                       double epsilon, YoVariableHolder yoVariableHolder)
   {
      assertTrue(waypointIndexInController < RigidBodyTaskspaceControlState.maxPointsInGenerator, "Index too high: " + waypointIndexInController);
      SO3TrajectoryPoint actualWaypoint = findSO3TrajectoryPoint(bodyName, waypointIndexInController, yoVariableHolder);
      assertEquals(expectedWaypoint.getTime(), actualWaypoint.getTime(), epsilon, "Time");
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Orientation", expectedWaypoint.getOrientation(), actualWaypoint.getOrientationCopy(), epsilon,
                                                              FORMAT);
      EuclidCoreTestTools.assertTuple3DEquals("Angular Velocity", expectedWaypoint.getAngularVelocity(), actualWaypoint.getAngularVelocityCopy(), epsilon,
                                              FORMAT);
   }

   public static void assertOneDoFJointsFeebackControllerDesireds(String[] jointNames, double[] desiredJointPositions, double[] desiredJointVelocities,
                                                                  double epsilon, YoVariableHolder yoVariableHolder)
   {
      for (int jointIndex = 0; jointIndex < jointNames.length; jointIndex++)
      {
         assertOneDoFJointFeedbackControllerDesireds(jointNames[jointIndex], desiredJointPositions[jointIndex], desiredJointVelocities[jointIndex], epsilon,
                                                     yoVariableHolder);
      }
   }

   public static void assertOneDoFJointFeedbackControllerDesireds(String jointName, double desiredPosition, double desiredVelocity, double epsilon,
                                                                  YoVariableHolder yoVariableHolder)
   {
      assertOneDoFJointFeedbackControllerDesiredPosition(jointName, desiredPosition, epsilon, yoVariableHolder);
      assertOneDoFJointFeedbackControllerDesiredVelocity(jointName, desiredVelocity, epsilon, yoVariableHolder);
   }

   public static void assertOneDoFJointFeedbackControllerDesiredPosition(String jointName, double desiredPosition, double epsilon,
                                                                         YoVariableHolder yoVariableHolder)
   {
      YoDouble scsDesiredPosition = findOneDoFJointFeedbackControllerDesiredPosition(jointName, yoVariableHolder);
      assertEquals(desiredPosition, scsDesiredPosition.getDoubleValue(), epsilon);
   }

   public static void assertOneDoFJointFeedbackControllerDesiredVelocity(String jointName, double desiredVelocity, double epsilon,
                                                                         YoVariableHolder yoVariableHolder)
   {
      YoDouble scsDesiredVelocity = findOneDoFJointFeedbackControllerDesiredVelocity(jointName, yoVariableHolder);
      assertEquals(desiredVelocity, scsDesiredVelocity.getDoubleValue(), epsilon);
   }

   public static void assertTotalNumberOfWaypointsInJointspaceManager(int expectedNumberOfTrajectoryPoints, String bodyName, String[] jointNames,
                                                                      YoVariableHolder yoVariableHolder)
   {
      for (String jointName : jointNames)
         assertTotalNumberOfWaypointsInJointspaceManager(expectedNumberOfTrajectoryPoints, bodyName, jointName, yoVariableHolder);
   }

   public static void assertTotalNumberOfWaypointsInJointspaceManager(int expectedNumberOfWaypoints, String bodyName, String jointName,
                                                                      YoVariableHolder yoVariableHolder)
   {
      int numberOfPoints = findTotalNumberOfWaypointsInJointspaceManager(bodyName, jointName, yoVariableHolder);
      assertEquals(expectedNumberOfWaypoints, numberOfPoints, "Unexpected number of trajectory points for " + jointName);
   }

   public static void assertNumberOfWaypointsInJointspaceManagerGenerator(int expectedNumberOfWaypoints, String bodyName, String jointName,
                                                                          YoVariableHolder yoVariableHolder)
   {
      int numberOfPoints = findNumberOfWaypointsInJointspaceManagerGenerator(bodyName, jointName, yoVariableHolder);
      assertEquals(expectedNumberOfWaypoints, numberOfPoints, "Unexpected number of trajectory points for " + jointName);
   }

   public static void assertNumberOfWaypointsInJointspaceManagerQueue(int expectedNumberOfWaypoints, String bodyName, String jointName,
                                                                      YoVariableHolder yoVariableHolder)
   {
      int numberOfPoints = findNumberOfWaypointsInJointspaceManagerQueue(bodyName, jointName, yoVariableHolder);
      assertEquals(expectedNumberOfWaypoints, numberOfPoints, "Unexpected number of trajectory points for " + jointName);
   }

   public static void assertJointspaceTrajectoryStatus(long expectedSequenceID, TrajectoryExecutionStatus expectedStatus, double expectedTimestamp,
                                                       double[] expectedDesiredPositions, String[] jointNames, JointspaceTrajectoryStatusMessage statusMessage,
                                                       double epsilon, double controllerDT)
   {
      assertJointspaceTrajectoryStatus(expectedSequenceID, expectedStatus, expectedTimestamp, jointNames, statusMessage, controllerDT);

      for (int jointIndex = 0; jointIndex < jointNames.length; jointIndex++)
      {
         assertEquals(expectedDesiredPositions[jointIndex], statusMessage.getDesiredJointPositions().get(jointIndex), epsilon);
      }
   }

   public static void assertJointspaceTrajectoryStatus(long expectedSequenceID, TrajectoryExecutionStatus expectedStatus, double expectedTimestamp,
                                                       String[] jointNames, JointspaceTrajectoryStatusMessage statusMessage, double controllerDT)
   {
      assertEquals(expectedSequenceID, statusMessage.getSequenceId());
      assertEquals(expectedStatus, TrajectoryExecutionStatus.fromByte(statusMessage.getTrajectoryExecutionStatus()));
      assertEquals(expectedTimestamp, statusMessage.getTimestamp(), 1.01 * controllerDT); // When queueing messages, the time can drift a tiny bit.
      assertEquals(jointNames.length, statusMessage.getJointNames().size());
      assertEquals(jointNames.length, statusMessage.getActualJointPositions().size());
      assertEquals(jointNames.length, statusMessage.getDesiredJointPositions().size());

      for (int jointIndex = 0; jointIndex < jointNames.length; jointIndex++)
      {
         assertEquals(jointNames[jointIndex], statusMessage.getJointNames().getString(jointIndex));
      }
   }

   public static void assertTaskspaceTrajectoryStatus(long expectedSequenceID, TrajectoryExecutionStatus expectedStatus, double expectedTimestamp,
                                                      Pose3DReadOnly expectedDesiredPose, String endEffectorName,
                                                      TaskspaceTrajectoryStatusMessage statusMessage, double epsilon, double controllerDT)
   {
      assertTaskspaceTrajectoryStatus(expectedSequenceID, expectedStatus, expectedTimestamp, expectedDesiredPose.getPosition(),
                                      expectedDesiredPose.getOrientation(), endEffectorName, statusMessage, epsilon, controllerDT);
   }

   public static void assertTaskspaceTrajectoryStatus(long expectedSequenceID, TrajectoryExecutionStatus expectedStatus, double expectedTimestamp,
                                                      Point3DReadOnly expectedDesiredPosition, Orientation3DReadOnly expectedDesiredOrientation,
                                                      String endEffectorName, TaskspaceTrajectoryStatusMessage statusMessage, double epsilon,
                                                      double controllerDT)
   {
      if (expectedDesiredPosition != null)
      {
         if (!expectedDesiredPosition.containsNaN())
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedDesiredPosition, statusMessage.getDesiredEndEffectorPosition(), epsilon);
            assertFalse(statusMessage.getActualEndEffectorPosition().containsNaN());
         }
         else
         {
            boolean areEqual = true;
            for (int i = 0; i < 3; i++)
            {
               if (!MathTools.epsilonCompare(expectedDesiredPosition.getElement(i), statusMessage.getDesiredEndEffectorPosition().getElement(i), epsilon))
               {
                  areEqual = false;
                  break;
               }
            }

            if (!areEqual)
            {
               fail("expected:\n" + expectedDesiredPosition.toString() + "\n but was:\n" + statusMessage.getDesiredEndEffectorPosition().toString());
            }
            else
            {
               boolean badActual = false;

               for (int i = 0; i < 3; i++)
               {
                  if (Double.isNaN(expectedDesiredPosition.getElement(i)))
                  {
                     if (!Double.isNaN(statusMessage.getActualEndEffectorPosition().getElement(i)))
                     {
                        badActual = true;
                        break;
                     }
                  }
               }

               if (badActual)
               {
                  Point3D expectedActualPosition = new Point3D(statusMessage.getActualEndEffectorPosition());

                  for (int i = 0; i < 3; i++)
                  {
                     if (Double.isNaN(expectedDesiredPosition.getElement(i)))
                        expectedActualPosition.setElement(i, Double.NaN);
                  }
                  fail("expected:\n" + expectedActualPosition.toString() + "\n but was:\n" + statusMessage.getActualEndEffectorPosition().toString());
               }
            }
         }
      }
      else
      {
         assertTrue(statusMessage.getDesiredEndEffectorPosition().containsNaN());
         assertTrue(statusMessage.getActualEndEffectorPosition().containsNaN());
      }

      if (expectedDesiredOrientation != null)
      {
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(new Quaternion(expectedDesiredOrientation), statusMessage.getDesiredEndEffectorOrientation(),
                                                                 epsilon);
         assertFalse(statusMessage.getActualEndEffectorOrientation().containsNaN());
      }
      else
      {
         assertTrue(statusMessage.getDesiredEndEffectorOrientation().containsNaN());
         assertTrue(statusMessage.getActualEndEffectorOrientation().containsNaN());
      }
      assertTaskspaceTrajectoryStatus(expectedSequenceID, expectedStatus, expectedTimestamp, endEffectorName, statusMessage, controllerDT);
   }

   public static void assertTaskspaceTrajectoryStatus(long expectedSequenceID, TrajectoryExecutionStatus expectedStatus, double expectedTimestamp,
                                                      String endEffectorName, TaskspaceTrajectoryStatusMessage statusMessage, double controllerDT)
   {
      assertEquals(expectedSequenceID, statusMessage.getSequenceId());
      assertEquals(expectedStatus, TrajectoryExecutionStatus.fromByte(statusMessage.getTrajectoryExecutionStatus()));
      assertEquals(expectedTimestamp, statusMessage.getTimestamp(), 1.01 * controllerDT); // When queueing messages, the time can drift a tiny bit.
      assertEquals(endEffectorName, statusMessage.getEndEffectorName().toString());

   }

   @SuppressWarnings("unchecked")
   public static RigidBodyControlMode findRigidBodyControlManagerState(String bodyName, YoVariableHolder yoVariableHolder)
   {
      String managerName = bodyName + "Manager";
      return ((YoEnum<RigidBodyControlMode>) yoVariableHolder.getVariable(managerName, managerName + "CurrentState")).getEnumValue();
   }

   public static SO3TrajectoryPoint findSO3TrajectoryPoint(String bodyName, int trajectoryPointIndex, YoVariableHolder yoVariableHolder)
   {
      String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      String suffix = "AtWaypoint" + trajectoryPointIndex;
      String timeName = bodyName + "Time";
      String orientationName = bodyName + "Orientation";
      String angularVelocityName = bodyName + "AngularVelocity";
      SO3TrajectoryPoint simpleSO3TrajectoryPoint = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setTime(yoVariableHolder.getVariable(orientationTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSO3TrajectoryPoint.setOrientation(findQuaternion(orientationTrajectoryName, orientationName, suffix, yoVariableHolder));
      simpleSO3TrajectoryPoint.setAngularVelocity(findVector3D(orientationTrajectoryName, angularVelocityName, suffix, yoVariableHolder));
      return simpleSO3TrajectoryPoint;
   }

   public static SE3TrajectoryPoint findSE3TrajectoryPoint(String bodyName, int trajectoryPointIndex, YoVariableHolder yoVariableHolder)
   {
      String positionTrajectoryName = bodyName + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = bodyName + "Time";
      String positionName = bodyName + "Position";
      String orientationName = bodyName + "Orientation";
      String linearVelocityName = bodyName + "LinearVelocity";
      String angularVelocityName = bodyName + "AngularVelocity";

      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setTime(yoVariableHolder.getVariable(positionTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.setPosition(findPoint3D(positionTrajectoryName, positionName, suffix, yoVariableHolder));
      simpleSE3TrajectoryPoint.setOrientation(findQuaternion(orientationTrajectoryName, orientationName, suffix, yoVariableHolder));
      simpleSE3TrajectoryPoint.setLinearVelocity(findVector3D(positionTrajectoryName, linearVelocityName, suffix, yoVariableHolder));
      simpleSE3TrajectoryPoint.setAngularVelocity(findVector3D(orientationTrajectoryName, angularVelocityName, suffix, yoVariableHolder));
      return simpleSE3TrajectoryPoint;
   }

   public static SE3TrajectoryPoint findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(String bodyName, YoVariableHolder yoVariableHolder)
   {
      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setPosition(findFeedbackControllerDesiredPosition(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.setOrientation(findFeedbackControllerDesiredOrientation(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.setLinearVelocity(findFeedbackControllerDesiredLinearVelocity(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.setAngularVelocity(findFeedbackControllerDesiredAngularVelocity(bodyName, yoVariableHolder));
      return simpleSE3TrajectoryPoint;
   }

   /**
    * Finds the number of waypoints in a {@link RigidBodyTaskspaceControlState} for the body with the
    * given name.
    */
   public static int findTotalNumberOfWaypointsInTaskspaceManager(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findTotalNumberOfWaypointsInTaskspaceManager(bodyName, "", yoVariableHolder);
   }

   public static int findTotalNumberOfWaypointsInTaskspaceManager(String bodyName, String postfix, YoVariableHolder yoVariableHolder)
   {
      String variableName = bodyName + postfix + "TaskspaceNumberOfPoints";
      return (int) yoVariableHolder.getVariable(variableName).getValueAsLongBits();
   }

   public static int findNumberOfWaypointsInTaskspaceManagerGenerator(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findNumberOfWaypointsInTaskspaceManagerGenerator(bodyName, "", yoVariableHolder);
   }

   public static int findNumberOfWaypointsInTaskspaceManagerGenerator(String bodyName, String postfix, YoVariableHolder yoVariableHolder)
   {
      String variableName = bodyName + postfix + "TaskspaceNumberOfPointsInGenerator";
      return (int) yoVariableHolder.getVariable(variableName).getValueAsLongBits();
   }

   /**
    * Finds the number of waypoints in a {@link RigidBodyJointspaceControlState} for the body & joint
    * with the given name.
    */
   public static int findTotalNumberOfWaypointsInJointspaceManager(String bodyName, String jointName, YoVariableHolder yoVariableHolder)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + jointName + "_numberOfPoints";
      return ((YoInteger) yoVariableHolder.getVariable(namespace, variable)).getIntegerValue();
   }

   public static int findNumberOfWaypointsInJointspaceManagerGenerator(String bodyName, String jointName, YoVariableHolder yoVariableHolder)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + jointName + "_numberOfPointsInGenerator";
      return ((YoInteger) yoVariableHolder.getVariable(namespace, variable)).getIntegerValue();

   }

   public static int findNumberOfWaypointsInJointspaceManagerQueue(String bodyName, String jointName, YoVariableHolder yoVariableHolder)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + jointName + "_numberOfPointsInQueue";
      return ((YoInteger) yoVariableHolder.getVariable(namespace, variable)).getIntegerValue();

   }

   public static Point3DReadOnly findFeedbackControllerDesiredPosition(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint3D(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.POSITION.getName(),
                                yoVariableHolder);
   }

   /**
    * Finds the current desired orientation in the controller for the body with the given name.
    */
   public static QuaternionReadOnly findFeedbackControllerDesiredOrientation(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameQuaternion(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.ORIENTATION.getName(),
                                   yoVariableHolder);
   }

   public static Vector3DReadOnly findFeedbackControllerDesiredLinearVelocity(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector3D(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.LINEAR_VELOCITY.getName(),
                                 yoVariableHolder);
   }

   public static Vector3DReadOnly findFeedbackControllerDesiredAngularVelocity(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector3D(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.ANGULAR_VELOCITY.getName(),
                                 yoVariableHolder);
   }

   public static YoDouble findOneDoFJointFeedbackControllerDesiredPosition(String jointName, YoVariableHolder yoVariableHolder)
   {
      String nameSpace = jointName + OneDoFJointFeedbackController.shortName;
      String variable = "q_d_" + jointName;
      return findYoDouble(nameSpace, variable, yoVariableHolder);
   }

   public static YoDouble findOneDoFJointFeedbackControllerDesiredVelocity(String jointName, YoVariableHolder yoVariableHolder)
   {
      String nameSpace = jointName + OneDoFJointFeedbackController.shortName;
      String variable = "qd_d_" + jointName;
      return findYoDouble(nameSpace, variable, yoVariableHolder);
   }

   /**
    * Finds the quaternion associated with the given namespace and name in scs.
    */
   public static Quaternion findQuaternion(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findQuaternion(nameSpace, varname, "", yoVariableHolder);
   }

   /**
    * Finds the quaternion associated with the given namespace, prefix and suffix in scs.
    */
   public static Quaternion findQuaternion(String nameSpace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      return new Quaternion(findYoFrameQuaternion(nameSpace, prefix, suffix, yoVariableHolder));
   }

   public static YoFrameQuaternion findYoFrameQuaternion(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameQuaternion(nameSpace, varname, "", yoVariableHolder);
   }

   public static YoFrameQuaternion findYoFrameQuaternion(String nameSpace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble qx = findYoDouble(nameSpace, YoFrameVariableNameTools.createQxName(prefix, suffix), yoVariableHolder);
      YoDouble qy = findYoDouble(nameSpace, YoFrameVariableNameTools.createQyName(prefix, suffix), yoVariableHolder);
      YoDouble qz = findYoDouble(nameSpace, YoFrameVariableNameTools.createQzName(prefix, suffix), yoVariableHolder);
      YoDouble qs = findYoDouble(nameSpace, YoFrameVariableNameTools.createQsName(prefix, suffix), yoVariableHolder);
      return new YoFrameQuaternion(qx, qy, qz, qs, ReferenceFrame.getWorldFrame());
   }

   public static Vector2D findVector2D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findVector2D(nameSpace, varname, "", yoVariableHolder);
   }

   public static Vector2D findVector2D(String nameSpace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Vector2D(findYoFramePoint2D(nameSpace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static Point2D findPoint2D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findPoint2D(nameSpace, varname, "", yoVariableHolder);
   }

   public static Point2D findPoint2D(String nameSpace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Point2D(findYoFramePoint2D(nameSpace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static Vector3D findVector3D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findVector3D(nameSpace, varname, "", yoVariableHolder);
   }

   public static Vector3D findVector3D(String nameSpace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Vector3D(findYoFramePoint3D(nameSpace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static Point3D findPoint3D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findPoint3D(nameSpace, varname, "", yoVariableHolder);
   }

   public static Point3D findPoint3D(String nameSpace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Point3D(findYoFramePoint3D(nameSpace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static YoFramePoint2D findYoFramePoint2D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint2D(nameSpace, varname, "", yoVariableHolder);
   }

   public static YoFramePoint2D findYoFramePoint2D(String nameSpace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix), yoVariableHolder);
      return new YoFramePoint2D(x, y, ReferenceFrame.getWorldFrame());
   }

   public static YoFrameVector2D findYoFrameVector2D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector2D(nameSpace, varname, "", yoVariableHolder);
   }

   public static YoFrameVector2D findYoFrameVector2D(String nameSpace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix), yoVariableHolder);
      return new YoFrameVector2D(x, y, ReferenceFrame.getWorldFrame());
   }

   public static YoFramePoint3D findYoFramePoint3D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint3D(nameSpace, varname, "", yoVariableHolder);
   }

   public static YoFramePoint3D findYoFramePoint3D(String nameSpace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix), yoVariableHolder);
      YoDouble z = findYoDouble(nameSpace, YoFrameVariableNameTools.createZName(prefix, suffix), yoVariableHolder);
      return new YoFramePoint3D(x, y, z, ReferenceFrame.getWorldFrame());
   }

   public static YoFrameVector3D findYoFrameVector3D(String nameSpace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector3D(nameSpace, varname, "", yoVariableHolder);
   }

   public static YoFrameVector3D findYoFrameVector3D(String nameSpace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix), yoVariableHolder);
      YoDouble z = findYoDouble(nameSpace, YoFrameVariableNameTools.createZName(prefix, suffix), yoVariableHolder);
      return new YoFrameVector3D(x, y, z, ReferenceFrame.getWorldFrame());
   }

   public static YoDouble findYoDouble(String namespace, String name, YoVariableHolder yoVariableHolder)
   {
      return findYoVariable(namespace, name, YoDouble.class, yoVariableHolder);
   }

   public static YoInteger findYoInteger(String namespace, String name, YoVariableHolder yoVariableHolder)
   {
      return findYoVariable(namespace, name, YoInteger.class, yoVariableHolder);
   }

   public static YoBoolean findYoBoolean(String namespace, String name, YoVariableHolder yoVariableHolder)
   {
      return findYoVariable(namespace, name, YoBoolean.class, yoVariableHolder);
   }

   public static <T extends YoVariable<T>> T findYoVariable(String namespace, String name, Class<T> clazz, YoVariableHolder yoVariableHolder)
   {
      YoVariable<?> uncheckedVariable = yoVariableHolder.getVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return clazz.cast(uncheckedVariable);
   }
}
