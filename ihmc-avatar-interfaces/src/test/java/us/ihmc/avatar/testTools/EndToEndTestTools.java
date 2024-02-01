package us.ihmc.avatar.testTools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointControlHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.yoVariables.multiBodySystem.interfaces.YoOneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.simulationConstructionSetTools.dataExporter.DataExporterExcelWorkbookCreator;
import us.ihmc.simulationConstructionSetTools.dataExporter.TorqueSpeedDataExporterGraphCreator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
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

   public static void assertTotalNumberOfWaypointsInTaskspaceManager(String bodyName,
                                                                     String postfix,
                                                                     int expectedNumberOfPoints,
                                                                     YoVariableHolder yoVariableHolder)
   {
      assertEquals(expectedNumberOfPoints,
                   findTotalNumberOfWaypointsInTaskspaceManager(bodyName, postfix, yoVariableHolder),
                   "Unexpected number of waypoints:");
   }

   public static void assertCurrentDesiredsMatchWaypoint(String bodyName,
                                                         SO3TrajectoryPointMessage expectedWaypoint,
                                                         double epsilon,
                                                         YoVariableHolder yoVariableHolder)
   {
      assertCurrentDesiredsMatch(bodyName, expectedWaypoint.getOrientation(), expectedWaypoint.getAngularVelocity(), epsilon, yoVariableHolder);
   }

   public static void assertCurrentDesiredsMatch(String bodyName,
                                                 QuaternionReadOnly expectedOrientation,
                                                 Vector3DReadOnly expectedAngularVelocity,
                                                 double epsilon,
                                                 YoVariableHolder yoVariableHolder)
   {
      QuaternionReadOnly desiredOrientation = findFeedbackControllerDesiredOrientation(bodyName, yoVariableHolder);
      Vector3DReadOnly desiredAngularVelocity = findFeedbackControllerDesiredAngularVelocity(bodyName, yoVariableHolder);
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals("Orientation", expectedOrientation, desiredOrientation, epsilon, FORMAT);
      EuclidCoreTestTools.assertEquals("Angular Velocity", expectedAngularVelocity, desiredAngularVelocity, epsilon, FORMAT);
   }

   public static void assertWaypointInGeneratorMatches(String bodyName,
                                                       int waypointIndexInController,
                                                       SO3TrajectoryPointMessage expectedWaypoint,
                                                       double epsilon,
                                                       YoVariableHolder yoVariableHolder)
   {
      assertTrue(waypointIndexInController < RigidBodyTaskspaceControlState.maxPointsInGenerator, "Index too high: " + waypointIndexInController);
      SO3TrajectoryPoint actualWaypoint = findSO3TrajectoryPoint(bodyName, waypointIndexInController, yoVariableHolder);
      assertEquals(expectedWaypoint.getTime(), actualWaypoint.getTime(), epsilon, "Time");
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals("Orientation",
                                                                 expectedWaypoint.getOrientation(),
                                                                 actualWaypoint.getOrientation(),
                                                                 epsilon,
                                                                 FORMAT);
      EuclidCoreTestTools.assertEquals("Angular Velocity", expectedWaypoint.getAngularVelocity(), actualWaypoint.getAngularVelocity(), epsilon, FORMAT);
   }

   public static void assertOneDoFJointsFeebackControllerDesireds(String[] jointNames,
                                                                  double[] desiredJointPositions,
                                                                  double[] desiredJointVelocities,
                                                                  double epsilon,
                                                                  YoVariableHolder yoVariableHolder)
   {
      for (int jointIndex = 0; jointIndex < jointNames.length; jointIndex++)
      {
         assertOneDoFJointFeedbackControllerDesireds(jointNames[jointIndex],
                                                     desiredJointPositions[jointIndex],
                                                     desiredJointVelocities[jointIndex],
                                                     epsilon,
                                                     yoVariableHolder);
      }
   }

   public static void assertOneDoFJointFeedbackControllerDesireds(String jointName,
                                                                  double desiredPosition,
                                                                  double desiredVelocity,
                                                                  double epsilon,
                                                                  YoVariableHolder yoVariableHolder)
   {
      assertOneDoFJointFeedbackControllerDesiredPosition(jointName, desiredPosition, epsilon, yoVariableHolder);
      assertOneDoFJointFeedbackControllerDesiredVelocity(jointName, desiredVelocity, epsilon, yoVariableHolder);
   }

   public static void assertOneDoFJointFeedbackControllerDesiredPosition(String jointName,
                                                                         double desiredPosition,
                                                                         double epsilon,
                                                                         YoVariableHolder yoVariableHolder)
   {
      YoDouble scsDesiredPosition = findOneDoFJointFeedbackControllerDesiredPosition(jointName, yoVariableHolder);
      assertEquals(desiredPosition, scsDesiredPosition.getDoubleValue(), epsilon);
   }

   public static void assertOneDoFJointFeedbackControllerDesiredVelocity(String jointName,
                                                                         double desiredVelocity,
                                                                         double epsilon,
                                                                         YoVariableHolder yoVariableHolder)
   {
      YoDouble scsDesiredVelocity = findOneDoFJointFeedbackControllerDesiredVelocity(jointName, yoVariableHolder);
      assertEquals(desiredVelocity, scsDesiredVelocity.getDoubleValue(), epsilon);
   }

   public static void assertTotalNumberOfWaypointsInJointspaceManager(int expectedNumberOfTrajectoryPoints,
                                                                      String bodyName,
                                                                      String[] jointNames,
                                                                      YoVariableHolder yoVariableHolder)
   {
      for (String jointName : jointNames)
         assertTotalNumberOfWaypointsInJointspaceManager(expectedNumberOfTrajectoryPoints, bodyName, jointName, yoVariableHolder);
   }

   public static void assertTotalNumberOfWaypointsInJointspaceManager(int expectedNumberOfWaypoints,
                                                                      String bodyName,
                                                                      String jointName,
                                                                      YoVariableHolder yoVariableHolder)
   {
      int numberOfPoints = findTotalNumberOfWaypointsInJointspaceManager(bodyName, jointName, yoVariableHolder);
      assertEquals(expectedNumberOfWaypoints, numberOfPoints, "Unexpected number of trajectory points for " + jointName);
   }

   public static void assertNumberOfWaypointsInJointspaceManagerGenerator(int expectedNumberOfWaypoints,
                                                                          String bodyName,
                                                                          String jointName,
                                                                          YoVariableHolder yoVariableHolder)
   {
      int numberOfPoints = findNumberOfWaypointsInJointspaceManagerGenerator(bodyName, jointName, yoVariableHolder);
      assertEquals(expectedNumberOfWaypoints, numberOfPoints, "Unexpected number of trajectory points for " + jointName);
   }

   public static void assertNumberOfWaypointsInJointspaceManagerQueue(int expectedNumberOfWaypoints,
                                                                      String bodyName,
                                                                      String jointName,
                                                                      YoVariableHolder yoVariableHolder)
   {
      int numberOfPoints = findNumberOfWaypointsInJointspaceManagerQueue(bodyName, jointName, yoVariableHolder);
      assertEquals(expectedNumberOfWaypoints, numberOfPoints, "Unexpected number of trajectory points for " + jointName);
   }

   public static void assertJointspaceTrajectoryStatus(long expectedSequenceID,
                                                       TrajectoryExecutionStatus expectedStatus,
                                                       double expectedTimestamp,
                                                       double[] expectedDesiredPositions,
                                                       String[] jointNames,
                                                       JointspaceTrajectoryStatusMessage statusMessage,
                                                       double epsilon,
                                                       double controllerDT)
   {
      assertJointspaceTrajectoryStatus(expectedSequenceID, expectedStatus, expectedTimestamp, jointNames, statusMessage, controllerDT);

      for (int jointIndex = 0; jointIndex < jointNames.length; jointIndex++)
      {
         assertEquals(expectedDesiredPositions[jointIndex], statusMessage.getDesiredJointPositions().get(jointIndex), epsilon);
      }
   }

   public static void assertJointspaceTrajectoryStatus(long expectedSequenceID,
                                                       TrajectoryExecutionStatus expectedStatus,
                                                       double expectedTimestamp,
                                                       String[] jointNames,
                                                       JointspaceTrajectoryStatusMessage statusMessage,
                                                       double controllerDT)
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

   public static void assertTaskspaceTrajectoryStatus(long expectedSequenceID,
                                                      TrajectoryExecutionStatus expectedStatus,
                                                      double expectedTimestamp,
                                                      Pose3DReadOnly expectedDesiredPose,
                                                      String endEffectorName,
                                                      TaskspaceTrajectoryStatusMessage statusMessage,
                                                      double epsilon,
                                                      double controllerDT)
   {
      assertTaskspaceTrajectoryStatus(expectedSequenceID,
                                      expectedStatus,
                                      expectedTimestamp,
                                      expectedDesiredPose.getPosition(),
                                      expectedDesiredPose.getOrientation(),
                                      endEffectorName,
                                      statusMessage,
                                      epsilon,
                                      controllerDT);
   }

   public static void assertTaskspaceTrajectoryStatus(long expectedSequenceID,
                                                      TrajectoryExecutionStatus expectedStatus,
                                                      double expectedTimestamp,
                                                      Point3DReadOnly expectedDesiredPosition,
                                                      Orientation3DReadOnly expectedDesiredOrientation,
                                                      String endEffectorName,
                                                      TaskspaceTrajectoryStatusMessage statusMessage,
                                                      double epsilon,
                                                      double controllerDT)
   {
      if (expectedDesiredPosition != null)
      {
         if (!expectedDesiredPosition.containsNaN())
         {
            EuclidCoreTestTools.assertEquals(expectedDesiredPosition, statusMessage.getDesiredEndEffectorPosition(), epsilon);
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
         EuclidCoreTestTools.assertEquals(expectedDesiredOrientation, statusMessage.getDesiredEndEffectorOrientation(), epsilon);
         assertFalse(statusMessage.getActualEndEffectorOrientation().containsNaN());
      }
      else
      {
         assertTrue(statusMessage.getDesiredEndEffectorOrientation().containsNaN());
         assertTrue(statusMessage.getActualEndEffectorOrientation().containsNaN());
      }
      assertTaskspaceTrajectoryStatus(expectedSequenceID, expectedStatus, expectedTimestamp, endEffectorName, statusMessage, controllerDT);
   }

   public static void assertTaskspaceTrajectoryStatus(long expectedSequenceID,
                                                      TrajectoryExecutionStatus expectedStatus,
                                                      double expectedTimestamp,
                                                      String endEffectorName,
                                                      TaskspaceTrajectoryStatusMessage statusMessage,
                                                      double controllerDT)
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
      return ((YoEnum<RigidBodyControlMode>) yoVariableHolder.findVariable(managerName, managerName + "CurrentState")).getEnumValue();
   }

   public static SO3TrajectoryPoint findSO3TrajectoryPoint(String bodyName, int trajectoryPointIndex, YoVariableHolder yoVariableHolder)
   {
      String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      String suffix = "AtWaypoint" + trajectoryPointIndex;
      String timeName = bodyName + "Time";
      String orientationName = bodyName + "Orientation";
      String angularVelocityName = bodyName + "AngularVelocity";
      SO3TrajectoryPoint simpleSO3TrajectoryPoint = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setTime(yoVariableHolder.findVariable(orientationTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSO3TrajectoryPoint.getOrientation()
                              .set((Orientation3DReadOnly) findQuaternion(orientationTrajectoryName, orientationName, suffix, yoVariableHolder));
      simpleSO3TrajectoryPoint.getAngularVelocity().set(findVector3D(orientationTrajectoryName, angularVelocityName, suffix, yoVariableHolder));
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
      simpleSE3TrajectoryPoint.setTime(yoVariableHolder.findVariable(positionTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.getPosition().set(findPoint3D(positionTrajectoryName, positionName, suffix, yoVariableHolder));
      simpleSE3TrajectoryPoint.getOrientation().set(findQuaternion(orientationTrajectoryName, orientationName, suffix, yoVariableHolder));
      simpleSE3TrajectoryPoint.getLinearVelocity().set(findVector3D(positionTrajectoryName, linearVelocityName, suffix, yoVariableHolder));
      simpleSE3TrajectoryPoint.getAngularVelocity().set(findVector3D(orientationTrajectoryName, angularVelocityName, suffix, yoVariableHolder));
      return simpleSE3TrajectoryPoint;
   }

   public static SE3TrajectoryPoint findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(String bodyName, YoVariableHolder yoVariableHolder)
   {
      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.getPosition().set(findFeedbackControllerDesiredPosition(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) findFeedbackControllerDesiredOrientation(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.getLinearVelocity().set(findFeedbackControllerDesiredLinearVelocity(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.getAngularVelocity().set(findFeedbackControllerDesiredAngularVelocity(bodyName, yoVariableHolder));
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
      return (int) yoVariableHolder.findVariable(variableName).getValueAsLongBits();
   }

   public static int findNumberOfWaypointsInTaskspaceManagerGenerator(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findNumberOfWaypointsInTaskspaceManagerGenerator(bodyName, "", yoVariableHolder);
   }

   public static int findNumberOfWaypointsInTaskspaceManagerGenerator(String bodyName, String postfix, YoVariableHolder yoVariableHolder)
   {
      String variableName = bodyName + postfix + "TaskspaceNumberOfPointsInGenerator";
      return (int) yoVariableHolder.findVariable(variableName).getValueAsLongBits();
   }

   /**
    * Finds the number of waypoints in a {@link RigidBodyJointspaceControlState} for the body & joint
    * with the given name.
    */
   public static int findTotalNumberOfWaypointsInJointspaceManager(String bodyName, String jointName, YoVariableHolder yoVariableHolder)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + jointName + "_numberOfPoints";
      return ((YoInteger) yoVariableHolder.findVariable(namespace, variable)).getIntegerValue();
   }

   public static int findNumberOfWaypointsInJointspaceManagerGenerator(String bodyName, String jointName, YoVariableHolder yoVariableHolder)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + jointName + "_numberOfPointsInGenerator";
      return ((YoInteger) yoVariableHolder.findVariable(namespace, variable)).getIntegerValue();

   }

   public static int findNumberOfWaypointsInJointspaceManagerQueue(String bodyName, String jointName, YoVariableHolder yoVariableHolder)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + jointName + "_numberOfPointsInQueue";
      return ((YoInteger) yoVariableHolder.findVariable(namespace, variable)).getIntegerValue();

   }

   public static Point3DReadOnly findFeedbackControllerDesiredPosition(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint3D(FeedbackControllerToolbox.class.getSimpleName(),
                                bodyName + Type.DESIRED.getName() + SpaceData3D.POSITION.getName(),
                                yoVariableHolder);
   }

   /**
    * Finds the current desired orientation in the controller for the body with the given name.
    */
   public static QuaternionReadOnly findFeedbackControllerDesiredOrientation(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameQuaternion(FeedbackControllerToolbox.class.getSimpleName(),
                                   bodyName + Type.DESIRED.getName() + SpaceData3D.ORIENTATION.getName(),
                                   yoVariableHolder);
   }

   public static Vector3DReadOnly findFeedbackControllerDesiredLinearVelocity(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector3D(FeedbackControllerToolbox.class.getSimpleName(),
                                 bodyName + Type.DESIRED.getName() + SpaceData3D.LINEAR_VELOCITY.getName(),
                                 yoVariableHolder);
   }

   public static Vector3DReadOnly findFeedbackControllerDesiredAngularVelocity(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector3D(FeedbackControllerToolbox.class.getSimpleName(),
                                 bodyName + Type.DESIRED.getName() + SpaceData3D.ANGULAR_VELOCITY.getName(),
                                 yoVariableHolder);
   }

   public static YoDouble findOneDoFJointFeedbackControllerDesiredPosition(String jointName, YoVariableHolder yoVariableHolder)
   {
      String variable = "q_d_" + jointName;
      return findYoDouble(FeedbackControllerToolbox.class.getSimpleName(), variable, yoVariableHolder);
   }

   public static YoDouble findOneDoFJointFeedbackControllerDesiredVelocity(String jointName, YoVariableHolder yoVariableHolder)
   {
      String variable = "qd_d_" + jointName;
      return findYoDouble(FeedbackControllerToolbox.class.getSimpleName(), variable, yoVariableHolder);
   }

   /**
    * Finds the quaternion associated with the given namespace and name in scs.
    */
   public static Quaternion findQuaternion(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findQuaternion(namespace, varname, "", yoVariableHolder);
   }

   /**
    * Finds the quaternion associated with the given namespace, prefix and suffix in scs.
    */
   public static Quaternion findQuaternion(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      return new Quaternion(findYoFrameQuaternion(namespace, prefix, suffix, yoVariableHolder));
   }

   public static YoFrameQuaternion findYoFrameQuaternion(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameQuaternion(namespace, varname, "", yoVariableHolder);
   }

   public static YoFrameQuaternion findYoFrameQuaternion(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble qx = findYoDouble(namespace, YoGeometryNameTools.createQxName(prefix, suffix), yoVariableHolder);
      YoDouble qy = findYoDouble(namespace, YoGeometryNameTools.createQyName(prefix, suffix), yoVariableHolder);
      YoDouble qz = findYoDouble(namespace, YoGeometryNameTools.createQzName(prefix, suffix), yoVariableHolder);
      YoDouble qs = findYoDouble(namespace, YoGeometryNameTools.createQsName(prefix, suffix), yoVariableHolder);
      return new YoFrameQuaternion(qx, qy, qz, qs, ReferenceFrame.getWorldFrame());
   }

   public static Vector2D findVector2D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findVector2D(namespace, varname, "", yoVariableHolder);
   }

   public static Vector2D findVector2D(String namespace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Vector2D(findYoFramePoint2D(namespace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static Point2D findPoint2D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findPoint2D(namespace, varname, "", yoVariableHolder);
   }

   public static Point2D findPoint2D(String namespace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Point2D(findYoFramePoint2D(namespace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static Vector3D findVector3D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findVector3D(namespace, varname, "", yoVariableHolder);
   }

   public static Vector3D findVector3D(String namespace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Vector3D(findYoFramePoint3D(namespace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static Point3D findPoint3D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findPoint3D(namespace, varname, "", yoVariableHolder);
   }

   public static Point3D findPoint3D(String namespace, String varnamePrefix, String varnameSuffix, YoVariableHolder yoVariableHolder)
   {
      return new Point3D(findYoFramePoint3D(namespace, varnamePrefix, varnameSuffix, yoVariableHolder));
   }

   public static YoFramePoint2D findYoFramePoint2D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint2D(namespace, varname, "", yoVariableHolder);
   }

   public static YoFramePoint2D findYoFramePoint2D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      return new YoFramePoint2D(x, y, ReferenceFrame.getWorldFrame());
   }

   public static YoFrameVector2D findYoFrameVector2D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector2D(namespace, varname, "", yoVariableHolder);
   }

   public static YoFrameVector2D findYoFrameVector2D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      return new YoFrameVector2D(x, y, ReferenceFrame.getWorldFrame());
   }

   public static YoFramePoint3D findYoFramePoint3D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFramePoint3D(namespace, varname, "", yoVariableHolder);
   }

   public static YoFramePoint3D findYoFramePoint3D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      YoDouble z = findYoDouble(namespace, YoGeometryNameTools.createZName(prefix, suffix), yoVariableHolder);
      return new YoFramePoint3D(x, y, z, ReferenceFrame.getWorldFrame());
   }

   public static YoFrameVector3D findYoFrameVector3D(String namespace, String varname, YoVariableHolder yoVariableHolder)
   {
      return findYoFrameVector3D(namespace, varname, "", yoVariableHolder);
   }

   public static YoFrameVector3D findYoFrameVector3D(String namespace, String prefix, String suffix, YoVariableHolder yoVariableHolder)
   {
      YoDouble x = findYoDouble(namespace, YoGeometryNameTools.createXName(prefix, suffix), yoVariableHolder);
      YoDouble y = findYoDouble(namespace, YoGeometryNameTools.createYName(prefix, suffix), yoVariableHolder);
      YoDouble z = findYoDouble(namespace, YoGeometryNameTools.createZName(prefix, suffix), yoVariableHolder);
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

   public static <T extends YoVariable> T findYoVariable(String namespace, String name, Class<T> clazz, YoVariableHolder yoVariableHolder)
   {
      YoVariable uncheckedVariable = yoVariableHolder.findVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return clazz.cast(uncheckedVariable);
   }

   public static FootstepDataListMessage generateStepsInPlace(FullHumanoidRobotModel fullRobotModel, int numberOfSteps)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      SideDependentList<FramePose3D> stepPositions = new SideDependentList<>(new FramePose3D(), new FramePose3D());
      stepPositions.forEach((side, position) -> position.setFromReferenceFrame(fullRobotModel.getSoleFrame(side)));

      RobotSide side = RobotSide.LEFT;

      for (int i = 0; i < numberOfSteps; i++)
      {
         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, stepPositions.get(side));
         footstepDataListMessage.getFootstepDataList().add().set(footstepDataMessage);
         side = side.getOppositeSide();
      }

      return footstepDataListMessage;
   }

   public static FootstepDataListMessage generateForwardSteps(RobotSide initialStepSide,
                                                              int numberOfSteps,
                                                              double stepLength,
                                                              double stepWidth,
                                                              double swingTime,
                                                              double transferTime,
                                                              Pose3DReadOnly startPose,
                                                              boolean squareUp)
   {
      return generateForwardSteps(initialStepSide, numberOfSteps, i -> stepLength, stepWidth, swingTime, transferTime, startPose, squareUp);
   }

   public static FootstepDataListMessage generateForwardSteps(RobotSide initialStepSide,
                                                              int numberOfSteps,
                                                              DoubleUnaryOperator stepLengthFunction,
                                                              double stepWidth,
                                                              double swingTime,
                                                              double transferTime,
                                                              Pose3DReadOnly startPose,
                                                              boolean squareUp)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage footstep = message.getFootstepDataList().add();

      RobotSide stepSide = initialStepSide;
      Pose3D stepPose = new Pose3D(startPose);
      stepPose.appendTranslation(0.5 * stepLengthFunction.applyAsDouble(0.0), stepSide.negateIfRightSide(0.5 * stepWidth), 0.0);
      footstep.setRobotSide(stepSide.toByte());
      footstep.getLocation().set(stepPose.getPosition());
      footstep.getOrientation().set(stepPose.getOrientation());
      footstep.setSwingDuration(swingTime);

      for (int i = 1; i < numberOfSteps; i++)
      {
         stepSide = stepSide.getOppositeSide();
         stepPose.appendTranslation(stepLengthFunction.applyAsDouble(i / (numberOfSteps - 1.0)), stepSide.negateIfRightSide(stepWidth), 0.0);
         footstep = message.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
         footstep.setTransferDuration(transferTime);
         footstep.setSwingDuration(swingTime);
      }

      if (squareUp)
      {
         stepSide = stepSide.getOppositeSide();
         stepPose.appendTranslation(0.0, stepSide.negateIfRightSide(stepWidth), 0.0);
         footstep = message.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
         footstep.setTransferDuration(transferTime);
         footstep.setSwingDuration(swingTime);
      }

      return message;
   }

   public static DoubleUnaryOperator trapezoidFunction(double bottomValue, double plateauValue, double startPlateau, double endPlateau)
   {
      return percent ->
      {
         if (percent < startPlateau)
            return EuclidCoreTools.interpolate(bottomValue, plateauValue, percent / startPlateau);
         else if (percent > endPlateau)
            return EuclidCoreTools.interpolate(plateauValue, bottomValue, (percent - endPlateau) / (1.0 - endPlateau));
         else
            return plateauValue;
      };
   }

   public static double computeWalkingDuration(FootstepDataListMessage footsteps, WalkingControllerParameters walkingControllerParameters)
   {
      double defaultSwingDuration = getDuration(footsteps.getDefaultSwingDuration(), walkingControllerParameters.getDefaultSwingTime());
      double defaultTransferDuration = getDuration(footsteps.getDefaultTransferDuration(), walkingControllerParameters.getDefaultTransferTime());
      double defaultInitialTransferDuration = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferDuration = walkingControllerParameters.getDefaultFinalTransferTime();

      double walkingDuration = 0.0;

      for (int i = 0; i < footsteps.getFootstepDataList().size(); i++)
      {
         if (i == 0)
            walkingDuration += computeStepDuration(footsteps.getFootstepDataList().get(i), defaultSwingDuration, defaultInitialTransferDuration);
         else
            walkingDuration += computeStepDuration(footsteps.getFootstepDataList().get(i), defaultSwingDuration, defaultTransferDuration);
      }

      walkingDuration += getDuration(footsteps.getFinalTransferDuration(), defaultFinalTransferDuration);

      return walkingDuration;
   }

   public static double computeStepDuration(FootstepDataMessage footstep, double defaultSwingDuration, double defaultTransferDuration)
   {
      return getDuration(footstep.getSwingDuration(), defaultSwingDuration) + getDuration(footstep.getTransferDuration(), defaultTransferDuration);
   }

   private static double getDuration(double duration, double defaultDuration)
   {
      if (duration <= 0.0 || !Double.isFinite(duration))
         return defaultDuration;
      else
         return duration;
   }

   public static FootstepDataListMessage setStepDurations(FootstepDataListMessage message, double swingDuration, double transferDuration)
   {
      if (Double.isFinite(swingDuration) && swingDuration > 0.0)
      {
         for (int i = 0; i < message.getFootstepDataList().size(); i++)
         {
            message.getFootstepDataList().get(i).setSwingDuration(swingDuration);
         }
      }

      if (Double.isFinite(transferDuration) && transferDuration > 0.0)
      {
         for (int i = 0; i < message.getFootstepDataList().size(); i++)
         {
            message.getFootstepDataList().get(i).setTransferDuration(transferDuration);
         }
      }

      return message;
   }

   public static FootstepDataListMessage generateFootstepsFromPose3Ds(RobotSide initialStepSide, Pose3DReadOnly[] footstepPoses)
   {
      return generateFootstepsFromPose3Ds(initialStepSide, footstepPoses, 0, 0);
   }

   public static FootstepDataListMessage generateFootstepsFromPose3Ds(RobotSide initialStepSide,
                                                                      Pose3DReadOnly[] footstepPoses,
                                                                      double swingTime,
                                                                      double transferTime)
   {
      return generateFootstepsFromPose3Ds(initialStepSide, Arrays.asList(footstepPoses), swingTime, transferTime);
   }

   public static FootstepDataListMessage generateFootstepsFromPose3Ds(RobotSide initialStepSide, List<? extends Pose3DReadOnly> footstepPoses)
   {
      return generateFootstepsFromPose3Ds(initialStepSide, footstepPoses, 0, 0);
   }

   public static FootstepDataListMessage generateFootstepsFromPose3Ds(RobotSide initialStepSide,
                                                                      List<? extends Pose3DReadOnly> footstepPoses,
                                                                      double swingTime,
                                                                      double transferTime)
   {
      FootstepDataListMessage footstepDataList = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      RobotSide side = initialStepSide;

      Object<FootstepDataMessage> list = footstepDataList.getFootstepDataList();

      for (int i = 0; i < footstepPoses.size(); i++)
      {
         list.add().set(HumanoidMessageTools.createFootstepDataMessage(side, footstepPoses.get(i)));
         side = side.getOppositeSide();
      }

      return footstepDataList;
   }

   public static void writeJointStatesMatlab(SCS2AvatarTestingSimulation simulationTestHelper, File destination) throws IOException
   {
      Robot robot = simulationTestHelper.getRobot();

      Set<YoVariable> jointStateVariables = new HashSet<>();

      for (SimJointBasics joint : robot.getAllJoints())
      {
         if (joint instanceof YoOneDoFJointBasics)
         {
            YoOneDoFJointBasics yoOneDoFJoint = (YoOneDoFJointBasics) joint;
            jointStateVariables.add(yoOneDoFJoint.getYoQ());
            jointStateVariables.add(yoOneDoFJoint.getYoQd());
            jointStateVariables.add(yoOneDoFJoint.getYoTau());
         }
      }

      simulationTestHelper.getSimulationConstructionSet().getBuffer()
                          .exportDataMatlab(destination, var -> jointStateVariables.contains(var), reg -> reg == robot.getRegistry());
   }

   public static void exportTorqueSpeedCurves(SCS2AvatarTestingSimulation simulationTestHelper, File dataParentFolder, String dataNameSuffix)
   {
      exportTorqueSpeedCurves(simulationTestHelper, dataParentFolder, dataNameSuffix, null);
   }

   // Pattern-matched from TorqueSpeedDataExporter
   public static void exportTorqueSpeedCurves(SCS2AvatarTestingSimulation simulationTestHelper, File dataParentFolder, String dataNameSuffix, String info)
   {
      YoDouble time = simulationTestHelper.getSimulationConstructionSet().getTime();
      YoSharedBuffer buffer = simulationTestHelper.getSimulationConstructionSet().getBuffer();
      us.ihmc.scs2.simulation.robot.Robot robot = simulationTestHelper.getRobot();
      TorqueSpeedDataExporterGraphCreator graphCreator = new TorqueSpeedDataExporterGraphCreator(time, robot, buffer);
      DataExporterExcelWorkbookCreator excelWorkbookCreator = new DataExporterExcelWorkbookCreator(time, robot, buffer);

      // Stop the sim and disable the GUI:
      simulationTestHelper.getSimulationConstructionSet().stopSimulationThread();
      simulationTestHelper.getSimulationConstructionSet().disableGUIControls();

      // Crop the Buffer to In/Out. This is important because of how we use the DataBuffer later and we assume that in point is at index=0:
      simulationTestHelper.cropBuffer();
      simulationTestHelper.gotoInPoint();

      String timeStamp = FormattingTools.getDateString() + "_" + FormattingTools.getTimeString();
      String tagName;
      if (dataNameSuffix != null)
         tagName = timeStamp + "_" + robot.getName() + "_" + dataNameSuffix;
      else
         tagName = timeStamp + "_" + robot.getName();

      File dataFolder = new File(dataParentFolder, tagName);
      dataFolder.mkdir();

      if (info != null)
      {
         LogTools.info("Saving ReadMe");
         writeReadme(new File(dataFolder, tagName + ".txt"), info);
         LogTools.info("Done Saving ReadMe");
      }

      try
      {
         LogTools.info("Saving data");
         simulationTestHelper.getSimulationConstructionSet().getBuffer().exportDataMatlab(new File(dataFolder, tagName + ".mat"));
         LogTools.info("Done Saving Data");
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      catch (OutOfMemoryError exception)
      {
         LogTools.error("Ran out of memory while saving to Matlab format. Try again with fewer points.");
         exception.printStackTrace();
      }

      try
      {
         LogTools.info("Saving data in Matlab format");
         writeJointStatesMatlab(simulationTestHelper, new File(dataFolder, tagName + "_jointStates.mat"));
         LogTools.info("Done Saving Data in Matlab format");
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      catch (OutOfMemoryError exception)
      {
         LogTools.error("Ran out of memory while saving to Matlab format. Try again with fewer points.");
         exception.printStackTrace();
      }

      LogTools.info("creating torque and speed spreadsheet");
      excelWorkbookCreator.createAndSaveTorqueAndSpeedSpreadSheet(dataFolder, tagName);
      LogTools.info("done creating torque and speed spreadsheet");

      LogTools.info("creating torque and speed graphs");
      // make graph directory inside destination directory
      File graphDirectory = new File(dataFolder, "graphs");
      graphDirectory.mkdir();
      graphCreator.createJointTorqueSpeedGraphs(graphDirectory, tagName, true, false);
      LogTools.info("done creating torque and speed graphs");

      LogTools.info("creating video");
      simulationTestHelper.exportVideo(new File(dataFolder, tagName + "_Video.mov"));
      LogTools.info("done creating video");

      simulationTestHelper.getSimulationConstructionSet().enableGUIControls();
   }

   private static void writeReadme(File readmeFile, String info)
   {
      try
      {
         FileWriter out = new FileWriter(readmeFile);
         out.write(info);
         out.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static final Path DATA_PATH = Paths.get("/home/val/DataAndVideos");

   public static File getDataOutputFolder(String robotName, String folderName) throws IOException
   {
      Path path = DATA_PATH.resolve(robotName);
      if (folderName != null)
         path = path.resolve(folderName);
      FileTools.ensureDirectoryExists(path);
      return path.toFile();
   }

   public static FootstepDataListMessage generateCircleSteps(RobotSide initialStepSide,
                                                             int numberOfSteps,
                                                             DoubleUnaryOperator stepLengthFunction,
                                                             double stepWidth,
                                                             double swingTime,
                                                             double transferTime,
                                                             Pose3DReadOnly startPose,
                                                             boolean squareUp,
                                                             RobotSide turnSide,
                                                             double circleRadius)
   {
      if (turnSide == RobotSide.RIGHT)
         circleRadius = -circleRadius;

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage footstep = message.getFootstepDataList().add();

      RobotSide stepSide = initialStepSide;
      Pose3D circleCenter = new Pose3D(startPose);
      circleCenter.appendTranslation(0.0, circleRadius, 0.0);
      Pose3D stepPose = new Pose3D(startPose);
      double stepLength = 0.5 * stepLengthFunction.applyAsDouble(0.0);
      double angle = stepLength / circleRadius;
      stepPose.appendTranslation(yawAboutPoint(angle, circleCenter.getPosition(), 0.0, stepSide.negateIfRightSide(0.5 * stepWidth), 0.0));
      stepPose.appendYawRotation(angle);
      footstep.setRobotSide(stepSide.toByte());
      footstep.getLocation().set(stepPose.getPosition());
      footstep.getOrientation().set(stepPose.getOrientation());
      footstep.setSwingDuration(swingTime);

      for (int i = 1; i < numberOfSteps; i++)
      {
         stepSide = stepSide.getOppositeSide();
         stepLength = stepLengthFunction.applyAsDouble(i / (numberOfSteps - 1.0));
         angle = stepLength / circleRadius;
         stepPose.appendTranslation(yawAboutPoint(angle, circleCenter.getPosition(), 0.0, stepSide.negateIfRightSide(stepWidth), 0.0));
         stepPose.getOrientation().appendYawRotation(angle);
         footstep = message.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
         footstep.setTransferDuration(transferTime);
         footstep.setSwingDuration(swingTime);
      }

      if (squareUp)
      {
         stepSide = stepSide.getOppositeSide();
         stepPose.appendTranslation(0.0, stepSide.negateIfRightSide(stepWidth), 0.0);
         footstep = message.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
         footstep.setTransferDuration(transferTime);
         footstep.setSwingDuration(swingTime);
      }

      return message;
   }

   public static Point3D yawAboutPoint(double yaw, Point3DReadOnly center, double x, double y, double z)
   {
      return yawAboutPoint(yaw, center, new Point3D(x, y, z));
   }

   public static Point3D yawAboutPoint(double yaw, Point3DReadOnly center, Point3DReadOnly pointToTransform)
   {
      Point3D output = new Point3D();
      output.sub(pointToTransform, center);
      RotationMatrixTools.applyYawRotation(yaw, output, output);
      output.add(center);
      return output;
   }
}
