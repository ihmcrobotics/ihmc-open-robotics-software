package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationSettings;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class RigidBodyControlManagerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(94391892L);
   private static final double epsilon = 1.0E-10;

   private YoVariableRegistry testRegistry;
   private DoubleYoVariable yoTime;
   private RigidBody bodyToControl;

   private OneDoFJoint joint1;
   private OneDoFJoint joint2;

   private double q1_init = random.nextDouble();
   private double q2_init = random.nextDouble();

   private double q1_home = random.nextDouble();
   private double q2_home = random.nextDouble();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testConstuctor()
   {
      createManager();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(expected=RuntimeException.class)
   public void testFailWithoutGains()
   {
      RigidBodyControlManager manager = createManager();
      manager.initialize();
      manager.compute();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testInitialize()
   {
      // create manager
      RigidBodyControlManager manager = createManager();
      assertEquals(q2_init, joint2.getQ(), Double.MIN_VALUE);
      assertEquals(q1_init, joint1.getQ(), Double.MIN_VALUE);

      // compute
      setGainsAndWeights(manager);
      manager.compute();
      assertEquals(RigidBodyControlMode.JOINTSPACE, manager.getActiveControlMode());

      // get commands and make sure they are initialized correctly
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.JOINTSPACE, feedbackControlCommand.getCommandType());
         JointspaceFeedbackControlCommand jointCommand = (JointspaceFeedbackControlCommand) feedbackControlCommand;
         assertEquals(1, jointCommand.getNumberOfJoints());
         assertEquals(jointCommand.getDesiredPosition(0), q2_init, Double.MIN_VALUE);
      }

      // go to time when the joints should be at the home position
      yoTime.set(RigidBodyControlManager.INITIAL_GO_HOME_TIME);
      manager.compute();
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.JOINTSPACE, feedbackControlCommand.getCommandType());
         JointspaceFeedbackControlCommand jointCommand = (JointspaceFeedbackControlCommand) feedbackControlCommand;
         assertEquals(1, jointCommand.getNumberOfJoints());
         assertEquals(jointCommand.getDesiredPosition(0), q2_home, Double.MIN_VALUE);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testTaskspaceMessage()
   {
      RigidBodyControlManager manager = createManager();
      setGainsAndWeights(manager);
      manager.compute();

      double trajectoryTime = 1.0;
      Point3D position = EuclidCoreRandomTools.generateRandomPoint3D(random);
      Quaternion orientation = EuclidCoreRandomTools.generateRandomQuaternion(random);
      Vector3D linearVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);
      Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);

      SE3Message message = new SE3Message(1, worldFrame);
      message.setTrajectoryPoint(0, trajectoryTime, position, orientation, linearVelocity, angularVelocity, worldFrame);

      SE3Command command = new SE3Command();
      command.set(worldFrame, worldFrame, message);
      manager.handleTaskspaceTrajectoryCommand(command);
      manager.compute();

      assertEquals(RigidBodyControlMode.TASKSPACE, manager.getActiveControlMode());

      FramePoint desiredPosition = new FramePoint();
      FrameVector desiredLinearVelocity = new FrameVector();
      FrameVector feedForwardLinearAcceleration = new FrameVector();
      FrameOrientation desiredOrientation = new FrameOrientation();
      FrameVector desiredAngularVelocity = new FrameVector();
      FrameVector feedForwardAngularAcceleration = new FrameVector();

      ReferenceFrame bodyFrame = bodyToControl.getBodyFixedFrame();
      FramePoint initialPosition = new FramePoint(bodyFrame);
      FrameOrientation initialOrientation = new FrameOrientation(bodyFrame);
      initialPosition.changeFrame(worldFrame);
      initialOrientation.changeFrame(worldFrame);

      // get commands and make sure they are initialized correctly
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(initialPosition.getPoint(), desiredPosition.getPoint(), Double.MIN_VALUE);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(initialOrientation.getQuaternion(), desiredOrientation.getQuaternion(), Double.MIN_VALUE);
      }

      // go forward to the end of the trajectory
      yoTime.set(trajectoryTime);
      manager.compute();
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(position, desiredPosition.getPoint(), Double.MIN_VALUE);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(orientation, desiredOrientation.getQuaternion(), Double.MIN_VALUE);

         EuclidCoreTestTools.assertTuple3DEquals(linearVelocity, desiredLinearVelocity.getVector(), Double.MIN_VALUE);
         EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, desiredAngularVelocity.getVector(), Double.MIN_VALUE);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testTaskspaceMessageWithCustomControlFrame()
   {
      RigidBodyControlManager manager = createManager();
      setGainsAndWeights(manager);
      manager.compute();

      double trajectoryTime = 1.0;
      Point3D position = EuclidCoreRandomTools.generateRandomPoint3D(random);
      Quaternion orientation = EuclidCoreRandomTools.generateRandomQuaternion(random);
      Vector3D linearVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);
      Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);

      Point3D controlFramePosition = EuclidCoreRandomTools.generateRandomPoint3D(random);
      Quaternion controlFrameOrientation = EuclidCoreRandomTools.generateRandomQuaternion(random);

      SE3Message message = new SE3Message(1, worldFrame);
      message.setControlFramePosition(controlFramePosition);
      message.setControlFrameOrientation(controlFrameOrientation);
      message.setUseCustomControlFrame(true);
      message.setTrajectoryPoint(0, trajectoryTime, position, orientation, linearVelocity, angularVelocity, worldFrame);

      SE3Command command = new SE3Command();
      command.set(worldFrame, worldFrame, message);
      manager.handleTaskspaceTrajectoryCommand(command);
      manager.compute();

      assertEquals(RigidBodyControlMode.TASKSPACE, manager.getActiveControlMode());

      FramePoint desiredPosition = new FramePoint();
      FrameVector desiredLinearVelocity = new FrameVector();
      FrameVector feedForwardLinearAcceleration = new FrameVector();
      FrameOrientation desiredOrientation = new FrameOrientation();
      FrameVector desiredAngularVelocity = new FrameVector();
      FrameVector feedForwardAngularAcceleration = new FrameVector();
      FrameOrientation actualControlFrameOrientation = new FrameOrientation();
      FramePoint actualControlFramePosition = new FramePoint();

      ReferenceFrame bodyFrame = bodyToControl.getBodyFixedFrame();
      PoseReferenceFrame controlFrame = new PoseReferenceFrame("TestControlFrame", bodyFrame);
      controlFrame.setPoseAndUpdate(controlFramePosition, controlFrameOrientation);
      FramePoint initialPosition = new FramePoint(controlFrame);
      FrameOrientation initialOrientation = new FrameOrientation(controlFrame);
      initialPosition.changeFrame(worldFrame);
      initialOrientation.changeFrame(worldFrame);

      // get commands and make sure they are initialized correctly
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(initialPosition.getPoint(), desiredPosition.getPoint(), Double.MIN_VALUE);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(initialOrientation.getQuaternion(), desiredOrientation.getQuaternion(), Double.MIN_VALUE);

         taskspaceCommand.getControlFramePoseIncludingFrame(actualControlFramePosition, actualControlFrameOrientation);
         actualControlFramePosition.checkReferenceFrameMatch(bodyFrame);
         actualControlFrameOrientation.checkReferenceFrameMatch(bodyFrame);
         EuclidCoreTestTools.assertTuple3DEquals(controlFramePosition, actualControlFramePosition.getPoint(), Double.MIN_VALUE);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(controlFrameOrientation, actualControlFrameOrientation.getQuaternion(), epsilon);
      }

      // go forward to the end of the trajectory
      yoTime.set(trajectoryTime);
      manager.compute();
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(position, desiredPosition.getPoint(), Double.MIN_VALUE);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(orientation, desiredOrientation.getQuaternion(), Double.MIN_VALUE);

         EuclidCoreTestTools.assertTuple3DEquals(linearVelocity, desiredLinearVelocity.getVector(), Double.MIN_VALUE);
         EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, desiredAngularVelocity.getVector(), Double.MIN_VALUE);
      }
   }

   private RigidBodyControlManager createManager()
   {
      // create test registry
      testRegistry = new YoVariableRegistry(getClass().getSimpleName());

      // create time variable
      yoTime = new DoubleYoVariable("yoTime", testRegistry);

      // create a dummy robot with elevator, two joints, and two rigid bodies
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      joint1 = ScrewTools.addRevoluteJoint("Joint1", elevator, new Vector3D(), new Vector3D(1.0, 0.0, 0.0));
      RigidBody link1 = ScrewTools.addRigidBody("Link1", joint1, new Matrix3D(), 0.0, new Vector3D());
      joint2 = ScrewTools.addRevoluteJoint("Joint2", link1, new Vector3D(), new Vector3D(1.0, 0.0, 0.0));
      RigidBody link2 = ScrewTools.addRigidBody("Link2", joint2, new Matrix3D(), 0.0, new Vector3D());

      joint1.setQ(q1_init);
      joint2.setQ(q2_init);

      // setup a home configuration for the robot
      TObjectDoubleHashMap<String> homeConfiguration = new TObjectDoubleHashMap<>();
      homeConfiguration.put(joint1.getName(), q1_home);
      homeConfiguration.put(joint2.getName(), q2_home);

      // no position controlled joints
      List<String> positionControlledJointNames = new ArrayList<>();
      Map<String, JointAccelerationIntegrationSettings> integrationSettings = new HashMap<>();

      // add some possible trajectory frames
      Collection<ReferenceFrame> trajectoryFrames = new ArrayList<>();
      trajectoryFrames.add(link1.getBodyFixedFrame());
      trajectoryFrames.add(link2.getBodyFixedFrame());

      // create a contactable body
      ContactablePlaneBody contactableBody = new SimpleContactPointPlaneBody("ContactableBody", link2, new RigidBodyTransform());

      // use default control and base frames
      bodyToControl = link2;
      RigidBody baseBody = link1;
      ReferenceFrame controlFrame = bodyToControl.getBodyFixedFrame();
      ReferenceFrame baseFrame = baseBody.getBodyFixedFrame();

      return new RigidBodyControlManager(bodyToControl, baseBody, elevator, homeConfiguration, positionControlledJointNames,
            integrationSettings, trajectoryFrames, controlFrame, baseFrame, contactableBody, yoTime, null, testRegistry);
   }

   private void setGainsAndWeights(RigidBodyControlManager manager)
   {
      // setup gains and weights to be all zero with weights 1.0
      Map<String, YoPIDGains> jointspaceGains = new HashMap<>();
      jointspaceGains.put(joint1.getName(), new YoPIDGains("Joint1Gains", testRegistry));
      jointspaceGains.put(joint2.getName(), new YoPIDGains("Joint2Gains", testRegistry));
      YoOrientationPIDGainsInterface taskspaceOrientationGains = new YoSymmetricSE3PIDGains("OrientationGains", testRegistry);
      YoPositionPIDGainsInterface taskspacePositionGains = new YoSymmetricSE3PIDGains("PositionGains", testRegistry);
      TObjectDoubleHashMap<String> jointspaceWeights = new TObjectDoubleHashMap<>();
      jointspaceWeights.put(joint1.getName(), 1.0);
      jointspaceWeights.put(joint2.getName(), 1.0);
      Vector3D taskspaceAngularWeight = new Vector3D(1.0, 1.0, 1.0);
      Vector3D taskspaceLinearWeight = new Vector3D(1.0, 1.0, 1.0);
      TObjectDoubleHashMap<String> userModeWeights = new TObjectDoubleHashMap<>();
      userModeWeights.put(joint1.getName(), 1.0);
      userModeWeights.put(joint2.getName(), 1.0);

      manager.setGains(jointspaceGains, taskspaceOrientationGains, taskspacePositionGains);
      manager.setWeights(jointspaceWeights, taskspaceAngularWeight, taskspaceLinearWeight, userModeWeights);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(RigidBodyControlManager.class, RigidBodyControlManagerTest.class);
   }
}
