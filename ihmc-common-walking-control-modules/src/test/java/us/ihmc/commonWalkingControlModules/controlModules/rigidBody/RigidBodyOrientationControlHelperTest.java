package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RigidBodyOrientationControlHelperTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testControlFrameChange()
   {
      double epsilon = 1.0e-10;
      double trajectoryTime = 1.0;

      Random random = new Random(3543L);

      for (int i = 0; i < 100; i++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("Test");
         RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
         SixDoFJoint joint = new SixDoFJoint("joint", elevator, EuclidCoreRandomTools.nextRigidBodyTransform(random));
         RigidBody body = new RigidBody("body", joint, new Matrix3D(), 1.0, EuclidCoreRandomTools.nextRigidBodyTransform(random));

         joint.setPositionAndRotation(EuclidCoreRandomTools.nextRigidBodyTransform(random));
         joint.updateFramesRecursively();

         MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();
         ReferenceFrame defaultControlFrame = EuclidFrameRandomTools.nextReferenceFrame(random, bodyFrame);
         ReferenceFrame commandControlFrame = EuclidFrameRandomTools.nextReferenceFrame(random, bodyFrame);
         ReferenceFrame commandDesiredFrame = EuclidFrameRandomTools.nextReferenceFrame(random, ReferenceFrame.getWorldFrame());

         Quaternion defaultControlFrameRotationInBody = new Quaternion();
         Quaternion commandControlFrameRotationInBody = new Quaternion();
         Quaternion commandDesiredFrameRotation = new Quaternion();
         FrameQuaternion expected = new FrameQuaternion();

         defaultControlFrame.getTransformToDesiredFrame(bodyFrame).getRotation(defaultControlFrameRotationInBody);
         commandControlFrame.getTransformToDesiredFrame(bodyFrame).getRotation(commandControlFrameRotationInBody);
         commandDesiredFrame.getTransformToWorldFrame().getRotation(commandDesiredFrameRotation);

         RigidBodyOrientationControlHelper helper = new RigidBodyOrientationControlHelper("", body, elevator, elevator, null, defaultControlFrame,
                                                                                          ReferenceFrame.getWorldFrame(), () -> false, () -> false, registry);
         helper.setGains(new DefaultPID3DGains());
         helper.setWeights(new Vector3D());

         helper.holdCurrentDesired();
         helper.doAction(0.0);

         expected.setIncludingFrame(bodyFrame, defaultControlFrameRotationInBody);
         expected.changeFrame(ReferenceFrame.getWorldFrame());
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, helper.getFeedbackControlCommand().getDesiredOrientation(), epsilon);

         SO3TrajectoryControllerCommand command = new SO3TrajectoryControllerCommand();
         command.clear(ReferenceFrame.getWorldFrame());
         command.setTrajectoryFrame(ReferenceFrame.getWorldFrame());
         command.setUseCustomControlFrame(true);
         command.getControlFramePose().set(commandControlFrame.getTransformToParent());
         command.getTrajectoryPointList().addTrajectoryPoint(trajectoryTime, commandDesiredFrameRotation, new Vector3D());

         helper.handleTrajectoryCommand(command);
         helper.doAction(0.0);
         expected.setIncludingFrame(bodyFrame, commandControlFrameRotationInBody);
         expected.changeFrame(ReferenceFrame.getWorldFrame());
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, helper.getFeedbackControlCommand().getDesiredOrientation(), epsilon);

         helper.doAction(1.0);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(commandDesiredFrameRotation, helper.getFeedbackControlCommand().getDesiredOrientation(),
                                                                 epsilon);

         // This is the non-trivial part to test: the helper will reset the control frame to the default and should adjust the desired orientation accordingly.
         PoseReferenceFrame desiredC1Frame = new PoseReferenceFrame("desiredC1Frame", ReferenceFrame.getWorldFrame());
         PoseReferenceFrame desiredC2Frame = new PoseReferenceFrame("desiredC2Frame", desiredC1Frame);
         // This transform is fixed on the body. Both the defaultControlFrame and the commandControlFrame are random frames with parent bodyFrame.
         RigidBodyTransform C1ToC2Transform = defaultControlFrame.getTransformToDesiredFrame(commandControlFrame);
         desiredC2Frame.setPoseAndUpdate(C1ToC2Transform);
         // This sets the orientation of C1 to the desired pose in the helper
         desiredC1Frame.setOrientationAndUpdate(helper.getFeedbackControlCommand().getDesiredOrientation());
         expected.set(desiredC2Frame.getTransformToWorldFrame().getRotationMatrix());

         // Finally the actual test: the helper was tracking a desired trajectory in the commandControlFrame and this method switches the control frame to the defaultControlFrame.
         helper.holdCurrentDesired();
         helper.doAction(0.0);

         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, helper.getFeedbackControlCommand().getDesiredOrientation(), epsilon);
      }
   }
}
