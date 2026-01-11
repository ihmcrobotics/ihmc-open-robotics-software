package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.KinematicPoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.euclid.filters.FilteredFiniteDifferenceYoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.Random;

public class FixedBaseRobotArm
{
   private final Robot robot;
   private final List<OneDoFJointReadOnly> joints;
   private final List<OneDoFJointStateBasics> jointOutputs;

   private final RigidBodyBasics elevator;
   private final RigidBodyBasics hand;
   private final JointBasics elbowPitch;
   private final JointBasics wristYaw;

   private final RigidBodyTransform controlFrameTransform = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, 0.4));
   private final ReferenceFrame handControlFrame;
   private final KinematicPoint controlFrameTracker;
   private final YoDouble dummyAlpha = new YoDouble("dummy", new YoRegistry("dummy"));
   private final FilteredFiniteDifferenceYoFrameVector3D controlFrameLinearAcceleration;
   private final FilteredFiniteDifferenceYoFrameVector3D controlFrameAngularAcceleration;

   public FixedBaseRobotArm(Robot robot, double dt, YoRegistry registry)
   {
      this.robot = robot;
      joints = MultiBodySystemTools.filterJoints(robot.getJointsToConsider(), OneDoFJointReadOnly.class);
      jointOutputs = robot.getControllerOutput().getOneDoFJointOutputs(joints);

      elevator = MultiBodySystemTools.findRigidBody(robot.getRootBody(), FixedBaseRobotArmDefinition.elevatorName);
      hand = MultiBodySystemTools.findRigidBody(robot.getRootBody(), FixedBaseRobotArmDefinition.handName);

      elbowPitch = MultiBodySystemTools.findJoint(robot.getRootBody(), FixedBaseRobotArmDefinition.elbowPitchName);
      wristYaw = MultiBodySystemTools.findJoint(robot.getRootBody(), FixedBaseRobotArmDefinition.wristYawName);

      handControlFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("handControlFrame", hand.getBodyFixedFrame(), controlFrameTransform);
      controlFrameTracker = new KinematicPoint("controlFrameTracker", robot.findJoint(hand.getParentJoint().getName()), controlFrameTransform);

      controlFrameLinearAcceleration = new FilteredFiniteDifferenceYoFrameVector3D("controlFrameLinearAcceleration",
                                                                                   "",
                                                                                   dummyAlpha,
                                                                                   dt,
                                                                                   registry,
                                                                                   controlFrameTracker.getTwist().getLinearPart());
      controlFrameAngularAcceleration = new FilteredFiniteDifferenceYoFrameVector3D("controlFrameAngularAcceleration",
                                                                                    "",
                                                                                    dummyAlpha,
                                                                                    dt,
                                                                                    registry,
                                                                                    controlFrameTracker.getTwist().getAngularPart());

      setJointLimits();
   }

   public void update()
   {
      handControlFrame.update();
      controlFrameLinearAcceleration.update();
      controlFrameAngularAcceleration.update();
   }

   private void setJointLimits()
   {
      RevoluteJoint[] allJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(elevator), RevoluteJoint.class);
      for (RevoluteJoint revoluteJoint : allJoints)
      {
         revoluteJoint.setJointLimitUpper(Math.PI);
         revoluteJoint.setJointLimitLower(-Math.PI);
      }
   }

   public void setRandomConfiguration()
   {
      Random random = new Random();

      for (SimJointBasics joint : robot.getAllJoints())
      {
         if (joint instanceof SimRevoluteJoint revoluteJoint)
         {
            double lowerLimit = revoluteJoint.getJointLimitLower();
            if (!Double.isFinite(lowerLimit))
               lowerLimit = -Math.PI;
            double upperLimit = revoluteJoint.getJointLimitUpper();
            if (!Double.isFinite(upperLimit))
               upperLimit = Math.PI;

            revoluteJoint.setQ(RandomNumbers.nextDouble(random, lowerLimit, upperLimit));
         }
      }
   }

   public void updateSCSRobotJointTaus(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJointStateBasics jointOutput = jointOutputs.get(i);
         OneDoFJointReadOnly oneDoFJoint = joints.get(i);
         JointDesiredOutputReadOnly data = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(oneDoFJoint);

         if (data.hasDesiredTorque())
         {
            double tau = data.getDesiredTorque();
            jointOutput.setEffort(tau);
         }
      }
   }

   public void updateSCSRobotJointConfiguration(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJointStateBasics jointOutput = jointOutputs.get(i);
         OneDoFJointReadOnly oneDoFJoint = joints.get(i);
         JointDesiredOutputReadOnly data = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(oneDoFJoint);

         if (data.hasDesiredPosition())
         {
            double q = data.getDesiredPosition();
            jointOutput.setConfiguration(q);
         }

         if (data.hasDesiredVelocity())
         {
            double qd = data.getDesiredVelocity();
            jointOutput.setVelocity(qd);
         }
      }
   }

   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   public RigidBodyBasics getHand()
   {
      return hand;
   }

   public ReferenceFrame getHandControlFrame()
   {
      return handControlFrame;
   }

   public JointBasics getElbowPitch()
   {
      return elbowPitch;
   }

   public JointBasics getWristYaw()
   {
      return wristYaw;
   }

   public SimJointBasics getSimWristYaw()
   {
      return robot.getJoint(FixedBaseRobotArmDefinition.wristYawName);
   }
}
