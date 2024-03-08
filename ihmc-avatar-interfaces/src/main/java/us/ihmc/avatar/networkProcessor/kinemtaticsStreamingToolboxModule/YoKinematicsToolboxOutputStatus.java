package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Arrays;

public class YoKinematicsToolboxOutputStatus
{
   private final YoRegistry registry;

   public enum Status
   {
      NO_STATUS, INITIALIZE_SUCCESSFUL, INITIALIZE_FAILURE_MISSING_RCD, RUNNING;

      public static final Status[] values = values();

      public static Status fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         else
            return values[enumAsByte];
      }

      public static byte toByte(Status status)
      {
         if (status == null)
            return -1;
         else
            return (byte) status.ordinal();
      }
   }

   private int numberOfJoints;
   private final YoEnum<Status> currentToolboxState;
   private final YoInteger jointNameHash;
   private final YoDouble[] desiredJointAngles;
   private final YoDouble[] desiredJointVelocities;
   private final YoFramePose3D desiredRootJointPose;
   private final YoFixedFrameSpatialVector desiredRootJointVelocity;

   public YoKinematicsToolboxOutputStatus(String namePrefix, FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this(namePrefix, fullRobotModel.getRootJoint(), FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel), parentRegistry);
   }

   public YoKinematicsToolboxOutputStatus(String namePrefix, FloatingJointBasics rootJoint, OneDoFJointBasics[] oneDoFJoints, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      currentToolboxState = new YoEnum<>(namePrefix + "CurrentToolboxState", registry, Status.class, true);

      jointNameHash = new YoInteger(namePrefix + "JointNameHash", registry);
      jointNameHash.set(Arrays.hashCode(oneDoFJoints));

      numberOfJoints = oneDoFJoints.length;
      desiredJointAngles = new YoDouble[numberOfJoints];
      desiredJointVelocities = new YoDouble[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         String jointName = oneDoFJoints[i].getName();
         desiredJointAngles[i] = new YoDouble("q_d_" + namePrefix + "_" + jointName, registry);
         desiredJointVelocities[i] = new YoDouble("qd_d_" + namePrefix + "_" + jointName, registry);
      }

      desiredRootJointPose = new YoFramePose3D(namePrefix + "Desired" + rootJoint.getName(), ReferenceFrame.getWorldFrame(), registry);
      desiredRootJointVelocity = new YoFixedFrameSpatialVector(namePrefix + "DesiredVelocity" + rootJoint.getName(), ReferenceFrame.getWorldFrame(), registry);
   }

   public void setToNaN()
   {
      currentToolboxState.set(null);
      for (YoDouble desiredJointAngle : desiredJointAngles)
         desiredJointAngle.setToNaN();
      for (YoDouble desiredJointVelocity : desiredJointVelocities)
         desiredJointVelocity.setToNaN();
      desiredRootJointPose.setToNaN();
      desiredRootJointVelocity.setToNaN();
   }

   public void set(KinematicsToolboxOutputStatus status)
   {
      if (status.getJointNameHash() != jointNameHash.getValue())
         throw new IllegalArgumentException("Incompatible status");

      currentToolboxState.set(Status.fromByte(status.getCurrentToolboxState()));

      for (int i = 0; i < numberOfJoints; i++)
      {
         desiredJointAngles[i].set(status.getDesiredJointAngles().get(i));
         desiredJointVelocities[i].set(status.getDesiredJointVelocities().get(i));
      }

      desiredRootJointPose.set(status.getDesiredRootPosition(), status.getDesiredRootOrientation());
      desiredRootJointVelocity.set(status.getDesiredRootAngularVelocity(), status.getDesiredRootLinearVelocity());
   }

   public void setConfigurationOnly(KinematicsToolboxOutputStatus status)
   {
      if (status.getJointNameHash() != jointNameHash.getValue())
         throw new IllegalArgumentException("Incompatible status");

      for (int i = 0; i < numberOfJoints; i++)
      {
         desiredJointAngles[i].set(status.getDesiredJointAngles().get(i));
      }

      desiredRootJointPose.set(status.getDesiredRootPosition(), status.getDesiredRootOrientation());
   }

   public void set(YoKinematicsToolboxOutputStatus other)
   {
      jointNameHash.set(other.jointNameHash.getValue());
      currentToolboxState.set(other.currentToolboxState.getEnumValue());
      numberOfJoints = other.numberOfJoints;
      for (int i = 0; i < numberOfJoints; i++)
      {
         desiredJointAngles[i].set(other.desiredJointAngles[i].getValue());
         desiredJointVelocities[i].set(other.desiredJointVelocities[i].getValue());
      }
      desiredRootJointPose.set(other.desiredRootJointPose);
      desiredRootJointVelocity.set(other.desiredRootJointVelocity);
   }

   private final KinematicsToolboxOutputStatus interpolationStatus = new KinematicsToolboxOutputStatus();

   public void interpolate(KinematicsToolboxOutputStatus end, double alpha)
   {
      interpolate(this.getStatus(), end, alpha);
   }

   public void interpolate(KinematicsToolboxOutputStatus start, KinematicsToolboxOutputStatus end, double alpha)
   {
      MessageTools.interpolateMessages(start, end, alpha, interpolationStatus);
      set(interpolationStatus);
   }

   public void interpolate(KinematicsToolboxOutputStatus start, KinematicsToolboxOutputStatus end, double alpha, double alphaDot)
   {
      MessageTools.interpolate(start, end, alpha, alphaDot, interpolationStatus);
      set(interpolationStatus);
   }

   public void scaleVelocities(double scaleFactor)
   {
      for (int i = 0; i < numberOfJoints; i++)
      {
         desiredJointVelocities[i].set(desiredJointVelocities[i].getValue() * scaleFactor);
      }
      desiredRootJointVelocity.scale(scaleFactor);
   }

   private final Vector4D quaternionDot = new Vector4D();
   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   public void setDesiredVelocitiesByFiniteDifference(KinematicsToolboxOutputStatus previous, KinematicsToolboxOutputStatus current, double duration)
   {
      if (previous.getJointNameHash() != current.getJointNameHash())
         throw new RuntimeException("Output status are not compatible.");

      for (int i = 0; i < numberOfJoints; i++)
      {
         double qd = (current.getDesiredJointAngles().get(i) - previous.getDesiredJointAngles().get(i)) / duration;
         if (!Double.isFinite(qd))
            qd = 0.0;
         desiredJointVelocities[i].set(qd);
      }

      desiredRootJointVelocity.getLinearPart().sub(current.getDesiredRootPosition(), current.getDesiredRootPosition());
      desiredRootJointVelocity.getLinearPart().scale(1.0 / duration);
      if (desiredRootJointVelocity.containsNaN())
         desiredRootJointVelocity.setToZero();
      else
         desiredRootJointPose.getOrientation().inverseTransform(desiredRootJointVelocity.getLinearPart());
      quaternionDot.sub(current.getDesiredRootOrientation(), previous.getDesiredRootOrientation());
      quaternionDot.scale(1.0 / duration);
      if (quaternionDot.containsNaN())
      {
         quaternionDot.setToZero();
         desiredRootJointVelocity.setToZero();
      }
      else
      {
         quaternionCalculus.computeAngularVelocityInBodyFixedFrame(desiredRootJointPose.getOrientation(),
                                                                   quaternionDot,
                                                                   desiredRootJointVelocity.getAngularPart());
      }
   }

   private final KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();

   public KinematicsToolboxOutputStatus getStatus()
   {
      status.setJointNameHash(jointNameHash.getValue());
      status.setCurrentToolboxState(Status.toByte(currentToolboxState.getEnumValue()));

      status.getDesiredJointAngles().reset();
      status.getDesiredJointVelocities().reset();

      for (int i = 0; i < numberOfJoints; i++)
      {
         status.getDesiredJointAngles().add((float) desiredJointAngles[i].getValue());
         status.getDesiredJointVelocities().add((float) desiredJointVelocities[i].getValue());
      }

      status.getDesiredRootPosition().set(desiredRootJointPose.getPosition());
      status.getDesiredRootOrientation().set(desiredRootJointPose.getOrientation());
      status.getDesiredRootLinearVelocity().set(desiredRootJointVelocity.getLinearPart());
      status.getDesiredRootAngularVelocity().set(desiredRootJointVelocity.getAngularPart());
      return status;
   }
}
