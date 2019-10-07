package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.Arrays;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoKinematicsToolboxOutputStatus
{
   private final YoVariableRegistry registry;

   public enum Status
   {
      NO_STATUS, INITIALIZE_SUCCESSFUL, INITIALIZE_FAILURE_MISSING_RCD, RUNNING;

      public static Status fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         else
            return values()[enumAsByte];
      }

      public static byte toByte(Status status)
      {
         if (status == null)
            return -1;
         else
            return (byte) status.ordinal();
      }
   };

   private int numberOfJoints;
   private final YoEnum<Status> currentToolboxState;
   private final YoInteger jointNameHash;
   private final YoDouble[] desiredJointAngles;
   private final YoDouble[] desiredJointVelocities;
   private final YoFramePose3D desiredRootJointPose;
   private final YoFixedFrameSpatialVector desiredRootJointVelocity;

   public YoKinematicsToolboxOutputStatus(String namePrefix, FloatingJointBasics rootJoint, OneDoFJointBasics[] oneDoFJoints, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
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

      desiredRootJointPose.set(status.getDesiredRootTranslation(), status.getDesiredRootOrientation());
      desiredRootJointVelocity.set(status.getDesiredRootAngularVelocity(), status.getDesiredRootLinearVelocity());
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

   public void interpolate(KinematicsToolboxOutputStatus end, double alpha)
   {
      interpolate(this.getStatus(), end, alpha);
   }

   public void interpolate(KinematicsToolboxOutputStatus start, KinematicsToolboxOutputStatus end, double alpha)
   {
      set(MessageTools.interpolateMessages(start, end, alpha));
   }

   public void interpolate(KinematicsToolboxOutputStatus start, KinematicsToolboxOutputStatus end, double alpha, double alphaDot)
   {
      set(MessageTools.interpolate(start, end, alpha, alphaDot));
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
         desiredJointVelocities[i].set(qd);
      }

      desiredRootJointVelocity.getLinearPart().sub(current.getDesiredRootTranslation(), current.getDesiredRootTranslation());
      desiredRootJointVelocity.getLinearPart().scale(1.0 / duration);
      desiredRootJointPose.getOrientation().inverseTransform(desiredRootJointVelocity.getLinearPart());
      quaternionDot.sub(current.getDesiredRootOrientation(), previous.getDesiredRootOrientation());
      quaternionDot.scale(1.0 / duration);
      quaternionCalculus.computeAngularVelocityInBodyFixedFrame(desiredRootJointPose.getOrientation(),
                                                                quaternionDot,
                                                                desiredRootJointVelocity.getAngularPart());
   }

   public KinematicsToolboxOutputStatus getStatus()
   {
      KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();
      status.setJointNameHash(jointNameHash.getValue());
      status.setCurrentToolboxState(Status.toByte(currentToolboxState.getEnumValue()));
      for (int i = 0; i < numberOfJoints; i++)
      {
         status.getDesiredJointAngles().add((float) desiredJointAngles[i].getValue());
         status.getDesiredJointVelocities().add((float) desiredJointVelocities[i].getValue());
      }

      status.getDesiredRootTranslation().set(desiredRootJointPose.getPosition());
      status.getDesiredRootOrientation().set(desiredRootJointPose.getOrientation());
      status.getDesiredRootLinearVelocity().set(desiredRootJointVelocity.getLinearPart());
      status.getDesiredRootAngularVelocity().set(desiredRootJointVelocity.getAngularPart());
      return status;
   }
}
