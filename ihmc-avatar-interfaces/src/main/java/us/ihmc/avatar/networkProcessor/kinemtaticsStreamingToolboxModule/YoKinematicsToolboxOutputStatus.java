package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.Arrays;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

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
   };

   private int numberOfJoints;
   private final YoEnum<Status> currentToolboxState;
   private final YoInteger jointNameHash;
   private final YoDouble[] desiredJointAngles;
   private final YoDouble[] desiredJointVelocities;
   private final YoDouble[] desiredJointAccelerations;
   private final YoFramePose3D desiredRootJointPose;
   private final YoFixedFrameSpatialVector desiredRootJointVelocity;
   private final YoFixedFrameSpatialVector desiredRootJointAcceleration;

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
      desiredJointAccelerations = new YoDouble[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         String jointName = oneDoFJoints[i].getName();
         desiredJointAngles[i] = new YoDouble("q_d_" + namePrefix + "_" + jointName, registry);
         desiredJointVelocities[i] = new YoDouble("qd_d_" + namePrefix + "_" + jointName, registry);
         desiredJointAccelerations[i] = new YoDouble("qdd_d_" + namePrefix + "_" + jointName, registry);
      }

      desiredRootJointPose = new YoFramePose3D(namePrefix + "Desired" + rootJoint.getName(), ReferenceFrame.getWorldFrame(), registry);
      desiredRootJointVelocity = new YoFixedFrameSpatialVector(namePrefix + "DesiredVelocity" + rootJoint.getName(), ReferenceFrame.getWorldFrame(), registry);
      desiredRootJointAcceleration = new YoFixedFrameSpatialVector(namePrefix + "DesiredAcceleration" + rootJoint.getName(), ReferenceFrame.getWorldFrame(), registry);
   }

   public void setToNaN()
   {
      currentToolboxState.set(null);
      for (YoDouble desiredJointAngle : desiredJointAngles)
         desiredJointAngle.setToNaN();
      for (YoDouble desiredJointVelocity : desiredJointVelocities)
         desiredJointVelocity.setToNaN();
      for (YoDouble desiredJointAcceleration : desiredJointAccelerations)
         desiredJointAcceleration.setToNaN();
      desiredRootJointPose.setToNaN();
      desiredRootJointVelocity.setToNaN();
      desiredRootJointAcceleration.setToNaN();
   }

   public int getJointNameHash()
   {
      return jointNameHash.getIntegerValue();
   }

   public double getDesiredJointVelocity(int i)
   {
      return desiredJointVelocities[i].getValue();
   }

   public FrameVector3DReadOnly getDesiredRootLinearVelocity()
   {
      return desiredRootJointVelocity.getLinearPart();
   }

   public FrameVector3DReadOnly getDesiredRootAngularVelocity()
   {
      return desiredRootJointVelocity.getAngularPart();
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
      desiredRootJointAcceleration.set(status.getDesiredRootAngularAcceleration(), status.getDesiredRootLinearAcceleration());
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
         desiredJointAccelerations[i].set(other.desiredJointAccelerations[i].getValue());
      }
      desiredRootJointPose.set(other.desiredRootJointPose);
      desiredRootJointVelocity.set(other.desiredRootJointVelocity);
      desiredRootJointAcceleration.set(other.desiredRootJointAcceleration);
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

   public void scaleAccelerations(double scaleFactor)
   {
      for (int i = 0; i < numberOfJoints; i++)
      {
         desiredJointAccelerations[i].set(desiredJointAccelerations[i].getValue() * scaleFactor);
      }
      desiredRootJointAcceleration.scale(scaleFactor);
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

      desiredRootJointVelocity.getLinearPart().sub(current.getDesiredRootPosition(), current.getDesiredRootPosition());
      desiredRootJointVelocity.getLinearPart().scale(1.0 / duration);
      desiredRootJointPose.getOrientation().inverseTransform(desiredRootJointVelocity.getLinearPart());
      quaternionDot.sub(current.getDesiredRootOrientation(), previous.getDesiredRootOrientation());
      quaternionDot.scale(1.0 / duration);
      quaternionCalculus.computeAngularVelocityInBodyFixedFrame(desiredRootJointPose.getOrientation(),
                                                                quaternionDot,
                                                                desiredRootJointVelocity.getAngularPart());
   }

   public void setDesiredAccelerationsByFiniteDifference(YoKinematicsToolboxOutputStatus previous, YoKinematicsToolboxOutputStatus current, double duration)
   {
      if (previous.getJointNameHash() != current.getJointNameHash())
         throw new RuntimeException("Output status are not compatible.");

      for (int i = 0; i < numberOfJoints; i++)
      {
         double qdd = (current.getDesiredJointVelocity(i) - previous.getDesiredJointVelocity(i)) / duration;
         desiredJointAccelerations[i].set(qdd);
      }

      desiredRootJointAcceleration.getLinearPart().sub(current.getDesiredRootLinearVelocity(), previous.getDesiredRootLinearVelocity());
      desiredRootJointAcceleration.getAngularPart().sub(current.getDesiredRootAngularVelocity(), previous.getDesiredRootAngularVelocity());
      desiredRootJointAcceleration.scale(1.0 / duration);
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
         status.getDesiredJointAccelerations().add((float) desiredJointAccelerations[i].getValue());
      }

      status.getDesiredRootPosition().set(desiredRootJointPose.getPosition());
      status.getDesiredRootOrientation().set(desiredRootJointPose.getOrientation());
      status.getDesiredRootLinearVelocity().set(desiredRootJointVelocity.getLinearPart());
      status.getDesiredRootAngularVelocity().set(desiredRootJointVelocity.getAngularPart());
      status.getDesiredRootLinearAcceleration().set(desiredRootJointAcceleration.getLinearPart());
      status.getDesiredRootAngularAcceleration().set(desiredRootJointAcceleration.getAngularPart());
      return status;
   }
}
