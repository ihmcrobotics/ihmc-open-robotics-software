package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output.KSTOutputDataBasics;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output.KSTOutputDataReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
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

public class YoKinematicsToolboxOutputStatus implements KSTOutputDataBasics
{
   private final YoRegistry registry;
   private final String namePrefix;

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

   private final int numberOfJoints;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final YoEnum<Status> currentToolboxState;
   private final YoInteger jointNameHash;

   private final YoFramePose3D desiredRootJointPose;
   private final YoDouble[] desiredJointAngles;

   private final YoFixedFrameSpatialVector desiredRootJointVelocity;
   private final YoDouble[] desiredJointVelocities;

   private YoFixedFrameSpatialVector desiredRootJointAcceleration;
   private YoDouble[] desiredJointAccelerations;

   public YoKinematicsToolboxOutputStatus(String namePrefix, FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      rootJoint = fullRobotModel.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
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

   public void createAccelerationState()
   {
      desiredRootJointAcceleration = new YoFixedFrameSpatialVector(namePrefix + "DesiredAcceleration" + rootJoint.getName(),
                                                                   ReferenceFrame.getWorldFrame(),
                                                                   registry);
      desiredJointAccelerations = new YoDouble[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         String jointName = oneDoFJoints[i].getName();
         desiredJointAccelerations[i] = new YoDouble("qdd_d_" + namePrefix + "_" + jointName, registry);
      }
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

      if (desiredJointAccelerations != null)
      {
         for (YoDouble desiredJointAcceleration : desiredJointAccelerations)
            desiredJointAcceleration.setToNaN();
         desiredRootJointAcceleration.setToNaN();
      }
   }

   public void set(YoKinematicsToolboxOutputStatus other)
   {
      if (jointNameHash.getValue() != other.jointNameHash.getValue())
         throw new IllegalArgumentException("Incompatible status");

      currentToolboxState.set(other.currentToolboxState.getEnumValue());

      for (int i = 0; i < numberOfJoints; i++)
      {
         desiredJointAngles[i].set(other.desiredJointAngles[i].getValue());
         desiredJointVelocities[i].set(other.desiredJointVelocities[i].getValue());
      }
      desiredRootJointPose.set(other.desiredRootJointPose);
      desiredRootJointVelocity.set(other.desiredRootJointVelocity);
   }

   public void interpolate(KSTOutputDataReadOnly start, KSTOutputDataReadOnly end, double alpha, double alphaDot)
   {
      if (getJointNameHash() != start.getJointNameHash() || getJointNameHash() != end.getJointNameHash())
         throw new IllegalArgumentException("Joint name hash does not match, cannot interpolate data from different robot.");

      if (getNumberOfJoints() != start.getNumberOfJoints() || getNumberOfJoints() != end.getNumberOfJoints())
         throw new IllegalArgumentException("Number of joints does not match, cannot interpolate data from different robot.");

      // 1-DoF joints:

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         double q = EuclidCoreTools.interpolate(start.getJointPosition(i), end.getJointPosition(i), alpha);
         double qDot = alphaDot * (end.getJointPosition(i) - start.getJointPosition(i));
         qDot += EuclidCoreTools.interpolate(start.getJointVelocity(i), end.getJointVelocity(i), alpha);

         setJointPosition(i, q);
         setJointVelocity(i, qDot);
      }

      // Root joint:
      // Do configuration
      getRootJointOrientation().interpolate(start.getRootJointOrientation(), end.getRootJointOrientation(), alpha);
      getRootJointPosition().interpolate(start.getRootJointPosition(), end.getRootJointPosition(), alpha);

      // Root joint velocity
      quaternionDot.sub(end.getRootJointOrientation(), start.getRootJointOrientation());
      quaternionDot.scale(alphaDot);
      quaternionCalculus.computeAngularVelocityInBodyFixedFrame(getRootJointOrientation(), quaternionDot, getRootJointAngularVelocity());
      getRootJointAngularVelocity().scaleAdd(1.0 - alpha, start.getRootJointAngularVelocity(), getRootJointAngularVelocity());
      getRootJointAngularVelocity().scaleAdd(alpha, end.getRootJointAngularVelocity(), getRootJointAngularVelocity());

      getRootJointLinearVelocity().sub(end.getRootJointPosition(), start.getRootJointPosition());
      getRootJointLinearVelocity().scale(alphaDot);
      getRootJointOrientation().inverseTransform(getRootJointLinearVelocity());
      getRootJointLinearVelocity().scaleAdd(1.0 - alpha, start.getRootJointLinearVelocity(), getRootJointLinearVelocity());
      getRootJointLinearVelocity().scaleAdd(alpha, end.getRootJointLinearVelocity(), getRootJointLinearVelocity());
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

   public void setDesiredVelocitiesByFiniteDifference(KSTOutputDataReadOnly previous, KSTOutputDataReadOnly current, double duration)
   {
      if (previous.getJointNameHash() != current.getJointNameHash() || previous.getJointNameHash() != jointNameHash.getValue())
         throw new RuntimeException("Output status are not compatible.");

      for (int i = 0; i < numberOfJoints; i++)
      {
         double qd = (current.getJointPosition(i) - previous.getJointPosition(i)) / duration;
         if (!Double.isFinite(qd))
            qd = 0.0;
         desiredJointVelocities[i].set(qd);
      }

      desiredRootJointVelocity.getLinearPart().sub(current.getRootJointPosition(), current.getRootJointPosition());
      desiredRootJointVelocity.getLinearPart().scale(1.0 / duration);
      if (desiredRootJointVelocity.containsNaN())
         desiredRootJointVelocity.setToZero();
      else
         desiredRootJointPose.getOrientation().inverseTransform(desiredRootJointVelocity.getLinearPart());
      quaternionDot.sub(current.getRootJointOrientation(), previous.getRootJointOrientation());
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

   @Override
   public int getJointNameHash()
   {
      return jointNameHash.getValue();
   }

   @Override
   public boolean hasAccelerationData()
   {
      return desiredJointAccelerations != null;
   }

   @Override
   public Point3DBasics getRootJointPosition()
   {
      return desiredRootJointPose.getPosition();
   }

   @Override
   public QuaternionBasics getRootJointOrientation()
   {
      return desiredRootJointPose.getOrientation();
   }

   @Override
   public Vector3DBasics getRootJointLinearVelocity()
   {
      return desiredRootJointVelocity.getLinearPart();
   }

   @Override
   public Vector3DBasics getRootJointAngularVelocity()
   {
      return desiredRootJointVelocity.getAngularPart();
   }

   @Override
   public Vector3DBasics getRootJointLinearAcceleration()
   {
      if (desiredRootJointAcceleration == null)
         return null;
      return desiredRootJointAcceleration.getLinearPart();
   }

   @Override
   public Vector3DBasics getRootJointAngularAcceleration()
   {
      if (desiredRootJointAcceleration == null)
         return null;
      return desiredRootJointAcceleration.getAngularPart();
   }

   @Override
   public int getNumberOfJoints()
   {
      return numberOfJoints;
   }

   @Override
   public double getJointPosition(int jointIndex)
   {
      return desiredJointAngles[jointIndex].getValue();
   }

   @Override
   public double getJointVelocity(int jointIndex)
   {
      return desiredJointVelocities[jointIndex].getValue();
   }

   @Override
   public double getJointAcceleration(int jointIndex)
   {
      if (desiredJointAccelerations == null)
         return 0.0;
      return desiredJointAccelerations[jointIndex].getValue();
   }

   @Override
   public void setJointPosition(int jointIndex, double position)
   {
      desiredJointAngles[jointIndex].set(position);
   }

   @Override
   public void setJointVelocity(int jointIndex, double velocity)
   {
      desiredJointVelocities[jointIndex].set(velocity);
   }

   @Override
   public void setJointAcceleration(int jointIndex, double acceleration)
   {
      if (desiredJointAccelerations == null)
         return;
      desiredJointAccelerations[jointIndex].set(acceleration);
   }
}
