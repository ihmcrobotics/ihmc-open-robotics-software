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
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialAcceleration;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
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
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
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

   private final YoFramePose3D rootJointPose;
   private final YoDouble[] jointAngles;

   private final YoFixedFrameTwist rootJointVelocity;
   private final YoDouble[] jointVelocities;

   private YoFixedFrameSpatialAcceleration rootJointAcceleration;
   private YoDouble[] jointAccelerations;

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
      jointAngles = new YoDouble[numberOfJoints];
      jointVelocities = new YoDouble[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         String jointName = oneDoFJoints[i].getName();
         jointAngles[i] = new YoDouble("q_d_" + namePrefix + "_" + jointName, registry);
         jointVelocities[i] = new YoDouble("qd_d_" + namePrefix + "_" + jointName, registry);
      }

      rootJointPose = new YoFramePose3D(namePrefix + "Desired" + rootJoint.getName(), worldFrame, registry);
      rootJointVelocity = new YoFixedFrameTwist(namePrefix + "DesiredVelocity" + rootJoint.getName(),
                                                rootJoint.getFrameAfterJoint(),
                                                worldFrame,
                                                rootJoint.getFrameAfterJoint(),
                                                registry);
   }

   public void createAccelerationState()
   {
      rootJointAcceleration = new YoFixedFrameSpatialAcceleration(namePrefix + "DesiredAcceleration" + rootJoint.getName(),
                                                                  rootJoint.getFrameAfterJoint(),
                                                                  worldFrame,
                                                                  rootJoint.getFrameAfterJoint(),
                                                                  registry);
      jointAccelerations = new YoDouble[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         String jointName = oneDoFJoints[i].getName();
         jointAccelerations[i] = new YoDouble("qdd_d_" + namePrefix + "_" + jointName, registry);
      }
   }

   public void setToNaN()
   {
      currentToolboxState.set(null);
      for (YoDouble desiredJointAngle : jointAngles)
         desiredJointAngle.setToNaN();
      for (YoDouble desiredJointVelocity : jointVelocities)
         desiredJointVelocity.setToNaN();
      rootJointPose.setToNaN();
      rootJointVelocity.setToNaN();

      if (jointAccelerations != null)
      {
         for (YoDouble desiredJointAcceleration : jointAccelerations)
            desiredJointAcceleration.setToNaN();
         rootJointAcceleration.setToNaN();
      }
   }

   public void set(YoKinematicsToolboxOutputStatus other)
   {
      if (jointNameHash.getValue() != other.jointNameHash.getValue())
         throw new IllegalArgumentException("Incompatible status");

      currentToolboxState.set(other.currentToolboxState.getEnumValue());

      for (int i = 0; i < numberOfJoints; i++)
      {
         jointAngles[i].set(other.jointAngles[i].getValue());
         jointVelocities[i].set(other.jointVelocities[i].getValue());
      }
      rootJointPose.set(other.rootJointPose);
      rootJointVelocity.set(other.rootJointVelocity);
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
         jointVelocities[i].set(jointVelocities[i].getValue() * scaleFactor);
      }
      rootJointVelocity.scale(scaleFactor);
   }

   private final Vector4D quaternionDot = new Vector4D();
   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   public void setVelocitiesByFiniteDifference(KSTOutputDataReadOnly previous, KSTOutputDataReadOnly current, double duration)
   {
      if (previous.getJointNameHash() != current.getJointNameHash() || previous.getJointNameHash() != jointNameHash.getValue())
         throw new RuntimeException("Output status are not compatible.");

      for (int i = 0; i < numberOfJoints; i++)
      {
         double qd = (current.getJointPosition(i) - previous.getJointPosition(i)) / duration;
         if (!Double.isFinite(qd))
            qd = 0.0;
         jointVelocities[i].set(qd);
      }

      rootJointVelocity.getLinearPart().sub(current.getRootJointPosition(), current.getRootJointPosition());
      rootJointVelocity.getLinearPart().scale(1.0 / duration);
      if (rootJointVelocity.containsNaN())
         rootJointVelocity.setToZero();
      else
         rootJointPose.getOrientation().inverseTransform((Vector3DBasics) rootJointVelocity.getLinearPart());
      quaternionDot.sub(current.getRootJointOrientation(), previous.getRootJointOrientation());
      quaternionDot.scale(1.0 / duration);
      if (quaternionDot.containsNaN())
      {
         quaternionDot.setToZero();
         rootJointVelocity.setToZero();
      }
      else
      {
         quaternionCalculus.computeAngularVelocityInBodyFixedFrame(rootJointPose.getOrientation(), quaternionDot, rootJointVelocity.getAngularPart());
      }
   }

   private MultiBodySystemStateIntegrator integrator;

   public void integrate(double dt)
   {
      if (integrator == null)
         integrator = new MultiBodySystemStateIntegrator();
      integrator.setIntegrationDT(dt);

      if (!hasAccelerationData())
      {
         for (int i = 0; i < getNumberOfJoints(); i++)
         {
            jointAngles[i].add(jointVelocities[i].getValue() * dt);
         }

         integrator.integrate(rootJointVelocity.getAngularPart(), rootJointPose.getOrientation(), rootJointPose.getOrientation());
         integrator.integrate(rootJointPose.getOrientation(), rootJointVelocity.getLinearPart(), rootJointPose.getPosition(), rootJointPose.getPosition());
      }
      else
      {
         for (int i = 0; i < getNumberOfJoints(); i++)
         {
            jointAngles[i].add(jointVelocities[i].getValue() * dt + 0.5 * dt * dt * jointAccelerations[i].getValue());
            jointVelocities[i].add(jointAccelerations[i].getValue() * dt);
         }

         integrator.doubleIntegrate(rootJointAcceleration, rootJointVelocity, rootJointPose);
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
      return jointAccelerations != null;
   }

   @Override
   public Point3DBasics getRootJointPosition()
   {
      return rootJointPose.getPosition();
   }

   @Override
   public QuaternionBasics getRootJointOrientation()
   {
      return rootJointPose.getOrientation();
   }

   @Override
   public Vector3DBasics getRootJointLinearVelocity()
   {
      return rootJointVelocity.getLinearPart();
   }

   @Override
   public Vector3DBasics getRootJointAngularVelocity()
   {
      return rootJointVelocity.getAngularPart();
   }

   @Override
   public Vector3DBasics getRootJointLinearAcceleration()
   {
      if (rootJointAcceleration == null)
         return null;
      return rootJointAcceleration.getLinearPart();
   }

   @Override
   public Vector3DBasics getRootJointAngularAcceleration()
   {
      if (rootJointAcceleration == null)
         return null;
      return rootJointAcceleration.getAngularPart();
   }

   @Override
   public int getNumberOfJoints()
   {
      return numberOfJoints;
   }

   @Override
   public double getJointPosition(int jointIndex)
   {
      return jointAngles[jointIndex].getValue();
   }

   @Override
   public double getJointVelocity(int jointIndex)
   {
      return jointVelocities[jointIndex].getValue();
   }

   @Override
   public double getJointAcceleration(int jointIndex)
   {
      if (jointAccelerations == null)
         return 0.0;
      return jointAccelerations[jointIndex].getValue();
   }

   @Override
   public void setJointPosition(int jointIndex, double position)
   {
      jointAngles[jointIndex].set(position);
   }

   @Override
   public void setJointVelocity(int jointIndex, double velocity)
   {
      jointVelocities[jointIndex].set(velocity);
   }

   @Override
   public void setJointAcceleration(int jointIndex, double acceleration)
   {
      if (jointAccelerations == null)
         return;
      jointAccelerations[jointIndex].set(acceleration);
   }
}
