package us.ihmc.avatar.initialSetup;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class HumanoidRobotInitialSetup implements RobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   /** Default height of the pelvis above the ground when lying down, used by {@link PoseType#GROUND_PRONE} and {@link PoseType#GROUND_SUPINE}. */
   public static final double DEFAULT_GROUND_PELVIS_HEIGHT = 0.2;

   /**
    * Describes the high-level pose the robot should be initialized in.
    */
   public enum PoseType
   {
      /** Robot is standing upright on its feet. */
      STANDING,
      /** Robot is lying face-down on the ground. */
      GROUND_PRONE,
      /** Robot is lying face-up on the ground. */
      GROUND_SUPINE
   }

   protected double initialYaw = 0.0;
   protected double initialGroundHeight = 0.0;
   protected final Vector3D additionalOffset = new Vector3D();

   protected final Point3D rootJointPosition = new Point3D();
   protected final Quaternion rootJointOrientation = new Quaternion();
   protected final Vector3D rootJointAngularVelocityInBody = new Vector3D();
   protected final Vector3D rootJointLinearVelocityInWorld = new Vector3D();
   protected final Map<String, Double> jointPositions = new HashMap<>();
   protected final HumanoidJointNameMap jointMap;

   protected PoseType poseType = PoseType.STANDING;
   protected double groundPelvisHeight = DEFAULT_GROUND_PELVIS_HEIGHT;

   public HumanoidRobotInitialSetup(HumanoidJointNameMap jointMap)
   {
      this.jointMap = jointMap;
   }

   /**
    * Template method that configures this initial setup for {@link #getPoseType()}: first the joint positions via
    * {@link #configureJointPositions()}, then the root joint pose via {@link #configureRootPose(RobotDefinition)}.
    */
   public void configureInitialSetup(RobotDefinition robotDefinition)
   {
      configureJointPositions();
      configureRootPose(robotDefinition);
   }

   /**
    * Hook for subclasses to set up the joint positions for {@link #getPoseType()}. Default implementation does nothing.
    */
   protected void configureJointPositions()
   {
   }

   /**
    * Configures the root joint position/orientation based on {@link #getPoseType()}. Standing robots are placed so
    * their lowest sole is at the ground, while ground poses place the pelvis {@link #getGroundPelvisHeight()} above
    * the ground with the robot lying on its front or back.
    */
   protected void configureRootPose(RobotDefinition robotDefinition)
   {
      switch (poseType)
      {
         case STANDING -> setRootJointHeightSuchThatLowestSoleIsAtZero(robotDefinition);
         case GROUND_PRONE -> setGroundRootPose(Math.PI / 2.0);
         case GROUND_SUPINE -> setGroundRootPose(-Math.PI / 2.0);
      }
   }

   private void setGroundRootPose(double pitch)
   {
      rootJointPosition.set(0.0, 0.0, groundPelvisHeight);
      rootJointOrientation.setYawPitchRoll(0.0, pitch, 0.0);
   }

   public void setJoint(RobotSide robotSide, LegJointName legJointName, double q)
   {
      String jointName = jointMap.getLegJointName(robotSide, legJointName);
      if (jointName != null)
         setJoint(jointName, q);
   }

   public void setJoint(RobotSide robotSide, ArmJointName armJointName, double q)
   {
      String jointName = jointMap.getArmJointName(robotSide, armJointName);
      if (jointName != null)
         setJoint(jointName, q);
   }

   public void setJoint(SpineJointName spineJointName, double q)
   {
      String jointName = jointMap.getSpineJointName(spineJointName);
      if (jointName != null)
         setJoint(jointName, q);
   }

   public void setJoint(NeckJointName neckJointName, double q)
   {
      String jointName = jointMap.getNeckJointName(neckJointName);
      if (jointName != null)
         setJoint(jointName, q);
   }

   public void setJoint(String jointName, double q)
   {
      jointPositions.put(jointName, q);
   }

   public double getHeightOfPelvisAboveLowestSoleZ(RobotDefinition robotDefinition)
   {
      RigidBodyBasics rootBody = robotDefinition.newInstance(ReferenceFrameTools.constructARootFrame("temp"));
      initializeRobot(rootBody, false);
      rootBody.updateFramesRecursively();

      double pelvisToLowestSoleZ = Double.NEGATIVE_INFINITY;
      RigidBodyTransform tempAnkleFrameToTempRootFrame = new RigidBodyTransform();
      RigidBodyTransform tempSoleFrameToTempRootFrame = new RigidBodyTransform();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = MultiBodySystemTools.findRigidBody(rootBody, jointMap.getFootName(robotSide));
         if (foot == null)
            continue;
         tempAnkleFrameToTempRootFrame.set(foot.getParentJoint().getFrameAfterJoint().getTransformToRoot());
         tempSoleFrameToTempRootFrame.set(tempAnkleFrameToTempRootFrame);
         tempSoleFrameToTempRootFrame.multiply(jointMap.getSoleToParentFrameTransform(robotSide));
         double pelvisToSoleZ = -tempSoleFrameToTempRootFrame.getTranslationZ();
         pelvisToLowestSoleZ = Math.max(pelvisToSoleZ, pelvisToLowestSoleZ);
      }

      return pelvisToLowestSoleZ;
   }

   public void setRootJointHeightSuchThatLowestSoleIsAtZero(RobotDefinition robotDefinition)
   {
      double farthestSoleToPelvisDistance = getHeightOfPelvisAboveLowestSoleZ(robotDefinition);

      if (Double.isFinite(farthestSoleToPelvisDistance))
         rootJointPosition.setZ(farthestSoleToPelvisDistance);
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot)
   {
      for (OneDegreeOfFreedomJoint joint : robot.getOneDegreeOfFreedomJoints())
      {
         Double jointPosition = getJointPosition(joint.getName());

         if (jointPosition != null)
         {
            joint.setQ(jointPosition);
         }
      }

      robot.getRootJoint().getPosition().set(rootJointPosition);
      robot.getRootJoint().getPosition().add(additionalOffset);
      robot.getRootJoint().getPosition().addZ(initialGroundHeight);
      robot.getRootJoint().setOrientation(rootJointOrientation);
      robot.getRootJoint().getOrientation().prependYawRotation(initialYaw);

      robot.getRootJoint().setVelocity(rootJointLinearVelocityInWorld);
      robot.getRootJoint().setAngularVelocityInBody(rootJointAngularVelocityInBody);

      robot.update();
   }

   @Override
   public void initializeFullRobotModel(FullHumanoidRobotModel fullRobotModel)
   {
      initializeRobot(fullRobotModel.getElevator());
   }

   @Override
   public void initializeRobot(RigidBodyBasics rootBody)
   {
      initializeRobot(rootBody, true);
   }

   private void initializeRobot(RigidBodyBasics rootBody, boolean applyRootJointPose)
   {
      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            Double jointPosition = getJointPosition(joint.getName());

            if (jointPosition != null)
            {
               ((OneDoFJointBasics) joint).setQ(jointPosition);
            }
         }
      }

      /* Root joint position */
      if (applyRootJointPose && rootBody.getChildrenJoints().size() == 1)
      {
         JointBasics rootJoint = rootBody.getChildrenJoints().get(0);

         if (rootJoint instanceof FloatingJointBasics floatingJoint)
         {
            Pose3DBasics jointPose = floatingJoint.getJointPose();
            jointPose.getPosition().set(rootJointPosition);
            jointPose.getPosition().add(additionalOffset);
            jointPose.getPosition().addZ(initialGroundHeight);
            jointPose.getOrientation().set(rootJointOrientation);
            jointPose.getOrientation().prependYawRotation(initialYaw);
         }
      }

      /* Root joint velocity */
      if (rootBody.getChildrenJoints().size() == 1)
      {
         JointBasics rootJoint = rootBody.getChildrenJoints().get(0);

         if (rootJoint instanceof FloatingJointBasics floatingJoint)
         {
            Pose3DBasics jointPose = floatingJoint.getJointPose();
            FixedFrameTwistBasics jointTwist = floatingJoint.getJointTwist();
            jointTwist.getAngularPart().set(rootJointAngularVelocityInBody);
            jointPose.getOrientation().inverseTransform(rootJointLinearVelocityInWorld, jointTwist.getLinearPart());
         }
      }
   }

   @Override
   public void initializeRobotDefinition(RobotDefinition robotDefinition)
   {
      SixDoFJointDefinition rootJoint = robotDefinition.getFloatingRootJointDefinition();
      if (rootJoint != null)
      {
         Point3D position = new Point3D(rootJointPosition);
         position.add(additionalOffset);
         position.addZ(initialGroundHeight);
         Quaternion orientation = new Quaternion(rootJointOrientation);
         orientation.prependYawRotation(initialYaw);
         SixDoFJointState rootJointState = new SixDoFJointState(orientation, position);
         rootJointState.setVelocity(EuclidCoreTools.zeroVector3D, EuclidCoreTools.zeroVector3D);
         rootJointState.setAcceleration(EuclidCoreTools.zeroVector3D, EuclidCoreTools.zeroVector3D);
         rootJoint.setInitialJointState(rootJointState);
      }

      robotDefinition.forEachOneDoFJointDefinition(jointDefinition ->
      {
         Double jointPosition = getJointPosition(jointDefinition.getName());

         if (jointPosition != null)
            jointDefinition.setInitialJointState(new OneDoFJointState(jointPosition, 0.0, 0.0));
         else
            jointDefinition.setInitialJointState(new OneDoFJointState(0.0, 0.0, 0.0));
      });
   }

   @Override
   public void setInitialYaw(double yaw)
   {
      initialYaw = yaw;
   }

   @Override
   public double getInitialYaw()
   {
      return initialYaw;
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
      initialGroundHeight = groundHeight;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return initialGroundHeight;
   }

   @Override
   public void setOffset(Tuple3DReadOnly additionalOffset)
   {
      this.additionalOffset.set(additionalOffset);
   }

   @Override
   public Vector3D getOffset()
   {
      return additionalOffset;
   }

   public Double getJointPosition(String jointName)
   {
      return jointPositions.get(jointName);
   }

   public boolean hasJointPosition(String jointName)
   {
      return jointPositions.containsKey(jointName);
   }

   public Map<String, Double> getJointPositions()
   {
      return jointPositions;
   }

   public void setPoseType(PoseType poseType)
   {
      this.poseType = poseType;
   }

   public PoseType getPoseType()
   {
      return poseType;
   }

   public void setGroundPelvisHeight(double groundPelvisHeight)
   {
      this.groundPelvisHeight = groundPelvisHeight;
   }

   public double getGroundPelvisHeight()
   {
      return groundPelvisHeight;
   }

   public void setRootJointPosition(Tuple3DReadOnly rootJointPosition)
   {
      this.rootJointPosition.set(rootJointPosition);
   }

   public Point3D getRootJointPosition()
   {
      return rootJointPosition;
   }

   public void setRootJointOrientation(Orientation3DReadOnly rootJointOrientation)
   {
      this.rootJointOrientation.set(rootJointOrientation);
   }

   public Quaternion getRootJointOrientation()
   {
      return rootJointOrientation;
   }

   public Vector3D getRootJointAngularVelocityInBody()
   {
      return rootJointAngularVelocityInBody;
   }

   public Vector3D getRootJointLinearVelocityInWorld()
   {
      return rootJointLinearVelocityInWorld;
   }
   public HumanoidJointNameMap getJointMap()
   {
      return jointMap;
   }
}
