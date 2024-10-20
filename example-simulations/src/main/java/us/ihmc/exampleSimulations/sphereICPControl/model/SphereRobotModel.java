package us.ihmc.exampleSimulations.sphereICPControl.model;

import java.util.Map;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.mecano.MultiBodySystemMissingTools;
import us.ihmc.commons.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.commons.robotics.partNames.SpineJointName;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class SphereRobotModel implements FullRobotModel
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double mass = 1.0;
   private static final double Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;

   private final RigidBodyBasics elevator;
   private final RigidBodyBasics body;

   private final SixDoFJoint floatingJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final ReferenceFrame centerOfMassFrame;

   private final double totalMass;

   public SphereRobotModel()
   {
      elevator = new RigidBody("elevator", worldFrame);

      floatingJoint = new SixDoFJoint("floatingJoint", elevator);

      Matrix3D inertia = new Matrix3D(Ixx1, 0.0, 0.0, 0.0, Iyy1, 0.0, 0.0, 0.0, Izz1);
      body = new RigidBody("body", floatingJoint, inertia, mass, new Vector3D());

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

      oneDoFJoints = MultiBodySystemTools.createOneDoFJointPath(elevator, body);
      totalMass = MultiBodySystemMissingTools.computeSubTreeMass(body);
   }

   @Override
   public double getTotalMass()
   {
      return totalMass;
   }

   public ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   @Override
   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   @Override
   public SixDoFJoint getRootJoint()
   {
      return floatingJoint;
   }

   @Override
   public void updateFrames()
   {
      elevator.updateFramesRecursively();
      centerOfMassFrame.update();
   }

   @Override
   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   @Override
   public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
   {
      return null;
   }

   @Override
   public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
   {
      return null;
   }

   @Override
   public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
   {
      return null;
   }

   @Override
   public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
   {
      return null;
   }

   @Override
   public JointBasics getLidarJoint(String lidarName)
   {
      return null;
   }

   @Override
   public ReferenceFrame getLidarBaseFrame(String name)
   {
      return null;
   }

   @Override
   public RigidBodyTransform getLidarBaseToSensorTransform(String name)
   {
      return null;
   }

   @Override
   public ReferenceFrame getCameraFrame(String name)
   {
      return null;
   }

   @Override
   public RigidBodyBasics getRootBody()
   {
      return null;
   }

   @Override
   public RigidBodyBasics getHead()
   {
      return null;
   }

   @Override
   public RobotSpecificJointNames getRobotSpecificJointNames()
   {
      return null;
   }

   @Override
   public IMUDefinition[] getIMUDefinitions()
   {
      return null;
   }

   @Override
   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      return null;
   }
}
