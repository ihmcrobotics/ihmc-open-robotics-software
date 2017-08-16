package us.ihmc.exampleSimulations.sphereICPControl.model;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class SphereRobotModel implements FullRobotModel
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double mass = 1.0;
   private static final double
         Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;

   private final RigidBody elevator;
   private final RigidBody body;

   private final SixDoFJoint floatingJoint;
   private final OneDoFJoint[] oneDoFJoints;

   private final ReferenceFrame centerOfMassFrame;

   private final CenterOfMassJacobian centerOfMassJacobian;

   private final double totalMass;

   public SphereRobotModel()
   {
      elevator = new RigidBody("elevator", worldFrame);

      floatingJoint = new SixDoFJoint("floatingJoint", elevator);

      Matrix3D inertia = new Matrix3D(Ixx1, 0.0, 0.0, 0.0, Iyy1, 0.0, 0.0, 0.0, Izz1);
      body = ScrewTools.addRigidBody("body", floatingJoint, inertia, mass, new Vector3D());

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

      centerOfMassJacobian = new CenterOfMassJacobian(elevator);

      oneDoFJoints = ScrewTools.createOneDoFJointPath(elevator, body);
      totalMass = TotalMassCalculator.computeSubTreeMass(body);
   }

   public double getTotalMass()
   {
      return totalMass;
   }

   public ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public RigidBody getElevator()
   {
      return elevator;
   }

   public SixDoFJoint getRootJoint()
   {
      return floatingJoint;
   }

   public void updateFrames()
   {
      elevator.updateFramesRecursively();
      centerOfMassFrame.update();
   }

   @Override
   public MovingReferenceFrame getElevatorFrame()
   {
      return null;
   }

   public OneDoFJoint[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
   {
      return null;
   }

   @Override public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
   {

   }

   public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
   {
      List<OneDoFJoint> list = Arrays.asList(oneDoFJoints);

      for (int i = 0; i < list.size(); i++)
         oneDoFJointsToPack.set(i, list.get(i));
   }

   @Override public OneDoFJoint getOneDoFJointByName(String name)
   {
      return null;
   }

   @Override
   public OneDoFJoint[] getControllableOneDoFJoints()
   {
      return null;
   }

   @Override
   public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
   {
   }

   public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
   {
      return null;
   }

   @Override public RigidBody getEndEffector(Enum<?> segmentEnum)
   {
      return null;
   }

   public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
   {
      return null;
   }

   public InverseDynamicsJoint getLidarJoint(String lidarName)
   {
      return null;
   }

   @Override public ReferenceFrame getLidarBaseFrame(String name)
   {
      return null;
   }

   @Override public RigidBodyTransform getLidarBaseToSensorTransform(String name)
   {
      return null;
   }

   @Override public ReferenceFrame getCameraFrame(String name)
   {
      return null;
   }

   public RigidBody getPelvis()
   {
      return null;
   }

   public RigidBody getChest()
   {
      return null;
   }

   public RigidBody getHead()
   {
      return null;
   }

   public ReferenceFrame getHeadBaseFrame()
   {
      return null;
   }

   public RobotSpecificJointNames getRobotSpecificJointNames()
   {
      return null;
   }

   public IMUDefinition[] getIMUDefinitions()
   {
      return null;
   }

   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      return null;
   }

   public ContactSensorDefinition[] getContactSensorDefinitions()
   {
      return null;
   }
}
