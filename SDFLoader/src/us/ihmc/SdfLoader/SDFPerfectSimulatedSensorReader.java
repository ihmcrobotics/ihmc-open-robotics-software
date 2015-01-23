package us.ihmc.SdfLoader;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.robotController.RawSensorReader;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class SDFPerfectSimulatedSensorReader implements RawSensorReader
{
   private final String name;
   private final SDFRobot robot;
   private final SixDoFJoint rootJoint;
   private final CommonHumanoidReferenceFrames referenceFrames;

   private final ArrayList<Pair<OneDegreeOfFreedomJoint, OneDoFJoint>> revoluteJoints = new ArrayList<Pair<OneDegreeOfFreedomJoint, OneDoFJoint>>();

   
   
   public SDFPerfectSimulatedSensorReader(SDFRobot robot, FullRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames)
   {
      this(robot, fullRobotModel.getRootJoint(), referenceFrames);
   }
   
   public SDFPerfectSimulatedSensorReader(SDFRobot robot, SixDoFJoint rootJoint, CommonHumanoidReferenceFrames referenceFrames)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
      this.referenceFrames = referenceFrames;

      this.rootJoint = rootJoint;

      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor());

      for (InverseDynamicsJoint joint : jointsArray)
      {
         
         if(joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            String name = oneDoFJoint.getName();
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = robot.getOneDegreeOfFreedomJoint(name);
   
            Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = new Pair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDegreeOfFreedomJoint, oneDoFJoint);
            this.revoluteJoints.add(jointPair);
         }
      }

   }

   public void initialize()
   {
      read();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   private RigidBodyTransform temporaryRootToWorldTransform = new RigidBodyTransform();

   public void read()
   {
      // Think about adding root body acceleration to the fullrobotmodel
      readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations();
      readAndUpdateRootJointPositionAndOrientation();
      updateReferenceFrames();
      readAndUpdateRootJointAngularAndLinearVelocity();
   }

   private void readAndUpdateRootJointAngularAndLinearVelocity()
   {
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame pelvisFrame = rootJoint.getFrameAfterJoint();

      FrameVector linearVelocity = robot.getRootJointVelocity();
      linearVelocity.changeFrame(pelvisFrame);

      FrameVector angularVelocity = robot.getPelvisAngularVelocityInPelvisFrame(pelvisFrame);
      angularVelocity.changeFrame(pelvisFrame);

      Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, linearVelocity.getVector(), angularVelocity.getVector());
      rootJoint.setJointTwist(bodyTwist);
   }

   private void updateReferenceFrames()
   {
      if (referenceFrames != null)
      {
         referenceFrames.updateFrames();
      }
      else
      {
         rootJoint.getPredecessor().updateFramesRecursively();
      }
   }

   private void readAndUpdateRootJointPositionAndOrientation()
   {
      packRootTransform(robot, temporaryRootToWorldTransform);
      temporaryRootToWorldTransform.normalize();
      rootJoint.setPositionAndRotation(temporaryRootToWorldTransform);
   }

   private void readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations()
   {
      for (Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();

         revoluteJoint.setQ(pinJoint.getQ().getDoubleValue());
         revoluteJoint.setQd(pinJoint.getQD().getDoubleValue());
         revoluteJoint.setQdd(pinJoint.getQDD().getDoubleValue());

      }
   }

   protected void packRootTransform(SDFRobot robot, RigidBodyTransform transformToPack)
   {
      robot.getRootJointToWorldTransform(transformToPack);
   }
}
