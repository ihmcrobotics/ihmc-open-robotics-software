package us.ihmc.SdfLoader;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.FingerName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.PinJoint;

public class SDFFullRobotModel implements FullRobotModel
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBody elevator;
   private final SixDoFJoint rootJoint;

   private final ArrayList<RevoluteJoint> revoluteJoints = new ArrayList<RevoluteJoint>();
   
   public SDFFullRobotModel(SDFLinkHolder rootLink)
   {
      /*
       * Create root object
       */
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      elevator = new RigidBody("elevator", elevatorFrame);
      rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);

      /*
       * Add joints
       */
      RigidBody rootBody = ScrewTools.addRigidBody(rootLink.getName(), rootJoint, rootLink.getInertia(), rootLink.getMass(), rootLink.getCoMOffset());

      for (SDFJointHolder sdfJoint : rootLink.getChilderen())
      {
         addJointsRecursively(sdfJoint, rootBody);
      }
   }

   private void addJointsRecursively(SDFJointHolder joint, RigidBody parentBody)
   {
      System.out.println("Adding joint " + joint.getName() + " to " + parentBody.getName());
      RevoluteJoint inverseDynamicsJoint = ScrewTools.addRevoluteJoint(joint.getName(), parentBody, joint.getTransformFromParentJoint(), joint.getAxis());
      SDFLinkHolder childLink = joint.getChild();
      RigidBody rigidBody = ScrewTools.addRigidBody(childLink.getName(), inverseDynamicsJoint, childLink.getInertia(), childLink.getMass(),
            childLink.getCoMOffset());
      System.out.println("Adding rigid body " + childLink.getName() + ", ixx: " + childLink.getInertia().m00 + "; iyy + " + childLink.getInertia().m11 + "; izz: " + childLink.getInertia().m22);
      
      revoluteJoints.add(inverseDynamicsJoint);

      for (SDFJointHolder sdfJoint : childLink.getChilderen())
      {
         addJointsRecursively(sdfJoint, rigidBody);
      }

   }

   public RobotSpecificJointNames getRobotSpecificJointNames()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public void updateFrames()
   {
      elevator.updateFramesRecursively();
   }

   public ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public ReferenceFrame getElevatorFrame()
   {
      return elevator.getBodyFixedFrame();
   }

   public ReferenceFrame getFrameAfterLegJoint(RobotSide robotSide, LegJointName legJointName)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public SixDoFJoint getRootJoint()
   {
      return rootJoint;
   }

   public RigidBody getElevator()
   {
      return elevator;
   }

   public RevoluteJoint getLegJoint(RobotSide robotSide, LegJointName legJointName)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RevoluteJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RevoluteJoint[] getLegJointList(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RevoluteJoint[] getArmJointList(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RevoluteJoint[] getSpineJointList()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RevoluteJoint[] getNeckJointList()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public LegJointVelocities getLegJointVelocities(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RigidBody getPelvis()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RigidBody getFoot(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RigidBody getHand(RobotSide robotSide)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RigidBody getChest()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public void setTorques(ProcessedOutputsInterface processedOutputs)
   {
      // TODO Auto-generated method stub

   }

   public RigidBody getEndEffector(RobotSide robotSide, LimbName limbName)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public ReferenceFrame getEndEffectorFrame(RobotSide robotSide, LimbName limbName)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public FramePoint getStaticWristToFingerOffset(RobotSide robotSide, FingerName fingerName)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RigidBody[] getLowerBodyRigidBodiesInOrder()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RigidBody[] getUpperBodyRigidBodiesInOrder()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public RevoluteJoint[] getRevoluteJoints()
   {
      // TODO Auto-generated method stub
      return null;
   }
   
   public void setJointAnglesAndVelocitiesBasedOnRobot(SDFRobot robot)
   {
      for(RevoluteJoint joint : revoluteJoints)
      {
         PinJoint robotJoint = robot.getJoint(joint.getName());
         
         joint.setQ(robotJoint.getQ().getDoubleValue());
         joint.setQd(robotJoint.getQD().getDoubleValue());
         joint.setQdd(robotJoint.getQDD().getDoubleValue());
      }
   }

}
