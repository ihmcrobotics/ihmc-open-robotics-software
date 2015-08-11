package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.LinkedHashMap;

public class TwistCalculator 
{
   private final RigidBody rootBody;
   private final Twist rootTwist;

   private final LinkedHashMap<RigidBody, Twist> twists = new LinkedHashMap<RigidBody, Twist>();
   private final ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
   private final Twist tempTwist = new Twist();

   public TwistCalculator(ReferenceFrame inertialFrame, RigidBody body)
   {
      this.rootBody = addAllPrecedingJoints(inertialFrame, body);
      this.rootTwist = new Twist(rootBody.getBodyFixedFrame(), inertialFrame, rootBody.getBodyFixedFrame());
      addAllSucceedingJoints(inertialFrame, body);
      populateMapsAndLists(inertialFrame);
   }

   private void addAllSucceedingJoints(ReferenceFrame inertialFrame, RigidBody body)
   {
      for (InverseDynamicsJoint inverseDynamicsJoint : body.getChildrenJoints())
      {
         if (inverseDynamicsJoint.getSuccessor() != null)
         {
            allJoints.add(inverseDynamicsJoint);
            addAllSucceedingJoints(inertialFrame, inverseDynamicsJoint.getSuccessor());
         }
      }
   }

   private RigidBody addAllPrecedingJoints(ReferenceFrame inertialFrame, RigidBody body)
   {
      ArrayList<InverseDynamicsJoint> jointsTillRootBody = new ArrayList<InverseDynamicsJoint>();

      RigidBody currentBody = body;
      while (currentBody.getParentJoint() != null)
      {
         jointsTillRootBody.add(currentBody.getParentJoint());
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      for (int i = jointsTillRootBody.size() - 1; i >= 0; i--)
      {
         allJoints.add(jointsTillRootBody.get(i));
      }

      return currentBody;
   }

   public void compute()
   {
      twists.get(rootBody).set(rootTwist);

      for (int jointIndex = 0; jointIndex < allJoints.size(); jointIndex++)
      {
         InverseDynamicsJoint joint = allJoints.get(jointIndex);
         computeSuccessorTwist(joint);
      }
   }

   private void computeSuccessorTwist(InverseDynamicsJoint joint)
   {
      RigidBody predecessor = joint.getPredecessor();
      RigidBody successor = joint.getSuccessor();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      joint.packSuccessorTwist(tempTwist);

      Twist successorTwist = twists.get(successor);
      successorTwist.set(twists.get(predecessor));
      successorTwist.changeFrame(successorFrame);
      successorTwist.add(tempTwist);
   }

   public void packTwistOfBody(Twist twistToPack, RigidBody rigidBody)
   {
      twistToPack.set(twists.get(rigidBody));
   }

   public void packRelativeTwist(Twist twistToPack, RigidBody base, RigidBody body)
   {
      twistToPack.set(twists.get(body));
      tempTwist.set(twists.get(base));
      tempTwist.changeFrame(twistToPack.getExpressedInFrame());
      twistToPack.sub(tempTwist);
   }

   public RigidBody getRootBody()
   {
      return rootBody;
   }

   private void populateMapsAndLists(ReferenceFrame inertialFrame)
   {
      addTwist(inertialFrame, rootBody);
      
      for (InverseDynamicsJoint joint : allJoints)
      {
         if (joint.getSuccessor() != null)
         {
            addTwist(inertialFrame, joint.getSuccessor());
         }
      }
   }
   
   private void addTwist(ReferenceFrame inertialFrame, RigidBody body)
   {
   	ReferenceFrame bodyFixedFrame = body.getBodyFixedFrame();
      twists.put(body, new Twist(bodyFixedFrame, inertialFrame, bodyFixedFrame));
   }
}
