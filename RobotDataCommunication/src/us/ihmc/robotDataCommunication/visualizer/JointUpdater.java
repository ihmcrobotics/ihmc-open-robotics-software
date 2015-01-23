package us.ihmc.robotDataCommunication.visualizer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.simulationconstructionset.Joint;

@SuppressWarnings("rawtypes") 
public class JointUpdater
{
   private final Joint joint;
   private final JointState jointState;

   public JointUpdater(Joint joint, JointState jointState)
   {
      this.joint = joint;
      this.jointState = jointState;
   }

   @SuppressWarnings("unchecked")
   public void update()
   {
      jointState.get(joint);
   }
   
   public static void getJointUpdaterList(ArrayList<Joint> rootJoints, List<JointState<? extends Joint>> jointStates, ArrayList<JointUpdater> jointUpdatersToPack)
   {
      HashMap<String, Joint> joints = new HashMap<String, Joint>();
      for (Joint joint : rootJoints)
      {
         joints.put(joint.getName(), joint);
   
         ArrayList<Joint> childeren = new ArrayList<Joint>();
         joint.recursiveGetChildrenJoints(childeren);
   
         for (Joint child : childeren)
         {
            joints.put(child.getName(), child);
         }
      }
      
      for (JointState<?> jointState : jointStates)
      {
         Joint joint = joints.get(jointState.getName());
         if (joint == null)
         {
            System.err.println("Cannot find joint " + jointState.getName());

            continue;
         }

         jointUpdatersToPack.add(new JointUpdater(joint, jointState));
      }
   }
}