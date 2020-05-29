package us.ihmc.robotDataVisualizer.visualizer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.jointState.OneDoFState;
import us.ihmc.robotDataLogger.jointState.SixDoFState;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public abstract class JointUpdater
{

   public abstract void update();
   
   public static void getJointUpdaterList(List<Joint> rootJoints, List<JointState> jointStates, List<JointUpdater> jointUpdatersToPack)
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
      
      for (JointState jointState : jointStates)
      {
         Joint joint = joints.get(jointState.getName());
         if (joint == null)
         {
            LogTools.error("Cannot find joint " + jointState.getName());

            continue;
         }
         
         switch(jointState.getType())
         {
         case OneDoFJoint:
            jointUpdatersToPack.add(new OneDegreeOfFreedomJointUpdater((OneDegreeOfFreedomJoint) joint, (OneDoFState) jointState));
            
            break;
         case SiXDoFJoint:
            jointUpdatersToPack.add(new SixDoFJointUpdater((FloatingJoint) joint, (SixDoFState)jointState));
            break;
         default:
            throw new RuntimeException("Unimplemented joint type " + jointState.getType());
         }
         
      }
   }
}