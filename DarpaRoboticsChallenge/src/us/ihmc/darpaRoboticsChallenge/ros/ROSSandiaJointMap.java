package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.darpaRoboticsChallenge.handControl.FingerJoint;
import us.ihmc.robotSide.RobotSide;

public class ROSSandiaJointMap
{
   public final static int f0_j0 = 0;
   public final static int f0_j1 = 1;
   public final static int f0_j2 = 2;
   public final static int f1_j0 = 3;
   public final static int f1_j1 = 4;
   public final static int f1_j2 = 5;
   public final static int f2_j0 = 6;
   public final static int f2_j1 = 7;
   public final static int f2_j2 = 8;
   public final static int f3_j0 = 9;
   public final static int f3_j1 = 10;
   public final static int f3_j2 = 11;

   public final static int numberOfJointsPerHand = f3_j2 + 1;
   
   public static FingerJoint[] getFingerMap(RobotSide robotSide, ArrayList<FingerJoint> handJoints)
   {
      String prefix = robotSide.getLowerCaseName() + "_";
      
      HashMap<String, FingerJoint> jointsByName = new HashMap<String, FingerJoint>();
      for(FingerJoint fingerJoint : handJoints)
      {
         jointsByName.put(fingerJoint.getName(), fingerJoint);
      }
      
      FingerJoint[] joints = new FingerJoint[numberOfJointsPerHand];
      joints[f0_j0] = jointsByName.get(prefix + "f0_j0");
      joints[f0_j1] = jointsByName.get(prefix + "f0_j1");
      joints[f0_j2] = jointsByName.get(prefix + "f0_j2");
      joints[f1_j0] = jointsByName.get(prefix + "f1_j0");
      joints[f1_j1] = jointsByName.get(prefix + "f1_j1");
      joints[f1_j2] = jointsByName.get(prefix + "f1_j2");
      joints[f2_j0] = jointsByName.get(prefix + "f2_j0");
      joints[f2_j1] = jointsByName.get(prefix + "f2_j1");
      joints[f2_j2] = jointsByName.get(prefix + "f2_j2");
      joints[f3_j0] = jointsByName.get(prefix + "f3_j0");
      joints[f3_j1] = jointsByName.get(prefix + "f3_j1");
      joints[f3_j2] = jointsByName.get(prefix + "f3_j2");
      
      return joints;
   }
   
}
