package us.ihmc.atlas.ros;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.avatar.handControl.FingerJoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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

   
   public static final SideDependentList<String[]> handNames = new SideDependentList<String[]>();

   static
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = robotSide.getLowerCaseName() + "_";
         String[] handNamesForSide = new String[numberOfJointsPerHand];
         handNamesForSide[f0_j0] = prefix + "f0_j0";
         handNamesForSide[f0_j1] = prefix + "f0_j1";
         handNamesForSide[f0_j2] = prefix + "f0_j2";
         handNamesForSide[f1_j0] = prefix + "f1_j0";
         handNamesForSide[f1_j1] = prefix + "f1_j1";
         handNamesForSide[f1_j2] = prefix + "f1_j2";
         handNamesForSide[f2_j0] = prefix + "f2_j0";
         handNamesForSide[f2_j1] = prefix + "f2_j1";
         handNamesForSide[f2_j2] = prefix + "f2_j2";
         handNamesForSide[f3_j0] = prefix + "f3_j0";
         handNamesForSide[f3_j1] = prefix + "f3_j1";
         handNamesForSide[f3_j2] = prefix + "f3_j2";
         
         handNames.put(robotSide, handNamesForSide);
      }
   }

   public static FingerJoint[] getFingerMap(RobotSide robotSide, ArrayList<FingerJoint> handJoints)
   {

      HashMap<String, FingerJoint> jointsByName = new HashMap<String, FingerJoint>();
      for (FingerJoint fingerJoint : handJoints)
      {
         jointsByName.put(fingerJoint.getName(), fingerJoint);
      }

      FingerJoint[] joints = new FingerJoint[numberOfJointsPerHand];
      
      for(int i = 0; i < numberOfJointsPerHand; i++)
      {
         joints[i] = jointsByName.get(handNames.get(robotSide)[i]);
      }

      return joints;
   }

}
