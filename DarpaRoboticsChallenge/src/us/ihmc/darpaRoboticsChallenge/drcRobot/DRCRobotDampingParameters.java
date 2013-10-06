package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap;
import us.ihmc.darpaRoboticsChallenge.ros.ROSSandiaJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class DRCRobotDampingParameters
{
   private final static double[] atlasDampingParameters = new double[ROSAtlasJointMap.numberOfJoints];
   private final static double[] atlasPositionControlDampingParameters = new double[ROSAtlasJointMap.numberOfJoints];

   private final static SideDependentList<double[]> sandiaDampingParameters = new SideDependentList<double[]>();
   private static final boolean USE_REALLY_HIGH_FINGER_DAMPING=true;
   static
   {
      atlasDampingParameters[ROSAtlasJointMap.back_bkz]  = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.back_bky]  = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.back_bkx]  = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.neck_ry]   = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_hpz] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_hpx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_hpy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_kny] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_aky] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_akx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_hpz] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_hpx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_hpy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_kny] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_aky] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_akx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_shy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_shx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_ely] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_elx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_wry] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_wrx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_shy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_shx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_ely] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_elx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_wry] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_wrx] = 0.1;
      
      atlasPositionControlDampingParameters[ROSAtlasJointMap.back_bkz]  = 10.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.back_bky]  = 10.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.back_bkx]  = 10.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.neck_ry]   = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_leg_hpz] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_leg_hpx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_leg_hpy] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_leg_kny] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_leg_aky] = 10.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_leg_akx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_leg_hpz] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_leg_hpx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_leg_hpy] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_leg_kny] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_leg_aky] = 10.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_leg_akx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_arm_shy] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_arm_shx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_arm_ely] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_arm_elx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_arm_wry] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.l_arm_wrx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_arm_shy] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_arm_shx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_arm_ely] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_arm_elx] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_arm_wry] = 1.0;
      atlasPositionControlDampingParameters[ROSAtlasJointMap.r_arm_wrx] = 1.0;
      
      for(RobotSide robotSide : RobotSide.values)
      {
         double[] sandiaDampingParametersForSide = new double[ROSSandiaJointMap.numberOfJointsPerHand];
         double standardFingerDamping = USE_REALLY_HIGH_FINGER_DAMPING?5.0:1.0; //1.0;
//         double firstJointDamping = USE_REALLY_HIGH_FINGER_DAMPING?5.0:30.0;
         
         sandiaDampingParametersForSide[ROSSandiaJointMap.f0_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f0_j1] = standardFingerDamping;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f0_j2] = standardFingerDamping;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f1_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f1_j1] = standardFingerDamping;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f1_j2] = standardFingerDamping;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f2_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f2_j1] = standardFingerDamping;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f2_j2] = standardFingerDamping;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f3_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f3_j1] = standardFingerDamping;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f3_j2] = 30.0;
         
         sandiaDampingParameters.put(robotSide, sandiaDampingParametersForSide);
      }
      
      
   }

   public static double getAtlasDamping(int i)
   {
      return atlasDampingParameters[i];
   }
   
   public static double getAtlasDampingForPositionControl(int i)
   {
      return atlasPositionControlDampingParameters[i];
   }

   public static double getSandiaHandDamping(RobotSide robotSide, int i)
   {
      return sandiaDampingParameters.get(robotSide)[i];
   }

   public static double getSandiaHandDamping(String name)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         for(int i = 0;  i < ROSSandiaJointMap.handNames.get(robotSide).length; i++)
         {
            if(ROSSandiaJointMap.handNames.get(robotSide)[i].equals(name))
            {
               return sandiaDampingParameters.get(robotSide)[i];
            }
         }
      }
      throw new RuntimeException("Joint not found: " + name);
   }
}
