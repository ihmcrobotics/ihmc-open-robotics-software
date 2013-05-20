package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap;
import us.ihmc.darpaRoboticsChallenge.ros.ROSSandiaJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class DRCRobotDampingParameters
{
   private final static double[] atlasDampingParameters = new double[ROSAtlasJointMap.numberOfJoints];

   private final static SideDependentList<double[]> sandiaDampingParameters = new SideDependentList<double[]>();
   
   static
   {
      atlasDampingParameters[ROSAtlasJointMap.back_lbz]  = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.back_mby]  = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.back_ubx]  = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.neck_ay]   = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_uhz] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_mhx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_lhy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_kny] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_uay] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_leg_lax] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_uhz] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_mhx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_lhy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_kny] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_uay] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_leg_lax] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_usy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_shx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_ely] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_elx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_uwy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.l_arm_mwx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_usy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_shx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_ely] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_elx] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_uwy] = 0.1;
      atlasDampingParameters[ROSAtlasJointMap.r_arm_mwx] = 0.1;
      
      for(RobotSide robotSide : RobotSide.values)
      {
         double[] sandiaDampingParametersForSide = new double[ROSSandiaJointMap.numberOfJointsPerHand];
         
         sandiaDampingParametersForSide[ROSSandiaJointMap.f0_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f0_j1] = 1.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f0_j2] = 1.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f1_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f1_j1] = 1.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f1_j2] = 1.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f2_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f2_j1] = 1.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f2_j2] = 1.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f3_j0] = 30.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f3_j1] = 1.0;
         sandiaDampingParametersForSide[ROSSandiaJointMap.f3_j2] = 30.0;
         
         sandiaDampingParameters.put(robotSide, sandiaDampingParametersForSide);
      }
      
      
   }

   public static double getAtlasDamping(int i)
   {
      return atlasDampingParameters[i];
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
