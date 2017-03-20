package us.ihmc.atlas;

import us.ihmc.atlas.ros.AtlasOrderedJointMap;
import us.ihmc.atlas.ros.ROSSandiaJointMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasDampingParameters
{
   private final static double[] atlasDampingParameters = new double[AtlasOrderedJointMap.numberOfJoints];
   private final static double[] atlasPositionControlDampingParameters = new double[AtlasOrderedJointMap.numberOfJoints];

   private final static SideDependentList<double[]> sandiaDampingParameters = new SideDependentList<double[]>();
   private static final boolean USE_REALLY_HIGH_FINGER_DAMPING=true;
   static
   {
      atlasDampingParameters[AtlasOrderedJointMap.back_bkz]  = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.back_bky]  = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.back_bkx]  = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.neck_ry]   = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_leg_hpz] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_leg_hpx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_leg_hpy] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_leg_kny] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_leg_aky] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_leg_akx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_leg_hpz] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_leg_hpx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_leg_hpy] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_leg_kny] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_leg_aky] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_leg_akx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_arm_shz] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_arm_shx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_arm_ely] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_arm_elx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_arm_wry] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.l_arm_wrx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_arm_shz] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_arm_shx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_arm_ely] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_arm_elx] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_arm_wry] = 0.1;
      atlasDampingParameters[AtlasOrderedJointMap.r_arm_wrx] = 0.1;
      
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.back_bkz]  = 10.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.back_bky]  = 10.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.back_bkx]  = 10.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.neck_ry]   = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_leg_hpz] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_leg_hpx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_leg_hpy] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_leg_kny] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_leg_aky] = 10.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_leg_akx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_leg_hpz] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_leg_hpx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_leg_hpy] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_leg_kny] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_leg_aky] = 10.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_leg_akx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_arm_shz] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_arm_shx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_arm_ely] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_arm_elx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_arm_wry] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.l_arm_wrx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_arm_shz] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_arm_shx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_arm_ely] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_arm_elx] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_arm_wry] = 1.0;
      atlasPositionControlDampingParameters[AtlasOrderedJointMap.r_arm_wrx] = 1.0;
      
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
   
   public static void setDampingParameters(FloatingRootJointRobot simulatedRobot, DRCHandType handModel, DRCRobotJointMap jointMap)
   {
      String[] orderedJointNames = jointMap.getOrderedJointNames();
      for (int i = 0; i < orderedJointNames.length; i++)
      {
         simulatedRobot.getOneDegreeOfFreedomJoint(orderedJointNames[i]).setDamping(getAtlasDamping(i));
      }
      
   }
}
