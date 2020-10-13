package us.ihmc.atlas.parameters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.ui.UIAuxiliaryRobotData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.function.ToDoubleFunction;

public class AtlasUIAuxiliaryData implements UIAuxiliaryRobotData
{
   private static final Vector3D rootJointToMidFootOffset = new Vector3D(0.0518637, 0.0, -0.8959426);
   private static final HashMap<String, Double> defaultJointAngleMap = new HashMap<>();
   private static final SideDependentList<double[]> armsInJointAngleMap = new SideDependentList<>();

   static
   {
      defaultJointAngleMap.put("back_bky", -0.05);
      defaultJointAngleMap.put("l_leg_aky", 0.5 * -0.75);
      defaultJointAngleMap.put("r_leg_aky", 0.5 * -0.75);
      defaultJointAngleMap.put("l_leg_kny", 0.5 * 1.65);
      defaultJointAngleMap.put("r_leg_kny", 0.5 * 1.65);
      defaultJointAngleMap.put("back_bkz", 0.0);
      defaultJointAngleMap.put("back_bkx", 0.0);

      defaultJointAngleMap.put("l_leg_hpz", 0.0);
      defaultJointAngleMap.put("l_leg_hpx", 0.09);
      defaultJointAngleMap.put("l_leg_hpy", 0.5 * -0.933);
      defaultJointAngleMap.put("l_leg_akx", -0.09);
      defaultJointAngleMap.put("r_leg_hpz", 0.0);
      defaultJointAngleMap.put("r_leg_hpx", -0.09);
      defaultJointAngleMap.put("r_leg_hpy", 0.5 * -0.933);
      defaultJointAngleMap.put("r_leg_akx", 0.09);
      defaultJointAngleMap.put("l_arm_shz", 0.0);
      defaultJointAngleMap.put("l_arm_shx", -1.3);
      defaultJointAngleMap.put("l_arm_ely", 2.0);
      defaultJointAngleMap.put("l_arm_elx", 0.5);
      defaultJointAngleMap.put("l_arm_wry", 0.01);
      defaultJointAngleMap.put("l_arm_wrx", 0.0);
      defaultJointAngleMap.put("l_arm_wry2", 0.0);
      defaultJointAngleMap.put("r_arm_shz", 0.0);
      defaultJointAngleMap.put("r_arm_shx", 1.3);
      defaultJointAngleMap.put("r_arm_ely", 2.0);
      defaultJointAngleMap.put("r_arm_elx", -0.5);
      defaultJointAngleMap.put("r_arm_wry", 0.01);
      defaultJointAngleMap.put("r_arm_wrx", 0.0);
      defaultJointAngleMap.put("r_arm_wry2", 0.0);

      double[] leftArmsInJointAngles = new double[7];
      double[] rightArmsInJointAngles = new double[7];

      leftArmsInJointAngles[0] = -1.5708;
      leftArmsInJointAngles[1] = -0.8423;
      leftArmsInJointAngles[2] = 0.3365;
      leftArmsInJointAngles[3] = 1.95176;
      leftArmsInJointAngles[4] = -0.3404;
      leftArmsInJointAngles[5] = 0.4884;
      leftArmsInJointAngles[6] = 0.5375;

      rightArmsInJointAngles[0] = -leftArmsInJointAngles[0];
      rightArmsInJointAngles[1] = -leftArmsInJointAngles[1];
      rightArmsInJointAngles[2] =  leftArmsInJointAngles[2];
      rightArmsInJointAngles[3] = -leftArmsInJointAngles[3];
      rightArmsInJointAngles[4] =  leftArmsInJointAngles[4];
      rightArmsInJointAngles[5] = -leftArmsInJointAngles[5];
      rightArmsInJointAngles[6] =  leftArmsInJointAngles[6];

      armsInJointAngleMap.put(RobotSide.LEFT, leftArmsInJointAngles);
      armsInJointAngleMap.put(RobotSide.RIGHT, rightArmsInJointAngles);
   }

   @Override
   public Vector3D getRootJointToMidFootOffset()
   {
      return rootJointToMidFootOffset;
   }

   @Override
   public ToDoubleFunction<String> getDefaultJointAngleMap()
   {
      return jointName -> defaultJointAngleMap.getOrDefault(jointName, 0.0);
   }

   @Override
   public SideDependentList<double[]> getArmsInJointAngles()
   {
      return armsInJointAngleMap;
   }
}
