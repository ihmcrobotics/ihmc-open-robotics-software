package us.ihmc.exampleSimulations.m2.Output;


import us.ihmc.exampleSimulations.m2.JointName;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;

/**
 * <p>
 * Title: PerfectProcessedOutputs
 * </p>
 *
 * <p>
 * Description: Perfect, noiseless outputs. sets the tau for each degree of
 * freedom.
 * </p>
 *
 * <p>
 * Company: IHMC
 * </p>
 *
 * @version 2.0
 */

public class PerfectProcessedOutputs
{
   private final Robot robot;

   private DoubleYoVariable tau_left_hip_yaw;
   private DoubleYoVariable tau_left_hip_roll;
   private DoubleYoVariable tau_left_hip_pitch;
   private DoubleYoVariable tau_left_knee;
   private DoubleYoVariable tau_left_ankle_pitch;
   private DoubleYoVariable tau_left_ankle_roll;

   private DoubleYoVariable tau_right_hip_yaw;
   private DoubleYoVariable tau_right_hip_roll;
   private DoubleYoVariable tau_right_hip_pitch;
   private DoubleYoVariable tau_right_knee;
   private DoubleYoVariable tau_right_ankle_pitch;
   private DoubleYoVariable tau_right_ankle_roll;

   private DoubleYoVariable[][] limbJointPositions;

   public PerfectProcessedOutputs(Robot robot)
   {
      this.robot = robot;
      init();
   }

   public void init()
   {
      tau_left_hip_yaw = (DoubleYoVariable)robot.getVariable("tau_left_hip_yaw");
      tau_left_hip_roll = (DoubleYoVariable)robot.getVariable("tau_left_hip_roll");
      tau_left_hip_pitch =(DoubleYoVariable) robot.getVariable("tau_left_hip_pitch");
      tau_left_knee = (DoubleYoVariable)robot.getVariable("tau_left_knee");
      tau_left_ankle_pitch =(DoubleYoVariable) robot.getVariable("tau_left_ankle_pitch");
      tau_left_ankle_roll = (DoubleYoVariable)robot.getVariable("tau_left_ankle_roll");

      tau_right_hip_yaw = (DoubleYoVariable)robot.getVariable("tau_right_hip_yaw");
      tau_right_hip_roll = (DoubleYoVariable)robot.getVariable("tau_right_hip_roll");
      tau_right_hip_pitch = (DoubleYoVariable)robot.getVariable("tau_right_hip_pitch");
      tau_right_knee = (DoubleYoVariable)robot.getVariable("tau_right_knee");
      tau_right_ankle_pitch = (DoubleYoVariable)robot.getVariable("tau_right_ankle_pitch");
      tau_right_ankle_roll =(DoubleYoVariable)robot.getVariable("tau_right_ankle_roll");

      limbJointPositions = new DoubleYoVariable[][]
      {
         {
            tau_left_hip_yaw, tau_left_hip_roll, tau_left_hip_pitch, tau_left_knee, tau_left_ankle_pitch, tau_left_ankle_roll
         },
         {
            tau_right_hip_yaw, tau_right_hip_roll, tau_right_hip_pitch, tau_right_knee, tau_right_ankle_pitch, tau_right_ankle_roll
         }
      };

   }

   public void setJointTorque(RobotSide robotSide, JointName jointName, double val)
   {
      limbJointPositions[robotSide.ordinal()][jointName.ordinal()].set(val);
   }

}
