package us.ihmc.exampleSimulations.m2.Output;


import us.ihmc.exampleSimulations.m2.JointName;
import us.ihmc.yoVariables.variable.YoDouble;
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

   private YoDouble tau_left_hip_yaw;
   private YoDouble tau_left_hip_roll;
   private YoDouble tau_left_hip_pitch;
   private YoDouble tau_left_knee;
   private YoDouble tau_left_ankle_pitch;
   private YoDouble tau_left_ankle_roll;

   private YoDouble tau_right_hip_yaw;
   private YoDouble tau_right_hip_roll;
   private YoDouble tau_right_hip_pitch;
   private YoDouble tau_right_knee;
   private YoDouble tau_right_ankle_pitch;
   private YoDouble tau_right_ankle_roll;

   private YoDouble[][] limbJointPositions;

   public PerfectProcessedOutputs(Robot robot)
   {
      this.robot = robot;
      init();
   }

   public void init()
   {
      tau_left_hip_yaw = (YoDouble)robot.getVariable("tau_left_hip_yaw");
      tau_left_hip_roll = (YoDouble)robot.getVariable("tau_left_hip_roll");
      tau_left_hip_pitch =(YoDouble) robot.getVariable("tau_left_hip_pitch");
      tau_left_knee = (YoDouble)robot.getVariable("tau_left_knee");
      tau_left_ankle_pitch =(YoDouble) robot.getVariable("tau_left_ankle_pitch");
      tau_left_ankle_roll = (YoDouble)robot.getVariable("tau_left_ankle_roll");

      tau_right_hip_yaw = (YoDouble)robot.getVariable("tau_right_hip_yaw");
      tau_right_hip_roll = (YoDouble)robot.getVariable("tau_right_hip_roll");
      tau_right_hip_pitch = (YoDouble)robot.getVariable("tau_right_hip_pitch");
      tau_right_knee = (YoDouble)robot.getVariable("tau_right_knee");
      tau_right_ankle_pitch = (YoDouble)robot.getVariable("tau_right_ankle_pitch");
      tau_right_ankle_roll =(YoDouble)robot.getVariable("tau_right_ankle_roll");

      limbJointPositions = new YoDouble[][]
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
