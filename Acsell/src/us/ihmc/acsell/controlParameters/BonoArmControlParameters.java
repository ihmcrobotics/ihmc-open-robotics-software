package us.ihmc.acsell.controlParameters;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.controller.YoPIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSE3PIDGains;

/**
 * Created by dstephen on 2/14/14.
 */
public class BonoArmControlParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   
   public BonoArmControlParameters()
   {
      this(false);
   }
   
   public BonoArmControlParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public double[] getLowLevelArmJointspaceKp()
   {
      return new double[0]; 
   }
   @Override
   public double[] getLowLevelArmJointspaceKi()
   {
      return new double[0]; 
   }
   @Override
   public double[] getLowLevelArmJointspaceKd()
   {
      return new double[0]; 
   }
   @Override
   public double[] getLowLevelArmJointspaceFfqd_d()
   {
      return new double[0]; 
   }
   @Override
   public double[] getLowLevelArmJointspaceQerrMax()
   {
      return new double[0];  
   }

   @Override
   public YoPIDGains createJointspaceControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public YoSE3PIDGains createTaskspaceControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public boolean useInverseKinematicsTaskspaceControl()
   {
      return false;
   }

   @Override
   public boolean doLowLevelPositionControl()
   {
      return false;
   }
   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<OneDoFJoint, Double>();
   }
}
