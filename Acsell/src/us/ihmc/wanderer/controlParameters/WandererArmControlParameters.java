package us.ihmc.wanderer.controlParameters;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;


/**
 * Created by dstephen on 2/14/14.
 */
public class WandererArmControlParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   
   public WandererArmControlParameters()
   {
      this(false);
   }
   
   public WandererArmControlParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
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
   public YoSE3PIDGains createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry)
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

   @Override
   public double getWristHandCenterOffset()
   {
      // TODO Auto-generated method stub
      return 0.0;
   }
}
