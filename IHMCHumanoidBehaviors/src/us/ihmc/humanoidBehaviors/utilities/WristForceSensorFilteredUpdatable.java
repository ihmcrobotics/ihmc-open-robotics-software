package us.ihmc.humanoidBehaviors.utilities;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderBandPassFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable.FirstOrderFilterType;

public class WristForceSensorFilteredUpdatable implements Updatable
{
   private final double forceSensorMinPassThroughFreq_Hz = 0.1;
   private final double forceSensorMaxPassThroughFreq_Hz = 50.0;
   
   private final String forceSensorName; //AtlasSensorInformation.handForceSensorNames.get(RobotSide.RIGHT);
   private ForceSensorDataHolder forceSensorData;
   private Wrench wristWrench;
   private Vector3d wristForce;
   private final DoubleYoVariable wristForceMagnitude;
   private final FirstOrderFilteredYoVariable wristForceBias;
   private final FirstOrderBandPassFilteredYoVariable wristForceBandPassFiltered;

   private double maxObservedWristForce = 0.0;

   public WristForceSensorFilteredUpdatable(FullRobotModel fullRobotModel, RobotSide robotSide, ForceSensorDataHolder forceSensorDataHolder, double DT,  YoVariableRegistry registry)
   {
      if (robotSide.equals(RobotSide.LEFT))
      {
         forceSensorName = "l_arm_wrx";
      }
      else
      {
         forceSensorName = "r_arm_wrx";
      }
         
      forceSensorData = forceSensorDataHolder;
      wristWrench = new Wrench();
      wristForce = new Vector3d();
      wristForceMagnitude = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "WristForceMag", registry);
      wristForceBias = new FirstOrderFilteredYoVariable(robotSide.getShortLowerCaseName() + "WristForceBias", "", 0.0001, DT, FirstOrderFilterType.LOW_PASS, registry);
      wristForceBandPassFiltered = new FirstOrderBandPassFilteredYoVariable(robotSide.getShortLowerCaseName() + "WristForceMagFiltered", "", forceSensorMinPassThroughFreq_Hz, forceSensorMaxPassThroughFreq_Hz, DT, registry);
   }

   public DoubleYoVariable getWristForceMagnitude()
   {
      return wristForceMagnitude;
   }

   public DoubleYoVariable getWristForceBandPassFiltered()
   {
      return wristForceBandPassFiltered;
   }

   public void updateWristSensorValues()
   {
      ForceSensorData rightWristForceSensorData = forceSensorData.getByName(forceSensorName);
      rightWristForceSensorData.packWrench(wristWrench);
      
      wristWrench.packLinearPart(wristForce);

      double noise = 0.1 * 2.0 * ( Math.random() - 0.5 ) + 0.25;
      
      wristForceMagnitude.set(wristForce.length() + noise);
      wristForceBias.update(wristForceMagnitude.getDoubleValue());
      wristForceBandPassFiltered.update(wristForceMagnitude.getDoubleValue());

      if (wristForceBandPassFiltered.getDoubleValue() > maxObservedWristForce)
      {
         maxObservedWristForce = wristForceBandPassFiltered.getDoubleValue();
      }
   }
   

   
   @Override
   public void update(double time)
   {
      updateWristSensorValues();      
   }

}
