package us.ihmc.quadrupedRobotics;

import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class QuadrupedTestYoVariables
{
   private final DoubleYoVariable yoTime;
   
   private final DoubleYoVariable yoComPositionInputX;
   private final DoubleYoVariable yoComPositionInputY;
   private final DoubleYoVariable yoComPositionInputZ;
   private final DoubleYoVariable yoComVelocityInputX;
   private final DoubleYoVariable yoComVelocityInputY;
   private final DoubleYoVariable yoComVelocityInputZ;
   private final DoubleYoVariable yoBodyOrientationInputYaw;
   private final DoubleYoVariable yoBodyOrientationInputPitch;
   private final DoubleYoVariable yoBodyOrientationInputRoll;
   private final DoubleYoVariable yoBodyAngularRateInputX;
   private final DoubleYoVariable yoBodyAngularRateInputY;
   private final DoubleYoVariable yoBodyAngularRateInputZ;
   private final DoubleYoVariable yoPlanarVelocityInputX;
   private final DoubleYoVariable yoPlanarVelocityInputY;
   private final DoubleYoVariable yoPlanarVelocityInputZ;
   
   private final DoubleYoVariable robotBodyX;
   private final DoubleYoVariable robotBodyY;
   private final DoubleYoVariable robotBodyZ;
   private final DoubleYoVariable robotBodyYaw;

   private final QuadrantDependentList<YoBoolean> controllerFootSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoBoolean> footSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> solePositionXs = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> solePositionYs = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> solePositionZs = new QuadrantDependentList<>();
   
   private final YoBoolean limitJointTorques;
   
   public QuadrupedTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (DoubleYoVariable) scs.getVariable("t");
      
      yoComPositionInputX = (DoubleYoVariable) scs.getVariable("comPositionInputX");
      yoComPositionInputY = (DoubleYoVariable) scs.getVariable("comPositionInputY");
      yoComPositionInputZ = (DoubleYoVariable) scs.getVariable("comPositionInputZ");
      yoComVelocityInputX = (DoubleYoVariable) scs.getVariable("comVelocityInputX");
      yoComVelocityInputY = (DoubleYoVariable) scs.getVariable("comVelocityInputY");
      yoComVelocityInputZ = (DoubleYoVariable) scs.getVariable("comVelocityInputZ");
      yoBodyOrientationInputYaw = (DoubleYoVariable) scs.getVariable("bodyOrientationInputYaw");
      yoBodyOrientationInputPitch = (DoubleYoVariable) scs.getVariable("bodyOrientationInputPitch");
      yoBodyOrientationInputRoll = (DoubleYoVariable) scs.getVariable("bodyOrientationInputRoll");
      yoBodyAngularRateInputX = (DoubleYoVariable) scs.getVariable("bodyAngularRateInputX");
      yoBodyAngularRateInputY = (DoubleYoVariable) scs.getVariable("bodyAngularRateInputY");
      yoBodyAngularRateInputZ = (DoubleYoVariable) scs.getVariable("bodyAngularRateInputZ");
      yoPlanarVelocityInputX = (DoubleYoVariable) scs.getVariable("planarVelocityInputX");
      yoPlanarVelocityInputY = (DoubleYoVariable) scs.getVariable("planarVelocityInputY");
      yoPlanarVelocityInputZ = (DoubleYoVariable) scs.getVariable("planarVelocityInputZ");
      
      robotBodyX = (DoubleYoVariable) scs.getVariable("q_x");
      robotBodyY = (DoubleYoVariable) scs.getVariable("q_y");
      robotBodyZ = (DoubleYoVariable) scs.getVariable("q_z");
      robotBodyYaw = (DoubleYoVariable) scs.getVariable("q_yaw");
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerFootSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch_controllerThinksHasTouchedDown"));
         footSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetected"));
         solePositionXs.set(robotQuadrant, (DoubleYoVariable) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionX"));
         solePositionYs.set(robotQuadrant, (DoubleYoVariable) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionY"));
         solePositionZs.set(robotQuadrant, (DoubleYoVariable) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionZ"));
      }
      
      limitJointTorques = (YoBoolean) scs.getVariable("limitJointTorques");
   }

   public DoubleYoVariable getYoComPositionInputX()
   {
      return yoComPositionInputX;
   }

   public DoubleYoVariable getYoComPositionInputY()
   {
      return yoComPositionInputY;
   }

   public DoubleYoVariable getYoComPositionInputZ()
   {
      return yoComPositionInputZ;
   }

   public DoubleYoVariable getYoComVelocityInputX()
   {
      return yoComVelocityInputX;
   }

   public DoubleYoVariable getYoComVelocityInputY()
   {
      return yoComVelocityInputY;
   }

   public DoubleYoVariable getYoComVelocityInputZ()
   {
      return yoComVelocityInputZ;
   }

   public DoubleYoVariable getYoBodyOrientationInputYaw()
   {
      return yoBodyOrientationInputYaw;
   }

   public DoubleYoVariable getYoBodyOrientationInputPitch()
   {
      return yoBodyOrientationInputPitch;
   }

   public DoubleYoVariable getYoBodyOrientationInputRoll()
   {
      return yoBodyOrientationInputRoll;
   }

   public DoubleYoVariable getYoBodyAngularRateInputX()
   {
      return yoBodyAngularRateInputX;
   }

   public DoubleYoVariable getYoBodyAngularRateInputY()
   {
      return yoBodyAngularRateInputY;
   }

   public DoubleYoVariable getYoBodyAngularRateInputZ()
   {
      return yoBodyAngularRateInputZ;
   }

   public DoubleYoVariable getYoPlanarVelocityInputX()
   {
      return yoPlanarVelocityInputX;
   }

   public DoubleYoVariable getYoPlanarVelocityInputY()
   {
      return yoPlanarVelocityInputY;
   }

   public DoubleYoVariable getYoPlanarVelocityInputZ()
   {
      return yoPlanarVelocityInputZ;
   }

   public DoubleYoVariable getRobotBodyX()
   {
      return robotBodyX;
   }

   public DoubleYoVariable getRobotBodyY()
   {
      return robotBodyY;
   }

   public DoubleYoVariable getRobotBodyZ()
   {
      return robotBodyZ;
   }

   public DoubleYoVariable getRobotBodyYaw()
   {
      return robotBodyYaw;
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTime;
   }

   public QuadrantDependentList<YoBoolean> getControllerFootSwitches()
   {
      return controllerFootSwitches;
   }

   public QuadrantDependentList<YoBoolean> getFootSwitches()
   {
      return footSwitches;
   }

   public QuadrantDependentList<DoubleYoVariable> getSolePositionXs()
   {
      return solePositionXs;
   }

   public QuadrantDependentList<DoubleYoVariable> getSolePositionYs()
   {
      return solePositionYs;
   }

   public QuadrantDependentList<DoubleYoVariable> getSolePositionZs()
   {
      return solePositionZs;
   }

   public YoBoolean getLimitJointTorques()
   {
      return limitJointTorques;
   }
}
