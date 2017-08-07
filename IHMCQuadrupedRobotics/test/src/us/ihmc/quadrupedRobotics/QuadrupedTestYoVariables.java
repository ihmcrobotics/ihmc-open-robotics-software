package us.ihmc.quadrupedRobotics;

import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class QuadrupedTestYoVariables
{
   private final YoDouble yoTime;
   
   private final YoDouble yoComPositionInputX;
   private final YoDouble yoComPositionInputY;
   private final YoDouble yoComPositionInputZ;
   private final YoDouble yoComVelocityInputX;
   private final YoDouble yoComVelocityInputY;
   private final YoDouble yoComVelocityInputZ;
   private final YoDouble yoBodyOrientationInputYaw;
   private final YoDouble yoBodyOrientationInputPitch;
   private final YoDouble yoBodyOrientationInputRoll;
   private final YoDouble yoBodyAngularRateInputX;
   private final YoDouble yoBodyAngularRateInputY;
   private final YoDouble yoBodyAngularRateInputZ;
   private final YoDouble yoPlanarVelocityInputX;
   private final YoDouble yoPlanarVelocityInputY;
   private final YoDouble yoPlanarVelocityInputZ;
   
   private final YoDouble robotBodyX;
   private final YoDouble robotBodyY;
   private final YoDouble robotBodyZ;
   private final YoDouble robotBodyYaw;

   private final QuadrantDependentList<YoBoolean> controllerFootSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoBoolean> footSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionXs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionYs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionZs = new QuadrantDependentList<>();
   
   private final YoBoolean limitJointTorques;
   
   public QuadrupedTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (YoDouble) scs.getVariable("t");
      
      yoComPositionInputX = (YoDouble) scs.getVariable("comPositionInputX");
      yoComPositionInputY = (YoDouble) scs.getVariable("comPositionInputY");
      yoComPositionInputZ = (YoDouble) scs.getVariable("comPositionInputZ");
      yoComVelocityInputX = (YoDouble) scs.getVariable("comVelocityInputX");
      yoComVelocityInputY = (YoDouble) scs.getVariable("comVelocityInputY");
      yoComVelocityInputZ = (YoDouble) scs.getVariable("comVelocityInputZ");
      yoBodyOrientationInputYaw = (YoDouble) scs.getVariable("bodyOrientationInputYaw");
      yoBodyOrientationInputPitch = (YoDouble) scs.getVariable("bodyOrientationInputPitch");
      yoBodyOrientationInputRoll = (YoDouble) scs.getVariable("bodyOrientationInputRoll");
      yoBodyAngularRateInputX = (YoDouble) scs.getVariable("bodyAngularRateInputX");
      yoBodyAngularRateInputY = (YoDouble) scs.getVariable("bodyAngularRateInputY");
      yoBodyAngularRateInputZ = (YoDouble) scs.getVariable("bodyAngularRateInputZ");
      yoPlanarVelocityInputX = (YoDouble) scs.getVariable("planarVelocityInputX");
      yoPlanarVelocityInputY = (YoDouble) scs.getVariable("planarVelocityInputY");
      yoPlanarVelocityInputZ = (YoDouble) scs.getVariable("planarVelocityInputZ");
      
      robotBodyX = (YoDouble) scs.getVariable("q_x");
      robotBodyY = (YoDouble) scs.getVariable("q_y");
      robotBodyZ = (YoDouble) scs.getVariable("q_z");
      robotBodyYaw = (YoDouble) scs.getVariable("q_yaw");
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerFootSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch_controllerThinksHasTouchedDown"));
         footSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetected"));
         solePositionXs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionX"));
         solePositionYs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionY"));
         solePositionZs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionZ"));
      }
      
      limitJointTorques = (YoBoolean) scs.getVariable("limitJointTorques");
   }

   public YoDouble getYoComPositionInputX()
   {
      return yoComPositionInputX;
   }

   public YoDouble getYoComPositionInputY()
   {
      return yoComPositionInputY;
   }

   public YoDouble getYoComPositionInputZ()
   {
      return yoComPositionInputZ;
   }

   public YoDouble getYoComVelocityInputX()
   {
      return yoComVelocityInputX;
   }

   public YoDouble getYoComVelocityInputY()
   {
      return yoComVelocityInputY;
   }

   public YoDouble getYoComVelocityInputZ()
   {
      return yoComVelocityInputZ;
   }

   public YoDouble getYoBodyOrientationInputYaw()
   {
      return yoBodyOrientationInputYaw;
   }

   public YoDouble getYoBodyOrientationInputPitch()
   {
      return yoBodyOrientationInputPitch;
   }

   public YoDouble getYoBodyOrientationInputRoll()
   {
      return yoBodyOrientationInputRoll;
   }

   public YoDouble getYoBodyAngularRateInputX()
   {
      return yoBodyAngularRateInputX;
   }

   public YoDouble getYoBodyAngularRateInputY()
   {
      return yoBodyAngularRateInputY;
   }

   public YoDouble getYoBodyAngularRateInputZ()
   {
      return yoBodyAngularRateInputZ;
   }

   public YoDouble getYoPlanarVelocityInputX()
   {
      return yoPlanarVelocityInputX;
   }

   public YoDouble getYoPlanarVelocityInputY()
   {
      return yoPlanarVelocityInputY;
   }

   public YoDouble getYoPlanarVelocityInputZ()
   {
      return yoPlanarVelocityInputZ;
   }

   public YoDouble getRobotBodyX()
   {
      return robotBodyX;
   }

   public YoDouble getRobotBodyY()
   {
      return robotBodyY;
   }

   public YoDouble getRobotBodyZ()
   {
      return robotBodyZ;
   }

   public YoDouble getRobotBodyYaw()
   {
      return robotBodyYaw;
   }

   public YoDouble getYoTime()
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

   public QuadrantDependentList<YoDouble> getSolePositionXs()
   {
      return solePositionXs;
   }

   public QuadrantDependentList<YoDouble> getSolePositionYs()
   {
      return solePositionYs;
   }

   public QuadrantDependentList<YoDouble> getSolePositionZs()
   {
      return solePositionZs;
   }

   public YoBoolean getLimitJointTorques()
   {
      return limitJointTorques;
   }
}
