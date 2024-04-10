package us.ihmc.commonWalkingControlModules.donkeyKick;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TriggerKickCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class KickInputParameters
{
   private YoEnum<RobotSide> kickFootSide; // side for the foot to kick
   private YoDouble kickHeight; // height of where you want the kick
   private YoDouble kickImpulse; // Newton-seconds
   private YoDouble kickTargetDistance;
   private YoDouble prekickWeightDistribution;

   public KickInputParameters(YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      kickFootSide = new YoEnum<>("inputKickFootSide", registry, RobotSide.class);
      kickHeight = new YoDouble("inputKickHeight", registry);
      kickImpulse = new YoDouble("inputKickImpulse", registry);
      kickTargetDistance = new YoDouble("inputKickTargetDistance", registry);
      prekickWeightDistribution = new YoDouble("inputPrekickWeightDistribution", registry);

      // These default parameters are generally overwritten with set(TriggerKickCommand kickMessage)
      kickFootSide.set(RobotSide.LEFT);
      kickHeight.set(0.55);
      kickImpulse.set(55.0);
      kickTargetDistance.set(0.75);
      prekickWeightDistribution.set(0.5);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public double getKickHeight()
   {
      return kickHeight.getDoubleValue();
   }

   public double getKickImpulse()
   {
      return kickImpulse.getDoubleValue();
   }

   public double getKickTargetDistance()
   {
      return kickTargetDistance.getDoubleValue();
   }

   public RobotSide getKickFootSide()
   {
      return kickFootSide.getEnumValue();
   }

   public double getPrekickWeightDistribution()
   {
      return prekickWeightDistribution.getDoubleValue();
   }

   public void setKickHeight(double kickHeight)
   {
      this.kickHeight.set(kickHeight);
   }

   public void setKickImpulse(double kickImpulse)
   {
      this.kickImpulse.set(kickImpulse);
   }

   public void setKickTargetDistance(double kickTargetDistance)
   {
      this.kickTargetDistance.set(kickTargetDistance);
   }

   public void setKickFootSide(RobotSide kickFootSide)
   {
      this.kickFootSide.set(kickFootSide);
   }

   public void setPrekickWeightDistribution(double prekickWeightDistribution)
   {
      this.prekickWeightDistribution.set(prekickWeightDistribution);
   }

   public void set(TriggerKickCommand kickMessage)
   {
      setKickFootSide(kickMessage.getRobotSide());
      setKickHeight(kickMessage.getKickHeight());
      setKickImpulse(kickMessage.getKickImpulse());
      setKickTargetDistance(kickMessage.getKickTargetDistance());
      setPrekickWeightDistribution(kickMessage.getPrekickWeightDistribution());
   }
}
