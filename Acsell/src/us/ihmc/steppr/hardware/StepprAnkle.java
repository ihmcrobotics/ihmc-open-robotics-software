package us.ihmc.steppr.hardware;

import us.ihmc.acsell.hardware.AcsellAnkle;
import us.ihmc.acsell.hardware.configuration.StrainGaugeInformation;

public enum StepprAnkle implements AcsellAnkle
{
   LEFT(new StrainGaugeInformation(StepprActuator.LEFT_ANKLE_LEFT, 1, -660.0, 3.9)),
   RIGHT(new StrainGaugeInformation(StepprActuator.RIGHT_ANKLE_LEFT, 1, -660.0, 3.9));
   
   private final StrainGaugeInformation shankSensor;
   
   StepprAnkle(StrainGaugeInformation shankSensor)
   {
      this.shankSensor = shankSensor;
   }
   
   public StrainGaugeInformation getShankSensor()
   {
      return shankSensor;
   }
}
