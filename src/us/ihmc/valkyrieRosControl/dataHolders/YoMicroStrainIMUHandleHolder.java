package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.sensors.IMUDefinition;

public class YoMicroStrainIMUHandleHolder extends YoIMUHandleHolder
{
   private final MicroStrainIMUHandle microStrainIMUHandle;

   public static YoMicroStrainIMUHandleHolder create(int sensorId, IMUDefinition imuDefinition, YoVariableRegistry parentRegistry)
   {
      return new YoMicroStrainIMUHandleHolder(new MicroStrainIMUHandle(imuDefinition.getName(), sensorId), imuDefinition, parentRegistry);
   }

   private YoMicroStrainIMUHandleHolder(MicroStrainIMUHandle handle, IMUDefinition imuDefinition, YoVariableRegistry parentRegistry)
   {
      super(handle, imuDefinition, parentRegistry);
      this.microStrainIMUHandle = handle;
   }

   @Override
   public void update()
   {
      microStrainIMUHandle.update();
      super.update();
   }

}
