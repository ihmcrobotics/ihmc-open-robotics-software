package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainData;

public class YoMicroStrainIMUHandleHolder extends YoIMUHandleHolder
{
   private final MicroStrainIMUHandle microStrainIMUHandle;

   private final YoEnum<MicroStrainData.MicrostrainFilterType> filterTypeToUse;

   public static YoMicroStrainIMUHandleHolder create(int sensorId, IMUDefinition imuDefinition, YoRegistry parentRegistry)
   {
      return new YoMicroStrainIMUHandleHolder(new MicroStrainIMUHandle(imuDefinition.getName(), sensorId), imuDefinition, parentRegistry);
   }

   private YoMicroStrainIMUHandleHolder(MicroStrainIMUHandle handle, IMUDefinition imuDefinition, YoRegistry parentRegistry)
   {
      super(handle, imuDefinition, parentRegistry);
      this.microStrainIMUHandle = handle;

      filterTypeToUse = new YoEnum<>(handle.getName() + "_filterTypeToUse", parentRegistry, MicroStrainData.MicrostrainFilterType.class);

      filterTypeToUse.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            microStrainIMUHandle.setFilterTypeToReturn(filterTypeToUse.getEnumValue());
         }
      });
      filterTypeToUse.set(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);

   }

   @Override
   public void update()
   {
      microStrainIMUHandle.update();
      super.update();
   }

}
