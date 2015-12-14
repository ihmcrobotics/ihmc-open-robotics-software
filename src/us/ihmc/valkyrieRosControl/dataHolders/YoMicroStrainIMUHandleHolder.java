package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.valkyrie.imu.MicroStrainData.MicrostrainPacketType;

public class YoMicroStrainIMUHandleHolder extends YoIMUHandleHolder
{
   private final MicroStrainIMUHandle microStrainIMUHandle;

   private final EnumYoVariable<MicrostrainPacketType> packetTypeToUse;

   public static YoMicroStrainIMUHandleHolder create(int sensorId, IMUDefinition imuDefinition, YoVariableRegistry parentRegistry)
   {
      return new YoMicroStrainIMUHandleHolder(new MicroStrainIMUHandle(imuDefinition.getName(), sensorId), imuDefinition, parentRegistry);
   }

   private YoMicroStrainIMUHandleHolder(MicroStrainIMUHandle handle, IMUDefinition imuDefinition, YoVariableRegistry parentRegistry)
   {
      super(handle, imuDefinition, parentRegistry);
      this.microStrainIMUHandle = handle;

      packetTypeToUse = new EnumYoVariable<>(handle.getName() + "_packetTypeToUse", parentRegistry, MicrostrainPacketType.class);

      packetTypeToUse.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            microStrainIMUHandle.setPacketTypeToReturn(packetTypeToUse.getEnumValue());
         }
      });
      packetTypeToUse.set(MicrostrainPacketType.COMPLIMENTARY_FILTER);

   }

   @Override
   public void update()
   {
      microStrainIMUHandle.update();
      super.update();
   }

}
