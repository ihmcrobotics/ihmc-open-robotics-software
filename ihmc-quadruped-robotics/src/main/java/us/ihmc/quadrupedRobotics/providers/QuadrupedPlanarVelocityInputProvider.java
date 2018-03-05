package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.communication.packets.PlanarVelocityPacket;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedPlanarVelocityInputProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter[] planarVelocityLowerLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] planarVelocityUpperLimitsParameter = new DoubleParameter[3];

   private final YoDouble yoPlanarVelocityInputX;
   private final YoDouble yoPlanarVelocityInputY;
   private final YoDouble yoPlanarVelocityInputZ;
   private final Vector3D planarVelocityInput;

   public QuadrupedPlanarVelocityInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      yoPlanarVelocityInputX = new YoDouble("planarVelocityInputX", registry);
      yoPlanarVelocityInputY = new YoDouble("planarVelocityInputY", registry);
      yoPlanarVelocityInputZ = new YoDouble("planarVelocityInputZ", registry);
      yoPlanarVelocityInputX.set(0);
      yoPlanarVelocityInputY.set(0);
      yoPlanarVelocityInputZ.set(0);
      planarVelocityInput = new Vector3D();

      for (int i = 0; i < 3; i++)
      {
         planarVelocityLowerLimitsParameter[i] = new DoubleParameter("planarVelocityLowerLimit" + Axis.values[i], registry, - Double.MAX_VALUE, - Double.MAX_VALUE, Double.MAX_VALUE);
         planarVelocityUpperLimitsParameter[i] = new DoubleParameter("planarVelocityUpperLimit" + Axis.values[i], registry, Double.MAX_VALUE, - Double.MAX_VALUE, Double.MAX_VALUE);
      }

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(PlanarVelocityPacket.class, new PacketConsumer<PlanarVelocityPacket>()
         {
            @Override
            public void receivedPacket(PlanarVelocityPacket packet)
            {
               packet.get(planarVelocityInput);
               yoPlanarVelocityInputX.set(MathTools
                     .clamp(planarVelocityInput.getX(), planarVelocityLowerLimitsParameter[0].getValue(), planarVelocityUpperLimitsParameter[0].getValue()));
               yoPlanarVelocityInputY.set(MathTools
                     .clamp(planarVelocityInput.getY(), planarVelocityLowerLimitsParameter[1].getValue(), planarVelocityUpperLimitsParameter[1].getValue()));
               yoPlanarVelocityInputZ.set(MathTools
                     .clamp(planarVelocityInput.getZ(), planarVelocityLowerLimitsParameter[2].getValue(), planarVelocityUpperLimitsParameter[2].getValue()));
            }
         });
      }

      parentRegistry.addChild(registry);
   }

   public Vector3D get()
   {
      planarVelocityInput.set(yoPlanarVelocityInputX.getDoubleValue(), yoPlanarVelocityInputY.getDoubleValue(), yoPlanarVelocityInputZ.getDoubleValue());
      return planarVelocityInput;
   }
}
