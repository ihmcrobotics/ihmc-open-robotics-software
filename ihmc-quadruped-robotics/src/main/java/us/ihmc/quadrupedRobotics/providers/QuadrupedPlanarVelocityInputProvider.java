package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.communication.packets.PlanarVelocityPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedPlanarVelocityInputProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter planarVelocityLowerLimitsParameter = parameterFactory.createDoubleArray("planarVelocityLowerLimits", -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
   private final DoubleArrayParameter planarVelocityUpperLimitsParameter = parameterFactory.createDoubleArray("planarVelocityUpperLimits", Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

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

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(PlanarVelocityPacket.class, new PacketConsumer<PlanarVelocityPacket>()
         {
            @Override
            public void receivedPacket(PlanarVelocityPacket packet)
            {
               packet.get(planarVelocityInput);
               yoPlanarVelocityInputX.set(MathTools
                     .clamp(planarVelocityInput.getX(), planarVelocityLowerLimitsParameter.get(0), planarVelocityUpperLimitsParameter.get(0)));
               yoPlanarVelocityInputY.set(MathTools
                     .clamp(planarVelocityInput.getY(), planarVelocityLowerLimitsParameter.get(1), planarVelocityUpperLimitsParameter.get(1)));
               yoPlanarVelocityInputZ.set(MathTools
                     .clamp(planarVelocityInput.getZ(), planarVelocityLowerLimitsParameter.get(2), planarVelocityUpperLimitsParameter.get(2)));
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
