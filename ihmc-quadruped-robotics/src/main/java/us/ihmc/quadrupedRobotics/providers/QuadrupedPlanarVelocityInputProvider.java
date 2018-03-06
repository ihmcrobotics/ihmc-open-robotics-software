package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.communication.packets.PlanarVelocityPacket;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedPlanarVelocityInputProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterVector3D planarVelocityLowerLimitsParameter;
   private final ParameterVector3D planarVelocityUpperLimitsParameter;

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

      Vector3D defaultPlanarVelocityLowerLimits = new Vector3D(- Double.MAX_VALUE, - Double.MAX_VALUE, - Double.MAX_VALUE);
      Vector3D defaultPlanarVelocityUpperLimits = new Vector3D(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

      planarVelocityLowerLimitsParameter = new ParameterVector3D("planarVelocityLowerLimit", defaultPlanarVelocityLowerLimits, registry);
      planarVelocityUpperLimitsParameter = new ParameterVector3D("planarVelocityUpperLimit", defaultPlanarVelocityUpperLimits, registry);

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(PlanarVelocityPacket.class, new PacketConsumer<PlanarVelocityPacket>()
         {
            @Override
            public void receivedPacket(PlanarVelocityPacket packet)
            {
               packet.get(planarVelocityInput);
               yoPlanarVelocityInputX.set(MathTools
                     .clamp(planarVelocityInput.getX(), planarVelocityLowerLimitsParameter.getX(), planarVelocityUpperLimitsParameter.getX()));
               yoPlanarVelocityInputY.set(MathTools
                     .clamp(planarVelocityInput.getY(), planarVelocityLowerLimitsParameter.getY(), planarVelocityUpperLimitsParameter.getY()));
               yoPlanarVelocityInputZ.set(MathTools
                     .clamp(planarVelocityInput.getZ(), planarVelocityLowerLimitsParameter.getZ(), planarVelocityUpperLimitsParameter.getZ()));
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
