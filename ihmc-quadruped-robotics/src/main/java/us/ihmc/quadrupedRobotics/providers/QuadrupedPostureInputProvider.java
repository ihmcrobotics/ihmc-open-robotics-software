package us.ihmc.quadrupedRobotics.providers;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ComPositionPacket;
import controller_msgs.msg.dds.ComVelocityPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedPostureInputProvider implements QuadrupedPostureInputProviderInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterVector3D comPositionLowerLimitsParameter;
   private final ParameterVector3D comPositionUpperLimitsParameter;
   private final ParameterVector3D comVelocityLowerLimitsParameter;
   private final ParameterVector3D comVelocityUpperLimitsParameter;

   private final AtomicReference<ComPositionPacket> comPositionPacket;
   private final AtomicReference<ComVelocityPacket> comVelocityPacket;
   private final YoDouble yoComPositionInputX;
   private final YoDouble yoComPositionInputY;
   private final YoDouble yoComPositionInputZ;
   private final YoDouble yoComVelocityInputX;
   private final YoDouble yoComVelocityInputY;
   private final YoDouble yoComVelocityInputZ;
   private final Point3D comPositionInput;
   private final Vector3D comVelocityInput;

   public QuadrupedPostureInputProvider(QuadrupedPhysicalProperties physicalProperties, GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      comPositionPacket = new AtomicReference<>(new ComPositionPacket());
      comVelocityPacket = new AtomicReference<>(new ComVelocityPacket());
      yoComPositionInputX = new YoDouble("comPositionInputX", registry);
      yoComPositionInputY = new YoDouble("comPositionInputY", registry);
      yoComPositionInputZ = new YoDouble("comPositionInputZ", registry);
      yoComVelocityInputX = new YoDouble("comVelocityInputX", registry);
      yoComVelocityInputY = new YoDouble("comVelocityInputY", registry);
      yoComVelocityInputZ = new YoDouble("comVelocityInputZ", registry);
      comPositionInput = new Point3D();
      comVelocityInput = new Vector3D();

      Vector3D defaultComPositionLowerLimits = new Vector3D(-Double.MAX_VALUE, - Double.MAX_VALUE, 0.2);
      Vector3D negativeMaximumLimit = new Vector3D(- Double.MAX_VALUE, - Double.MAX_VALUE, - Double.MAX_VALUE);
      Vector3D positiveMaximumLimit = new Vector3D(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

      comPositionLowerLimitsParameter = new ParameterVector3D("comPositionLowerLimit", defaultComPositionLowerLimits, registry);
      comPositionUpperLimitsParameter = new ParameterVector3D("comPositionUpperLimit", positiveMaximumLimit, registry);
      comVelocityLowerLimitsParameter = new ParameterVector3D("comVelocityLowerLimit", negativeMaximumLimit, registry);
      comVelocityUpperLimitsParameter = new ParameterVector3D("comVelocityUpperLimit", positiveMaximumLimit, registry);

      // initialize com height
      yoComPositionInputZ.set(physicalProperties.getNominalCoMHeight());

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(ComPositionPacket.class, new PacketConsumer<ComPositionPacket>()
         {
            @Override
            public void receivedPacket(ComPositionPacket packet)
            {
               comPositionPacket.set(packet);
               yoComPositionInputX.set(
                     MathTools.clamp(comPositionPacket.get().getPosition().getX(), comPositionLowerLimitsParameter.getX(), comPositionUpperLimitsParameter.getX()));
               yoComPositionInputY.set(
                     MathTools.clamp(comPositionPacket.get().getPosition().getY(), comPositionLowerLimitsParameter.getY(), comPositionUpperLimitsParameter.getY()));
               yoComPositionInputZ.set(
                     MathTools.clamp(comPositionPacket.get().getPosition().getZ(), comPositionLowerLimitsParameter.getZ(), comPositionUpperLimitsParameter.getZ()));
            }
         });

         globalDataProducer.attachListener(ComVelocityPacket.class, new PacketConsumer<ComVelocityPacket>()
         {
            @Override
            public void receivedPacket(ComVelocityPacket packet)
            {
               comVelocityPacket.set(packet);
               yoComVelocityInputX.set(
                     MathTools.clamp(comVelocityPacket.get().getVelocity().getX(), comVelocityLowerLimitsParameter.getX(), comVelocityUpperLimitsParameter.getX()));
               yoComVelocityInputY.set(
                     MathTools.clamp(comVelocityPacket.get().getVelocity().getY(), comVelocityLowerLimitsParameter.getY(), comVelocityUpperLimitsParameter.getY()));
               yoComVelocityInputZ.set(
                     MathTools.clamp(comVelocityPacket.get().getVelocity().getZ(), comVelocityLowerLimitsParameter.getZ(), comVelocityUpperLimitsParameter.getZ()));
            }
         });

      }

      parentRegistry.addChild(registry);
   }

   @Override
   public Point3D getComPositionInput()
   {
      comPositionInput.set(yoComPositionInputX.getDoubleValue(), yoComPositionInputY.getDoubleValue(), yoComPositionInputZ.getDoubleValue());
      return comPositionInput;
   }

   @Override
   public Vector3D getComVelocityInput()
   {
      comVelocityInput.set(yoComVelocityInputX.getDoubleValue(), yoComVelocityInputY.getDoubleValue(), yoComVelocityInputZ.getDoubleValue());
      return comVelocityInput;
   }
}
