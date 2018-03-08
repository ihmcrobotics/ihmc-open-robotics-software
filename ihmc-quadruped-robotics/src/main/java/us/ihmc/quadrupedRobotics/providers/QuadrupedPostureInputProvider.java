package us.ihmc.quadrupedRobotics.providers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedRobotics.communication.packets.BodyAngularRatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComVelocityPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedPostureInputProvider implements QuadrupedPostureInputProviderInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterVector3D comPositionLowerLimitsParameter;
   private final ParameterVector3D comPositionUpperLimitsParameter;
   private final ParameterVector3D comVelocityLowerLimitsParameter;
   private final ParameterVector3D comVelocityUpperLimitsParameter;
   private final ParameterVector3D bodyOrientationLowerLimitsParameter;
   private final ParameterVector3D bodyOrientationUpperLimitsParameter;
   private final ParameterVector3D bodyAngularRateLowerLimitsParameter;
   private final ParameterVector3D bodyAngularRateUpperLimitsParameter;

   private final AtomicReference<ComPositionPacket> comPositionPacket;
   private final AtomicReference<ComVelocityPacket> comVelocityPacket;
   private final AtomicReference<BodyOrientationPacket> bodyOrientationPacket;
   private final AtomicReference<BodyAngularRatePacket> bodyAngularRatePacket;
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
   private final Point3D comPositionInput;
   private final Vector3D comVelocityInput;
   private final Quaternion bodyOrientationInput;
   private final Vector3D bodyAngularRateInput;

   public QuadrupedPostureInputProvider(QuadrupedPhysicalProperties physicalProperties, GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      comPositionPacket = new AtomicReference<>(new ComPositionPacket());
      comVelocityPacket = new AtomicReference<>(new ComVelocityPacket());
      bodyOrientationPacket = new AtomicReference<>(new BodyOrientationPacket());
      bodyAngularRatePacket = new AtomicReference<>(new BodyAngularRatePacket());
      yoComPositionInputX = new YoDouble("comPositionInputX", registry);
      yoComPositionInputY = new YoDouble("comPositionInputY", registry);
      yoComPositionInputZ = new YoDouble("comPositionInputZ", registry);
      yoComVelocityInputX = new YoDouble("comVelocityInputX", registry);
      yoComVelocityInputY = new YoDouble("comVelocityInputY", registry);
      yoComVelocityInputZ = new YoDouble("comVelocityInputZ", registry);
      yoBodyOrientationInputYaw = new YoDouble("bodyOrientationInputYaw", registry);
      yoBodyOrientationInputPitch = new YoDouble("bodyOrientationInputPitch", registry);
      yoBodyOrientationInputRoll = new YoDouble("bodyOrientationInputRoll", registry);
      yoBodyAngularRateInputX = new YoDouble("bodyAngularRateInputX", registry);
      yoBodyAngularRateInputY = new YoDouble("bodyAngularRateInputY", registry);
      yoBodyAngularRateInputZ = new YoDouble("bodyAngularRateInputZ", registry);
      comPositionInput = new Point3D();
      comVelocityInput = new Vector3D();
      bodyOrientationInput = new Quaternion();
      bodyAngularRateInput = new Vector3D();

      Vector3D defaultComPositionLowerLimits = new Vector3D(-Double.MAX_VALUE, - Double.MAX_VALUE, 0.2);
      Vector3D negativeMaximumLimit = new Vector3D(- Double.MAX_VALUE, - Double.MAX_VALUE, - Double.MAX_VALUE);
      Vector3D positiveMaximumLimit = new Vector3D(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

      comPositionLowerLimitsParameter = new ParameterVector3D("comPositionLowerLimit", defaultComPositionLowerLimits, registry);
      comPositionUpperLimitsParameter = new ParameterVector3D("comPositionUpperLimit", positiveMaximumLimit, registry);
      comVelocityLowerLimitsParameter = new ParameterVector3D("comVelocityLowerLimit", negativeMaximumLimit, registry);
      comVelocityUpperLimitsParameter = new ParameterVector3D("comVelocityUpperLimit", positiveMaximumLimit, registry);
      bodyOrientationLowerLimitsParameter = new ParameterVector3D("bodyOrientationLowerLimit", negativeMaximumLimit, registry);
      bodyOrientationUpperLimitsParameter = new ParameterVector3D("bodyOrientationUpperLimit", positiveMaximumLimit, registry);
      bodyAngularRateLowerLimitsParameter = new ParameterVector3D("bodyAngularRateLowerLimit", negativeMaximumLimit, registry);
      bodyAngularRateUpperLimitsParameter = new ParameterVector3D("bodyAngularRateUpperLimit", positiveMaximumLimit, registry);

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
                     MathTools.clamp(comPositionPacket.get().getX(), comPositionLowerLimitsParameter.getX(), comPositionUpperLimitsParameter.getX()));
               yoComPositionInputY.set(
                     MathTools.clamp(comPositionPacket.get().getY(), comPositionLowerLimitsParameter.getY(), comPositionUpperLimitsParameter.getY()));
               yoComPositionInputZ.set(
                     MathTools.clamp(comPositionPacket.get().getZ(), comPositionLowerLimitsParameter.getZ(), comPositionUpperLimitsParameter.getZ()));
            }
         });

         globalDataProducer.attachListener(ComVelocityPacket.class, new PacketConsumer<ComVelocityPacket>()
         {
            @Override
            public void receivedPacket(ComVelocityPacket packet)
            {
               comVelocityPacket.set(packet);
               yoComVelocityInputX.set(
                     MathTools.clamp(comVelocityPacket.get().getX(), comVelocityLowerLimitsParameter.getX(), comVelocityUpperLimitsParameter.getX()));
               yoComVelocityInputY.set(
                     MathTools.clamp(comVelocityPacket.get().getY(), comVelocityLowerLimitsParameter.getY(), comVelocityUpperLimitsParameter.getY()));
               yoComVelocityInputZ.set(
                     MathTools.clamp(comVelocityPacket.get().getZ(), comVelocityLowerLimitsParameter.getZ(), comVelocityUpperLimitsParameter.getZ()));
            }
         });

         globalDataProducer.attachListener(BodyOrientationPacket.class, new PacketConsumer<BodyOrientationPacket>()
         {
            @Override
            public void receivedPacket(BodyOrientationPacket packet)
            {
               bodyOrientationPacket.set(packet);
               yoBodyOrientationInputYaw.set(MathTools.clamp(bodyOrientationPacket.get().getYaw(), bodyOrientationLowerLimitsParameter.getX(),
                     bodyOrientationUpperLimitsParameter.getX()));
               yoBodyOrientationInputPitch.set(MathTools.clamp(bodyOrientationPacket.get().getPitch(), bodyOrientationLowerLimitsParameter.getY(),
                     bodyOrientationUpperLimitsParameter.getY()));
               yoBodyOrientationInputRoll.set(MathTools.clamp(bodyOrientationPacket.get().getRoll(), bodyOrientationLowerLimitsParameter.getZ(),
                     bodyOrientationUpperLimitsParameter.getZ()));
            }
         });

         globalDataProducer.attachListener(BodyAngularRatePacket.class, new PacketConsumer<BodyAngularRatePacket>()
         {
            @Override
            public void receivedPacket(BodyAngularRatePacket packet)
            {
               bodyAngularRatePacket.set(packet);
               yoBodyAngularRateInputX.set(MathTools.clamp(bodyAngularRatePacket.get().getX(), bodyAngularRateLowerLimitsParameter.getX(),
                     bodyAngularRateUpperLimitsParameter.getX()));
               yoBodyAngularRateInputY.set(MathTools.clamp(bodyAngularRatePacket.get().getY(), bodyAngularRateLowerLimitsParameter.getY(),
                     bodyAngularRateUpperLimitsParameter.getY()));
               yoBodyAngularRateInputZ.set(MathTools.clamp(bodyAngularRatePacket.get().getZ(), bodyAngularRateLowerLimitsParameter.getZ(),
                     bodyAngularRateUpperLimitsParameter.getZ()));
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

   @Override
   public Quaternion getBodyOrientationInput()
   {
      bodyOrientationInput.setYawPitchRoll(yoBodyOrientationInputYaw.getDoubleValue(), yoBodyOrientationInputPitch.getDoubleValue(), yoBodyOrientationInputRoll.getDoubleValue());
      return bodyOrientationInput;
   }

   @Override
   public Vector3D getBodyAngularRateInput()
   {
      bodyAngularRateInput.set(yoBodyAngularRateInputX.getDoubleValue(), yoBodyAngularRateInputY.getDoubleValue(), yoBodyAngularRateInputZ.getDoubleValue());
      return bodyAngularRateInput;
   }
}
