package us.ihmc.quadrupedRobotics.providers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedRobotics.communication.packets.BodyAngularRatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComVelocityPacket;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class QuadrupedPostureInputProvider implements QuadrupedPostureInputProviderInterface
{
   private final ParameterFactory parameterFactory = ParameterFactory.createWithoutRegistry(getClass());
   private final DoubleParameter comHeightNominalParameter = parameterFactory.createDouble("comHeightNominal", 0.55);
   private final DoubleArrayParameter comPositionLowerLimitsParameter = parameterFactory.createDoubleArray("comPositionLowerLimits", -Double.MAX_VALUE, -Double.MAX_VALUE, 0.2);
   private final DoubleArrayParameter comPositionUpperLimitsParameter = parameterFactory.createDoubleArray("comPositionUpperLimits", Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
   private final DoubleArrayParameter comVelocityLowerLimitsParameter = parameterFactory.createDoubleArray("comVelocityLowerLimits", -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
   private final DoubleArrayParameter comVelocityUpperLimitsParameter = parameterFactory.createDoubleArray("comVelocityUpperLimits", Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
   private final DoubleArrayParameter bodyOrientationLowerLimitsParameter = parameterFactory.createDoubleArray("bodyOrientationLowerLimits", -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
   private final DoubleArrayParameter bodyOrientationUpperLimitsParameter = parameterFactory.createDoubleArray("bodyOrientationUpperLimits", Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
   private final DoubleArrayParameter bodyAngularRateLowerLimitsParameter = parameterFactory.createDoubleArray("bodyAngularRateLowerLimits", -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
   private final DoubleArrayParameter bodyAngularRateUpperLimitsParameter = parameterFactory.createDoubleArray("bodyAngularRateUpperLimits", Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

   private final AtomicReference<ComPositionPacket> comPositionPacket;
   private final AtomicReference<ComVelocityPacket> comVelocityPacket;
   private final AtomicReference<BodyOrientationPacket> bodyOrientationPacket;
   private final AtomicReference<BodyAngularRatePacket> bodyAngularRatePacket;
   private final DoubleYoVariable yoComPositionInputX;
   private final DoubleYoVariable yoComPositionInputY;
   private final DoubleYoVariable yoComPositionInputZ;
   private final DoubleYoVariable yoComVelocityInputX;
   private final DoubleYoVariable yoComVelocityInputY;
   private final DoubleYoVariable yoComVelocityInputZ;
   private final DoubleYoVariable yoBodyOrientationInputYaw;
   private final DoubleYoVariable yoBodyOrientationInputPitch;
   private final DoubleYoVariable yoBodyOrientationInputRoll;
   private final DoubleYoVariable yoBodyAngularRateInputX;
   private final DoubleYoVariable yoBodyAngularRateInputY;
   private final DoubleYoVariable yoBodyAngularRateInputZ;
   private final Point3D comPositionInput;
   private final Vector3D comVelocityInput;
   private final Quaternion bodyOrientationInput;
   private final Vector3D bodyAngularRateInput;

   public QuadrupedPostureInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry registry)
   {
      comPositionPacket = new AtomicReference<>(new ComPositionPacket());
      comVelocityPacket = new AtomicReference<>(new ComVelocityPacket());
      bodyOrientationPacket = new AtomicReference<>(new BodyOrientationPacket());
      bodyAngularRatePacket = new AtomicReference<>(new BodyAngularRatePacket());
      yoComPositionInputX = new DoubleYoVariable("comPositionInputX", registry);
      yoComPositionInputY = new DoubleYoVariable("comPositionInputY", registry);
      yoComPositionInputZ = new DoubleYoVariable("comPositionInputZ", registry);
      yoComVelocityInputX = new DoubleYoVariable("comVelocityInputX", registry);
      yoComVelocityInputY = new DoubleYoVariable("comVelocityInputY", registry);
      yoComVelocityInputZ = new DoubleYoVariable("comVelocityInputZ", registry);
      yoBodyOrientationInputYaw = new DoubleYoVariable("bodyOrientationInputYaw", registry);
      yoBodyOrientationInputPitch = new DoubleYoVariable("bodyOrientationInputPitch", registry);
      yoBodyOrientationInputRoll = new DoubleYoVariable("bodyOrientationInputRoll", registry);
      yoBodyAngularRateInputX = new DoubleYoVariable("bodyAngularRateInputX", registry);
      yoBodyAngularRateInputY = new DoubleYoVariable("bodyAngularRateInputY", registry);
      yoBodyAngularRateInputZ = new DoubleYoVariable("bodyAngularRateInputZ", registry);
      comPositionInput = new Point3D();
      comVelocityInput = new Vector3D();
      bodyOrientationInput = new Quaternion();
      bodyAngularRateInput = new Vector3D();

      // initialize com height
      yoComPositionInputZ.set(comHeightNominalParameter.get());

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(ComPositionPacket.class, new PacketConsumer<ComPositionPacket>()
         {
            @Override
            public void receivedPacket(ComPositionPacket packet)
            {
               comPositionPacket.set(packet);
               yoComPositionInputX.set(
                     MathTools.clamp(comPositionPacket.get().getX(), comPositionLowerLimitsParameter.get(0), comPositionUpperLimitsParameter.get(0)));
               yoComPositionInputY.set(
                     MathTools.clamp(comPositionPacket.get().getY(), comPositionLowerLimitsParameter.get(1), comPositionUpperLimitsParameter.get(1)));
               yoComPositionInputZ.set(
                     MathTools.clamp(comPositionPacket.get().getZ(), comPositionLowerLimitsParameter.get(2), comPositionUpperLimitsParameter.get(2)));
            }
         });

         globalDataProducer.attachListener(ComVelocityPacket.class, new PacketConsumer<ComVelocityPacket>()
         {
            @Override
            public void receivedPacket(ComVelocityPacket packet)
            {
               comVelocityPacket.set(packet);
               yoComVelocityInputX.set(
                     MathTools.clamp(comVelocityPacket.get().getX(), comVelocityLowerLimitsParameter.get(0), comVelocityUpperLimitsParameter.get(0)));
               yoComVelocityInputY.set(
                     MathTools.clamp(comVelocityPacket.get().getY(), comVelocityLowerLimitsParameter.get(1), comVelocityUpperLimitsParameter.get(1)));
               yoComVelocityInputZ.set(
                     MathTools.clamp(comVelocityPacket.get().getZ(), comVelocityLowerLimitsParameter.get(2), comVelocityUpperLimitsParameter.get(2)));
            }
         });

         globalDataProducer.attachListener(BodyOrientationPacket.class, new PacketConsumer<BodyOrientationPacket>()
         {
            @Override
            public void receivedPacket(BodyOrientationPacket packet)
            {
               bodyOrientationPacket.set(packet);
               yoBodyOrientationInputYaw.set(MathTools.clamp(bodyOrientationPacket.get().getYaw(), bodyOrientationLowerLimitsParameter.get(0),
                     bodyOrientationUpperLimitsParameter.get(0)));
               yoBodyOrientationInputPitch.set(MathTools.clamp(bodyOrientationPacket.get().getPitch(), bodyOrientationLowerLimitsParameter.get(1),
                     bodyOrientationUpperLimitsParameter.get(1)));
               yoBodyOrientationInputRoll.set(MathTools.clamp(bodyOrientationPacket.get().getRoll(), bodyOrientationLowerLimitsParameter.get(2),
                     bodyOrientationUpperLimitsParameter.get(2)));
            }
         });

         globalDataProducer.attachListener(BodyAngularRatePacket.class, new PacketConsumer<BodyAngularRatePacket>()
         {
            @Override
            public void receivedPacket(BodyAngularRatePacket packet)
            {
               bodyAngularRatePacket.set(packet);
               yoBodyAngularRateInputX.set(MathTools.clamp(bodyAngularRatePacket.get().getX(), bodyAngularRateLowerLimitsParameter.get(0),
                     bodyAngularRateUpperLimitsParameter.get(0)));
               yoBodyAngularRateInputY.set(MathTools.clamp(bodyAngularRatePacket.get().getY(), bodyAngularRateLowerLimitsParameter.get(1),
                     bodyAngularRateUpperLimitsParameter.get(1)));
               yoBodyAngularRateInputZ.set(MathTools.clamp(bodyAngularRatePacket.get().getZ(), bodyAngularRateLowerLimitsParameter.get(2),
                     bodyAngularRateUpperLimitsParameter.get(2)));
            }
         });
      }
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
