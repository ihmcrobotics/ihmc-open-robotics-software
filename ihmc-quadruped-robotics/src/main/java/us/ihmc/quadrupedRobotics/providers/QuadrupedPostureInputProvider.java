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
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedPostureInputProvider implements QuadrupedPostureInputProviderInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double defaultCoMHeightNominalParameter = 0.55;
   private final DoubleParameter comHeightNominalParameter = new DoubleParameter("comHeightNominal", registry, defaultCoMHeightNominalParameter);
   private final DoubleParameter[] comPositionLowerLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] comPositionUpperLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] comVelocityLowerLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] comVelocityUpperLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] bodyOrientationLowerLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] bodyOrientationUpperLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] bodyAngularRateLowerLimitsParameter = new DoubleParameter[3];
   private final DoubleParameter[] bodyAngularRateUpperLimitsParameter = new DoubleParameter[3];

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

   public QuadrupedPostureInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
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

      for (int i = 0; i < 3; i++)
      {
         double negMaxDouble = - Double.MIN_VALUE;
         double maxDouble = Double.MAX_VALUE;
         double comLowerLimit = (i == 2) ? 0.2 : negMaxDouble;

         comPositionLowerLimitsParameter[i] = new DoubleParameter("comPositionLowerLimit" + Axis.values[i], registry, comLowerLimit);
         comPositionUpperLimitsParameter[i] = new DoubleParameter("comPositionUpperLimit" + Axis.values[i], registry, maxDouble);
         comVelocityLowerLimitsParameter[i] = new DoubleParameter("comVelocityLowerLimit" + Axis.values[i], registry, negMaxDouble);
         comVelocityUpperLimitsParameter[i] = new DoubleParameter("comVelocityUpperLimit" + Axis.values[i], registry, maxDouble);
         bodyOrientationLowerLimitsParameter[i] = new DoubleParameter("bodyOrientationLowerLimit" + Axis.values[i], registry, negMaxDouble);
         bodyOrientationUpperLimitsParameter[i] = new DoubleParameter("bodyOrientationUpperLimit" + Axis.values[i], registry, maxDouble);
         bodyAngularRateLowerLimitsParameter[i] = new DoubleParameter("bodyAngularRateLowerLimit" + Axis.values[i], registry, negMaxDouble);
         bodyAngularRateUpperLimitsParameter[i] = new DoubleParameter("bodyAngularRateUpperLimit" + Axis.values[i], registry, maxDouble);
      }

      // initialize com height
      yoComPositionInputZ.set(defaultCoMHeightNominalParameter);

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(ComPositionPacket.class, new PacketConsumer<ComPositionPacket>()
         {
            @Override
            public void receivedPacket(ComPositionPacket packet)
            {
               comPositionPacket.set(packet);
               yoComPositionInputX.set(
                     MathTools.clamp(comPositionPacket.get().getX(), comPositionLowerLimitsParameter[0].getValue(), comPositionUpperLimitsParameter[0].getValue()));
               yoComPositionInputY.set(
                     MathTools.clamp(comPositionPacket.get().getY(), comPositionLowerLimitsParameter[1].getValue(), comPositionUpperLimitsParameter[1].getValue()));
               yoComPositionInputZ.set(
                     MathTools.clamp(comPositionPacket.get().getZ(), comPositionLowerLimitsParameter[2].getValue(), comPositionUpperLimitsParameter[2].getValue()));
            }
         });

         globalDataProducer.attachListener(ComVelocityPacket.class, new PacketConsumer<ComVelocityPacket>()
         {
            @Override
            public void receivedPacket(ComVelocityPacket packet)
            {
               comVelocityPacket.set(packet);
               yoComVelocityInputX.set(
                     MathTools.clamp(comVelocityPacket.get().getX(), comVelocityLowerLimitsParameter[0].getValue(), comVelocityUpperLimitsParameter[0].getValue()));
               yoComVelocityInputY.set(
                     MathTools.clamp(comVelocityPacket.get().getY(), comVelocityLowerLimitsParameter[1].getValue(), comVelocityUpperLimitsParameter[1].getValue()));
               yoComVelocityInputZ.set(
                     MathTools.clamp(comVelocityPacket.get().getZ(), comVelocityLowerLimitsParameter[2].getValue(), comVelocityUpperLimitsParameter[2].getValue()));
            }
         });

         globalDataProducer.attachListener(BodyOrientationPacket.class, new PacketConsumer<BodyOrientationPacket>()
         {
            @Override
            public void receivedPacket(BodyOrientationPacket packet)
            {
               bodyOrientationPacket.set(packet);
               yoBodyOrientationInputYaw.set(MathTools.clamp(bodyOrientationPacket.get().getYaw(), bodyOrientationLowerLimitsParameter[0].getValue(),
                     bodyOrientationUpperLimitsParameter[0].getValue()));
               yoBodyOrientationInputPitch.set(MathTools.clamp(bodyOrientationPacket.get().getPitch(), bodyOrientationLowerLimitsParameter[1].getValue(),
                     bodyOrientationUpperLimitsParameter[1].getValue()));
               yoBodyOrientationInputRoll.set(MathTools.clamp(bodyOrientationPacket.get().getRoll(), bodyOrientationLowerLimitsParameter[2].getValue(),
                     bodyOrientationUpperLimitsParameter[2].getValue()));
            }
         });

         globalDataProducer.attachListener(BodyAngularRatePacket.class, new PacketConsumer<BodyAngularRatePacket>()
         {
            @Override
            public void receivedPacket(BodyAngularRatePacket packet)
            {
               bodyAngularRatePacket.set(packet);
               yoBodyAngularRateInputX.set(MathTools.clamp(bodyAngularRatePacket.get().getX(), bodyAngularRateLowerLimitsParameter[0].getValue(),
                     bodyAngularRateUpperLimitsParameter[0].getValue()));
               yoBodyAngularRateInputY.set(MathTools.clamp(bodyAngularRatePacket.get().getY(), bodyAngularRateLowerLimitsParameter[1].getValue(),
                     bodyAngularRateUpperLimitsParameter[1].getValue()));
               yoBodyAngularRateInputZ.set(MathTools.clamp(bodyAngularRatePacket.get().getZ(), bodyAngularRateLowerLimitsParameter[2].getValue(),
                     bodyAngularRateUpperLimitsParameter[2].getValue()));
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
