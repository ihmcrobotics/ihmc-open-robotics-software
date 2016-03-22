package us.ihmc.aware.communication;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.aware.packets.*;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RotationTools;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class QuadrupedControllerInputProvider
{
   private final static String COM_HEIGHT_NOMINAL = "comHeightNominal";
   private final static String COM_POSITION_LOWER_LIMITS = "comPositionLowerLimits";
   private final static String COM_POSITION_UPPER_LIMITS = "comPositionUpperLimits";
   private final static String COM_VELOCITY_LOWER_LIMITS = "comVelocityLowerLimits";
   private final static String COM_VELOCITY_UPPER_LIMITS = "comVelocityUpperLimits";
   private final static String BODY_ORIENTATION_LOWER_LIMITS = "bodyOrientationLowerLimits";
   private final static String BODY_ORIENTATION_UPPER_LIMITS = "bodyOrientationUpperLimits";
   private final static String BODY_ANGULAR_RATE_LOWER_LIMITS = "bodyAngularRateLowerLimits";
   private final static String BODY_ANGULAR_RATE_UPPER_LIMITS = "bodyAngularRateUpperLimits";
   private final static String PLANAR_VELOCITY_LOWER_LIMITS = "planarVelocityLowerLimits";
   private final static String PLANAR_VELOCITY_UPPER_LIMITS = "planarVelocityUpperLimits";

   private final ParameterMap params;
   private final AtomicReference<ComPositionPacket> comPositionPacket;
   private final AtomicReference<ComVelocityPacket> comVelocityPacket;
   private final AtomicReference<BodyOrientationPacket> bodyOrientationPacket;
   private final AtomicReference<BodyAngularRatePacket> bodyAngularRatePacket;
   private final AtomicReference<PlanarVelocityPacket> planarVelocityPacket;
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
   private final DoubleYoVariable yoPlanarVelocityInputX;
   private final DoubleYoVariable yoPlanarVelocityInputY;
   private final DoubleYoVariable yoPlanarVelocityInputZ;
   private final Point3d comPositionInput;
   private final Vector3d comVelocityInput;
   private final Quat4d bodyOrientationInput;
   private final Vector3d bodyAngularRateInput;
   private final Vector3d planarVelocityInput;


   public QuadrupedControllerInputProvider(GlobalDataProducer globalDataProducer, ParameterMapRepository parameterMapRepository, YoVariableRegistry registry)
   {
      params = parameterMapRepository.get(QuadrupedControllerInputProvider.class);
      params.setDefault(COM_HEIGHT_NOMINAL, 0.55);
      params.setDefault(COM_POSITION_LOWER_LIMITS, -Double.MAX_VALUE, -Double.MAX_VALUE, 0.0);
      params.setDefault(COM_POSITION_UPPER_LIMITS, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      params.setDefault(COM_VELOCITY_LOWER_LIMITS, -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
      params.setDefault(COM_VELOCITY_UPPER_LIMITS, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      params.setDefault(BODY_ORIENTATION_LOWER_LIMITS, -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
      params.setDefault(BODY_ORIENTATION_UPPER_LIMITS, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      params.setDefault(BODY_ANGULAR_RATE_LOWER_LIMITS, -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
      params.setDefault(BODY_ANGULAR_RATE_UPPER_LIMITS, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      params.setDefault(PLANAR_VELOCITY_LOWER_LIMITS, -Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);
      params.setDefault(PLANAR_VELOCITY_UPPER_LIMITS, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

      comPositionPacket = new AtomicReference<>(new ComPositionPacket());
      comVelocityPacket = new AtomicReference<>(new ComVelocityPacket());
      bodyOrientationPacket = new AtomicReference<>(new BodyOrientationPacket());
      bodyAngularRatePacket = new AtomicReference<>(new BodyAngularRatePacket());
      planarVelocityPacket = new AtomicReference<>(new PlanarVelocityPacket());
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
      yoPlanarVelocityInputX = new DoubleYoVariable("planarVelocityInputX", registry);
      yoPlanarVelocityInputY = new DoubleYoVariable("planarVelocityInputY", registry);
      yoPlanarVelocityInputZ = new DoubleYoVariable("planarVelocityInputZ", registry);
      comPositionInput = new Point3d();
      comVelocityInput = new Vector3d();
      bodyOrientationInput = new Quat4d();
      bodyAngularRateInput = new Vector3d();
      planarVelocityInput = new Vector3d();

      // initialize com height
      yoComPositionInputZ.set(params.get(COM_HEIGHT_NOMINAL));

      globalDataProducer.attachListener(ComPositionPacket.class, new PacketConsumer<ComPositionPacket>()
      {
         @Override
         public void receivedPacket(ComPositionPacket packet)
         {
            comPositionPacket.set(packet);
            yoComPositionInputX.set(MathTools.clipToMinMax(comPositionPacket.get().getX(), params.get(COM_POSITION_LOWER_LIMITS, 0), params.get(COM_POSITION_UPPER_LIMITS, 0)));
            yoComPositionInputY.set(MathTools.clipToMinMax(comPositionPacket.get().getY(), params.get(COM_POSITION_LOWER_LIMITS, 1), params.get(COM_POSITION_UPPER_LIMITS, 1)));
            yoComPositionInputZ.set(MathTools.clipToMinMax(comPositionPacket.get().getZ(), params.get(COM_POSITION_LOWER_LIMITS, 2), params.get(COM_POSITION_UPPER_LIMITS, 2)));
         }
      });

      globalDataProducer.attachListener(ComVelocityPacket.class, new PacketConsumer<ComVelocityPacket>()
      {
         @Override
         public void receivedPacket(ComVelocityPacket packet)
         {
            comVelocityPacket.set(packet);
            yoComVelocityInputX.set(MathTools.clipToMinMax(comVelocityPacket.get().getX(), params.get(COM_VELOCITY_LOWER_LIMITS, 0), params.get(COM_VELOCITY_UPPER_LIMITS, 0)));
            yoComVelocityInputY.set(MathTools.clipToMinMax(comVelocityPacket.get().getY(), params.get(COM_VELOCITY_LOWER_LIMITS, 1), params.get(COM_VELOCITY_UPPER_LIMITS, 1)));
            yoComVelocityInputZ.set(MathTools.clipToMinMax(comVelocityPacket.get().getZ(), params.get(COM_VELOCITY_LOWER_LIMITS, 2), params.get(COM_VELOCITY_UPPER_LIMITS, 2)));
         }
      });

      globalDataProducer.attachListener(BodyOrientationPacket.class, new PacketConsumer<BodyOrientationPacket>()
      {
         @Override
         public void receivedPacket(BodyOrientationPacket packet)
         {
            yoBodyOrientationInputYaw.set(MathTools.clipToMinMax(bodyOrientationPacket.get().getYaw(), params.get(BODY_ORIENTATION_LOWER_LIMITS, 0), params.get(BODY_ORIENTATION_UPPER_LIMITS, 0)));
            yoBodyOrientationInputPitch.set(MathTools.clipToMinMax(bodyOrientationPacket.get().getPitch(), params.get(BODY_ORIENTATION_LOWER_LIMITS, 1), params.get(BODY_ORIENTATION_UPPER_LIMITS, 1)));
            yoBodyOrientationInputRoll.set(MathTools.clipToMinMax(bodyOrientationPacket.get().getRoll(), params.get(BODY_ORIENTATION_LOWER_LIMITS, 2), params.get(BODY_ORIENTATION_UPPER_LIMITS, 2)));
            bodyOrientationPacket.set(packet);
         }
      });

      globalDataProducer.attachListener(BodyAngularRatePacket.class, new PacketConsumer<BodyAngularRatePacket>()
      {
         @Override
         public void receivedPacket(BodyAngularRatePacket packet)
         {
            yoBodyAngularRateInputX.set(MathTools.clipToMinMax(bodyAngularRatePacket.get().getX(), params.get(BODY_ANGULAR_RATE_LOWER_LIMITS, 0), params.get(BODY_ANGULAR_RATE_UPPER_LIMITS, 0)));
            yoBodyAngularRateInputY.set(MathTools.clipToMinMax(bodyAngularRatePacket.get().getY(), params.get(BODY_ANGULAR_RATE_LOWER_LIMITS, 1), params.get(BODY_ANGULAR_RATE_UPPER_LIMITS, 1)));
            yoBodyAngularRateInputZ.set(MathTools.clipToMinMax(bodyAngularRatePacket.get().getZ(), params.get(BODY_ANGULAR_RATE_LOWER_LIMITS, 2), params.get(BODY_ANGULAR_RATE_UPPER_LIMITS, 2)));
            bodyAngularRatePacket.set(packet);
         }
      });

      globalDataProducer.attachListener(PlanarVelocityPacket.class, new PacketConsumer<PlanarVelocityPacket>()
      {
         @Override
         public void receivedPacket(PlanarVelocityPacket packet)
         {
            yoPlanarVelocityInputX.set(MathTools.clipToMinMax(planarVelocityPacket.get().getX(), params.get(PLANAR_VELOCITY_LOWER_LIMITS, 0), params.get(PLANAR_VELOCITY_UPPER_LIMITS, 0)));
            yoPlanarVelocityInputY.set(MathTools.clipToMinMax(planarVelocityPacket.get().getY(), params.get(PLANAR_VELOCITY_LOWER_LIMITS, 1), params.get(PLANAR_VELOCITY_UPPER_LIMITS, 1)));
            yoPlanarVelocityInputZ.set(MathTools.clipToMinMax(planarVelocityPacket.get().getZ(), params.get(PLANAR_VELOCITY_LOWER_LIMITS, 2), params.get(PLANAR_VELOCITY_UPPER_LIMITS, 2)));
            planarVelocityPacket.set(packet);
         }
      });
   }

   public Point3d getComPositionInput()
   {
      comPositionInput.set(yoComPositionInputX.getDoubleValue(), yoComPositionInputY.getDoubleValue(), yoComPositionInputZ.getDoubleValue());
      return comPositionInput;
   }

   public Vector3d getComVelocityInput()
   {
      comVelocityInput.set(yoComVelocityInputX.getDoubleValue(), yoComVelocityInputY.getDoubleValue(), yoComVelocityInputZ.getDoubleValue());
      return comVelocityInput;
   }

   public Quat4d getBodyOrientationInput()
   {
      RotationTools.convertYawPitchRollToQuaternion(yoBodyOrientationInputYaw.getDoubleValue(), yoBodyOrientationInputPitch.getDoubleValue(), yoBodyOrientationInputRoll.getDoubleValue(), bodyOrientationInput);
      return bodyOrientationInput;
   }

   public Vector3d getBodyAngularRateInput()
   {
      bodyAngularRateInput.set(yoBodyAngularRateInputX.getDoubleValue(), yoBodyAngularRateInputY.getDoubleValue(), yoBodyAngularRateInputZ.getDoubleValue());
      return bodyAngularRateInput;
   }

   public Vector3d getPlanarVelocityInput()
   {
      planarVelocityInput.set(yoPlanarVelocityInputX.getDoubleValue(), yoPlanarVelocityInputY.getDoubleValue(), yoPlanarVelocityInputZ.getDoubleValue());
      return planarVelocityInput;
   }
}
