package us.ihmc.aware.communication;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.aware.packets.*;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.MathTools;
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
   private final Point3d comPositionInput;
   private final Vector3d comVelocityInput;
   private final Quat4d bodyOrientationInput;
   private final Vector3d bodyAngularRateInput;
   private final Vector3d planarVelocityInput;

   public QuadrupedControllerInputProvider(GlobalDataProducer globalDataProducer, ParameterMapRepository parameterMapRepository)
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

      comPositionPacket = new AtomicReference<>(new ComPositionPacket(0, 0, params.get(COM_HEIGHT_NOMINAL)));
      comVelocityPacket = new AtomicReference<>(new ComVelocityPacket());
      bodyOrientationPacket = new AtomicReference<>(new BodyOrientationPacket());
      bodyAngularRatePacket = new AtomicReference<>(new BodyAngularRatePacket());
      planarVelocityPacket = new AtomicReference<>(new PlanarVelocityPacket());

      comPositionInput = new Point3d(0.0, 0.0, params.get(COM_HEIGHT_NOMINAL));
      comVelocityInput = new Vector3d();
      bodyOrientationInput = new Quat4d();
      bodyAngularRateInput = new Vector3d();
      planarVelocityInput = new Vector3d();

      globalDataProducer.attachListener(ComPositionPacket.class, new PacketConsumer<ComPositionPacket>()
      {
         @Override
         public void receivedPacket(ComPositionPacket packet)
         {
            comPositionPacket.set(packet);
         }
      });

      globalDataProducer.attachListener(ComVelocityPacket.class, new PacketConsumer<ComVelocityPacket>()
      {
         @Override
         public void receivedPacket(ComVelocityPacket packet)
         {
            comVelocityPacket.set(packet);
         }
      });

      globalDataProducer.attachListener(BodyOrientationPacket.class, new PacketConsumer<BodyOrientationPacket>()
      {
         @Override
         public void receivedPacket(BodyOrientationPacket packet)
         {
            bodyOrientationPacket.set(packet);
         }
      });

      globalDataProducer.attachListener(BodyAngularRatePacket.class, new PacketConsumer<BodyAngularRatePacket>()
      {
         @Override
         public void receivedPacket(BodyAngularRatePacket packet)
         {
            bodyAngularRatePacket.set(packet);
         }
      });

      globalDataProducer.attachListener(PlanarVelocityPacket.class, new PacketConsumer<PlanarVelocityPacket>()
      {
         @Override
         public void receivedPacket(PlanarVelocityPacket packet)
         {
            planarVelocityPacket.set(packet);
         }
      });
   }

   public Point3d getComPositionInput()
   {
      return comPositionInput;
   }

   public Vector3d getComVelocityInput()
   {
      return comVelocityInput;
   }

   public Quat4d getBodyOrientationInput()
   {
      return bodyOrientationInput;
   }

   public Vector3d getBodyAngularRateInput()
   {
      return bodyAngularRateInput;
   }

   public Vector3d getPlanarVelocityInput()
   {
      return planarVelocityInput;
   }

   public void process()
   {
      double comPositionX = MathTools.clipToMinMax(comPositionPacket.get().getX(), params.get(COM_POSITION_LOWER_LIMITS, 0), params.get(COM_POSITION_UPPER_LIMITS, 0));
      double comPositionY = MathTools.clipToMinMax(comPositionPacket.get().getY(), params.get(COM_POSITION_LOWER_LIMITS, 1), params.get(COM_POSITION_UPPER_LIMITS, 1));
      double comPositionZ = MathTools.clipToMinMax(comPositionPacket.get().getZ(), params.get(COM_POSITION_LOWER_LIMITS, 2), params.get(COM_POSITION_UPPER_LIMITS, 2));
      comPositionInput.set(comPositionX, comPositionY, comPositionZ);

      double comVelocityX = MathTools.clipToMinMax(comVelocityPacket.get().getX(), params.get(COM_VELOCITY_LOWER_LIMITS, 0), params.get(COM_VELOCITY_UPPER_LIMITS, 0));
      double comVelocityY = MathTools.clipToMinMax(comVelocityPacket.get().getY(), params.get(COM_VELOCITY_LOWER_LIMITS, 1), params.get(COM_VELOCITY_UPPER_LIMITS, 1));
      double comVelocityZ = MathTools.clipToMinMax(comVelocityPacket.get().getZ(), params.get(COM_VELOCITY_LOWER_LIMITS, 2), params.get(COM_VELOCITY_UPPER_LIMITS, 2));
      comVelocityInput.set(comVelocityX, comVelocityY, comVelocityZ);

      double bodyYaw = MathTools.clipToMinMax(bodyOrientationPacket.get().getYaw(), params.get(BODY_ORIENTATION_LOWER_LIMITS, 0), params.get(BODY_ORIENTATION_UPPER_LIMITS, 0));
      double bodyPitch = MathTools.clipToMinMax(bodyOrientationPacket.get().getPitch(), params.get(BODY_ORIENTATION_LOWER_LIMITS, 1), params.get(BODY_ORIENTATION_UPPER_LIMITS, 1));
      double bodyRoll = MathTools.clipToMinMax(bodyOrientationPacket.get().getRoll(), params.get(BODY_ORIENTATION_LOWER_LIMITS, 2), params.get(BODY_ORIENTATION_UPPER_LIMITS, 2));
      RotationTools.convertYawPitchRollToQuaternion(bodyYaw, bodyPitch, bodyRoll, bodyOrientationInput);

      double bodyAngularRateX = MathTools.clipToMinMax(bodyAngularRatePacket.get().getX(), params.get(BODY_ANGULAR_RATE_LOWER_LIMITS, 0), params.get(BODY_ANGULAR_RATE_UPPER_LIMITS, 0));
      double bodyAngularRateY = MathTools.clipToMinMax(bodyAngularRatePacket.get().getY(), params.get(BODY_ANGULAR_RATE_LOWER_LIMITS, 1), params.get(BODY_ANGULAR_RATE_UPPER_LIMITS, 1));
      double bodyAngularRateZ = MathTools.clipToMinMax(bodyAngularRatePacket.get().getZ(), params.get(BODY_ANGULAR_RATE_LOWER_LIMITS, 2), params.get(BODY_ANGULAR_RATE_UPPER_LIMITS, 2));
      bodyAngularRateInput.set(bodyAngularRateX, bodyAngularRateY, bodyAngularRateZ);

      double planarVelocityX = MathTools.clipToMinMax(planarVelocityPacket.get().getX(), params.get(PLANAR_VELOCITY_LOWER_LIMITS, 0), params.get(PLANAR_VELOCITY_UPPER_LIMITS, 0));
      double planarVelocityY = MathTools.clipToMinMax(planarVelocityPacket.get().getY(), params.get(PLANAR_VELOCITY_LOWER_LIMITS, 1), params.get(PLANAR_VELOCITY_UPPER_LIMITS, 1));
      double planarVelocityZ = MathTools.clipToMinMax(planarVelocityPacket.get().getZ(), params.get(PLANAR_VELOCITY_LOWER_LIMITS, 2), params.get(PLANAR_VELOCITY_UPPER_LIMITS, 2));
      planarVelocityInput.set(planarVelocityX, planarVelocityY, planarVelocityZ);
   }
}
