package us.ihmc.kalman.imu.testCases;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.kalman.imu.QuaternionBasedFullIMUKalmanFilter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.robotController.RobotController;

public class TestIMUKalmanFilterControllerJerryOne implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("TestIMUKalmanFilterControllerJerryOne");
   @SuppressWarnings("unused")
   private static final boolean USING_FAST_QUAT = true;

   public double G = 9.81;
   public double DEGREES = Math.PI / 180.0;
   public double PI = Math.PI;

   private Random random = new java.util.Random();

// private FullIMUKalmanFilter fullIMUKalmanFilter = new FullIMUKalmanFilter(.001);
// private QuaternionBasedFullIMUKalmanFilter fullIMUKalmanFilter = new QuaternionBasedJamaFullIMUKalmanFilter(.001);
// private QuaternionBasedFullIMUKalmanFilter fastFullIMUKalmanFilter = new QuaternionBasedArrayFullIMUKalmanFilter(.001);

   private QuaternionBasedFullIMUKalmanFilter imuKalmanFilter;
   private QuaternionBasedFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilterTwo;

// private final FastQuaternionBasedFullIMUKalmanFilter fastQuaternionBasedFullIMUKalmanFilter;

   private DenseMatrix64F accel = new DenseMatrix64F(3, 1);
   @SuppressWarnings("unused")
   private DenseMatrix64F theta = new DenseMatrix64F(3, 1);
   private DenseMatrix64F pqr = new DenseMatrix64F(3, 1);

   private DoubleYoVariable wx_amp = new DoubleYoVariable("wx_amp", registry);
   private DoubleYoVariable wy_amp = new DoubleYoVariable("wy_amp", registry);
   private DoubleYoVariable wz_amp = new DoubleYoVariable("wz_amp", registry);

   private DoubleYoVariable wx_freq = new DoubleYoVariable("wx_freq", registry);
   private DoubleYoVariable wy_freq = new DoubleYoVariable("wy_freq", registry);
   private DoubleYoVariable wz_freq = new DoubleYoVariable("wz_freq", registry);

   private DoubleYoVariable q_noise = new DoubleYoVariable("q_noise", registry);
   private DoubleYoVariable r_noise = new DoubleYoVariable("r_noise", registry);

   private DoubleYoVariable fx_amp = new DoubleYoVariable("fx_amp", registry);
   private DoubleYoVariable fy_amp = new DoubleYoVariable("fy_amp", registry);
   @SuppressWarnings("unused")
   private DoubleYoVariable fz_amp = new DoubleYoVariable("fz_amp", registry);

   private DoubleYoVariable fx_freq = new DoubleYoVariable("fx_freq", registry);
   private DoubleYoVariable fy_freq = new DoubleYoVariable("fy_freq", registry);
   @SuppressWarnings("unused")
   private DoubleYoVariable fz_freq = new DoubleYoVariable("fz_freq", registry);


// private YoVariable yaw_from_accel = new YoVariable("yaw_from_accel", registry);
// private YoVariable pitch_from_accel = new YoVariable("pitch_from_accel", registry);
// private YoVariable roll_from_accel = new YoVariable("roll_from_accel", registry);

   private DoubleYoVariable q0_from_accel = new DoubleYoVariable("q0_from_accel", registry);
   private DoubleYoVariable q1_from_accel = new DoubleYoVariable("q1_from_accel", registry);
   private DoubleYoVariable q2_from_accel = new DoubleYoVariable("q2_from_accel", registry);
   private DoubleYoVariable q3_from_accel = new DoubleYoVariable("q3_from_accel", registry);

   private DoubleYoVariable yaw_from_quat = new DoubleYoVariable("yaw_from_quat", registry);
   private DoubleYoVariable pitch_from_quat = new DoubleYoVariable("pitch_from_quat", registry);
   private DoubleYoVariable roll_from_quat = new DoubleYoVariable("roll_from_quat", registry);

   private DoubleYoVariable estimated_yaw = new DoubleYoVariable("estimated_yaw", registry);
   private DoubleYoVariable estimated_pitch = new DoubleYoVariable("estimated_pitch", registry);
   private DoubleYoVariable estimated_roll = new DoubleYoVariable("estimated_roll", registry);

   private DoubleYoVariable estimated_yaw_2 = new DoubleYoVariable("estimated_yaw_2", registry);
   private DoubleYoVariable estimated_pitch_2 = new DoubleYoVariable("estimated_pitch_2", registry);
   private DoubleYoVariable estimated_roll_2 = new DoubleYoVariable("estimated_roll_2", registry);

   private DoubleYoVariable x_accel = new DoubleYoVariable("x_accel", registry);
   private DoubleYoVariable y_accel = new DoubleYoVariable("y_accel", registry);
   private DoubleYoVariable z_accel = new DoubleYoVariable("z_accel", registry);

   private DoubleYoVariable compass_noise = new DoubleYoVariable("compass_noise", registry);
   @SuppressWarnings("unused")
   private DoubleYoVariable alpha_compass_noise = new DoubleYoVariable("alpha_compass_noise", registry);
   private DoubleYoVariable compass_offset = new DoubleYoVariable("heading_offset", registry);
   private DoubleYoVariable heading_noise = new DoubleYoVariable("heading_noise", registry);
   private DoubleYoVariable heading_sensor = new DoubleYoVariable("heading_sensor", registry);

   private DoubleYoVariable accel_noise = new DoubleYoVariable("accel_noise", registry);
   private DoubleYoVariable alpha_accel_noise = new DoubleYoVariable("alpha_accel_noise", registry);

   private DoubleYoVariable x_accel_noise = new DoubleYoVariable("x_accel_noise", registry);
   private DoubleYoVariable y_accel_noise = new DoubleYoVariable("y_accel_noise", registry);
   private DoubleYoVariable z_accel_noise = new DoubleYoVariable("z_accel_noise", registry);

   private DoubleYoVariable x_accel_sensor = new DoubleYoVariable("x_accel_sensor", registry);
   private DoubleYoVariable y_accel_sensor = new DoubleYoVariable("y_accel_sensor", registry);
   private DoubleYoVariable z_accel_sensor = new DoubleYoVariable("z_accel_sensor", registry);

   private DoubleYoVariable x_gyro = new DoubleYoVariable("x_gyro", registry);
   private DoubleYoVariable y_gyro = new DoubleYoVariable("y_gyro", registry);
   private DoubleYoVariable z_gyro = new DoubleYoVariable("z_gyro", registry);

   private DoubleYoVariable x_gyro_bias = new DoubleYoVariable("x_gyro_bias", registry);
   private DoubleYoVariable y_gyro_bias = new DoubleYoVariable("y_gyro_bias", registry);
   private DoubleYoVariable z_gyro_bias = new DoubleYoVariable("z_gyro_bias", registry);

   private DoubleYoVariable gyro_noise = new DoubleYoVariable("gyro_noise", registry);
   private DoubleYoVariable alpha_gyro_noise = new DoubleYoVariable("alpha_gyro_noise", registry);

   private DoubleYoVariable x_gyro_noise = new DoubleYoVariable("x_gyro_noise", registry);
   private DoubleYoVariable y_gyro_noise = new DoubleYoVariable("y_gyro_noise", registry);
   private DoubleYoVariable z_gyro_noise = new DoubleYoVariable("z_gyro_noise", registry);

   private DoubleYoVariable x_gyro_sensor = new DoubleYoVariable("x_gyro_sensor", registry);
   private DoubleYoVariable y_gyro_sensor = new DoubleYoVariable("y_gyro_sensor", registry);
   private DoubleYoVariable z_gyro_sensor = new DoubleYoVariable("z_gyro_sensor", registry);

   private DoubleYoVariable estimated_qd_wx_bias = new DoubleYoVariable("estimated_qd_wx_bias", registry);
   private DoubleYoVariable estimated_qd_wy_bias = new DoubleYoVariable("estimated_qd_wy_bias", registry);
   private DoubleYoVariable estimated_qd_wz_bias = new DoubleYoVariable("estimated_qd_wz_bias", registry);

   private DoubleYoVariable estimated_q0 = new DoubleYoVariable("estimated_q0", registry);
   private DoubleYoVariable estimated_q1 = new DoubleYoVariable("estimated_q1", registry);
   private DoubleYoVariable estimated_q2 = new DoubleYoVariable("estimated_q2", registry);
   private DoubleYoVariable estimated_q3 = new DoubleYoVariable("estimated_q3", registry);

   private DoubleYoVariable estimated_q0_2 = new DoubleYoVariable("estimated_q0_2", registry);
   private DoubleYoVariable estimated_q1_2 = new DoubleYoVariable("estimated_q1_2", registry);
   private DoubleYoVariable estimated_q2_2 = new DoubleYoVariable("estimated_q2_2", registry);
   private DoubleYoVariable estimated_q3_2 = new DoubleYoVariable("estimated_q3_2", registry);


   // Simulation variables
// private YoVariable dt = new YoVariable("dt", this);
   private DoubleYoVariable t;

   @SuppressWarnings("unused")
   private java.text.DecimalFormat fmt = new java.text.DecimalFormat();

   private final TestIMUKalmanFilterRobot robot;

   private boolean profiling;

   @SuppressWarnings("unused")
   private final DoubleYoVariable ef_body_fx, ef_body_fy, ef_body_fz;
   private String name;

   public TestIMUKalmanFilterControllerJerryOne(TestIMUKalmanFilterRobot robot,

// double dt,
   QuaternionBasedFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilter, QuaternionBasedFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilterTwo, String name)
   {
      this(robot, false, quaternionBasedFullIMUKalmanFilter, quaternionBasedFullIMUKalmanFilterTwo, name);
   }

   public TestIMUKalmanFilterControllerJerryOne(TestIMUKalmanFilterRobot robot,

// double dt,
   boolean profiling, QuaternionBasedFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilter,
                      QuaternionBasedFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilterTwo, String name)
   {
      this.name = name;
      this.profiling = profiling;
      this.imuKalmanFilter = quaternionBasedFullIMUKalmanFilter;
      this.quaternionBasedFullIMUKalmanFilterTwo = quaternionBasedFullIMUKalmanFilterTwo;

//    this.fastQuaternionBasedFullIMUKalmanFilter = quaternionBasedFullIMUKalmanFilterTwo;

//    this.dt.val = dt;
      t = (DoubleYoVariable)robot.getVariable("t");

      this.robot = robot;

      ef_body_fx =(DoubleYoVariable) robot.getVariable("ef_body_fx");
      ef_body_fy = (DoubleYoVariable)robot.getVariable("ef_body_fy");
      ef_body_fz = (DoubleYoVariable)robot.getVariable("ef_body_fz");


      initControl();

   }

   public void initControl()
   {
      long newClockTime = System.currentTimeMillis();
      random.setSeed(newClockTime);

//    wx_amp.val = 0.5;
//    wy_amp.val = 0.4;
//    wz_amp.val = 0.3;
//    wx_freq.val = 1.0;
//    wy_freq.val = 2.0;
//    wz_freq.val = 0.2;

      wx_amp.set(0.0);
      wy_amp.set(0.0);
      wz_amp.set(0.0);


      fx_amp.set(1.0);
      fx_freq.set(1.0);

      q_noise.set(5.0);
      r_noise.set(1.0);

      compass_noise.set(0.0);    // 1.0; //0.5; //0.0; //2.0; //1.0;
      compass_offset.set(0.0);    // 0.5;
      accel_noise.set(0.0);    // 1.0; //5.0; //1.0;
      gyro_noise.set(0.0);    // 1.0; //2.0; //1.0;

      x_gyro_bias.set(0.0);    // 0.5; //0.1;
      y_gyro_bias.set(0.0);    // 0.7; //0.2; //1.0;
      z_gyro_bias.set(0.0);    // 0.9; //0.3;

      // As of March 14, 2007 9AM, the following tests seemed to work ok:
//    robot.setYawPitchRoll(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

//    robot.setYawPitchRoll(Math.PI*0.34, 0.0, 0.0, 0.0, 0.0, 0.0);

//    robot.setYawPitchRoll(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
//    robot.setYawPitchRoll(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
//    robot.setYawPitchRoll(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

//    robot.setYawPitchRoll(-1.782, -0.55, 2.629, 0.0, 0.0, 0.0);

//    robot.setYawPitchRoll(0.0, 0.0, 0.0, 0.1, 0.298, 0.7689);
//    robot.setYawPitchRoll(0.0, 0.0, 0.0, 0.01, 1.0, 0.01);


      // As of March 14, 2007 9AM, the following tests did not seem to work ok:

//    robot.setYawPitchRoll(0.1, 0.0, 0.0, 0.0, 0.0, 0.05);

//    robot.setYawPitchRoll(Math.PI*1.34, Math.PI*1.27, Math.PI*1.82, 0.17, 1.219, 0.356);
//    robot.setYawPitchRoll(-1.782, -2.762, 2.629, 0.0, 0.0, 0.0);
//    robot.setYawPitchRoll(0.5, -Math.PI*0.49, 0.0, 0.0, 0.0, 0.0);

//    robot.setYawPitchRoll(0.5, -0.7, 0.0, 0.0, 0.0, 0.0);
//    robot.setYawPitchRoll(0.5, -0.7, 0.0, 0.1, 0.298, 0.7689);

//    robot.setYawPitchRoll(0.5, 0.0, 0.0, 0.1, 0.298, 0.7689);

//    robot.setYawPitchRoll(0.1, 0.2, 0.3, 0.01, 0.02, 1.0);

//    robot.setYawPitchRoll(0.7, -0.6, 0.8, 1.123, 2.105, 1.379);


//    robot.setYawPitchRoll(0.1, 0.2, 0.3, 1.0, 2.0, 1.0);
//    robot.setYawPitchRoll(0.1, 0.2, 0.3, 0.3, 0.2, 0.1);

//    robot.setYawPitchRoll(0.1, 0.2, 0.3, 0.01, 0.02, 0.01);


      robot.setXYZ(0.0, 0.0, 0.4, 0.0, 0.0, 0.0);

//    robot.update();
//    robot.updateYawPitchRoll();

      computeAccelerationsFromRobot();

      System.out.println("Initial Yaw = " + robot.yaw.getDoubleValue());

      accel.set(0, 0, -x_accel.getDoubleValue());
      accel.set(1, 0, -y_accel.getDoubleValue());
      accel.set(2, 0, -z_accel.getDoubleValue());

      pqr.set(0, 0, 0.0);    // x_gyro.val); // + x_gyro_bias.val);
      pqr.set(1, 0, 0.0);    // y_gyro.val); // + y_gyro_bias.val);
      pqr.set(2, 0, 0.0);    // z_gyro.val); // + z_gyro_bias.val);

//    if(USING_FAST_QUAT)
//    {
//       double[][] accel = new double[3][1];
//       double[][] pqr = new double[3][1];
//       accel[0][0] = -x_accel.val;
//       accel[1][0] = -y_accel.val;
//       accel[2][0] = -z_accel.val;
//
//       pqr[0][0] = 0.0;
//       pqr[1][0] = 0.0;
//       pqr[2][0] = 0.0;
//       fastQuaternionBasedFullIMUKalmanFilter.initialize(accel, pqr, robot.yaw.val);
//       fastQuaternionBasedFullIMUKalmanFilter.reset(fastQuaternionBasedFullIMUKalmanFilter.P);
//    }
//    else
      {
         imuKalmanFilter.initialize(accel, pqr, robot.yaw.getDoubleValue());
         imuKalmanFilter.reset();
         imuKalmanFilter.setNoiseParameters(q_noise.getDoubleValue(), r_noise.getDoubleValue());

         quaternionBasedFullIMUKalmanFilterTwo.initialize(accel, pqr, robot.yaw.getDoubleValue());
         quaternionBasedFullIMUKalmanFilterTwo.reset();
         quaternionBasedFullIMUKalmanFilterTwo.setNoiseParameters(q_noise.getDoubleValue(), r_noise.getDoubleValue());


      }
   }

   @SuppressWarnings("unused")
   private boolean doneStep = false;
   @SuppressWarnings("unused")
   private boolean doneSecondStep = false;

   public void doControl()
   {
      int numberOfTimesToCall = 1;
      long startTime = 0L;
      if (profiling)
      {
         numberOfTimesToCall = 1000000;
         startTime = System.currentTimeMillis();
         System.out.println("TestIMUKalmanFilterControllerJerryOne::doControl: starting control loop...");
      }

      for (int i = 0; i < numberOfTimesToCall; i++)
      {
         // if (t.val > 2.0 && !doneStep)
         // {
         // //         robot.setYawPitchRoll(0.0, Math.PI/4.0, 0.0, 0.0, 0.0, 0.0);
         // robot.setYawPitchRoll(0.0, 0.0, 0.0, 0.0, 12.0, 0.0);
         // doneStep = true;
         // }
         //
         // if (t.val > 2.2 && !doneSecondStep)
         // {
         // //         robot.setYawPitchRoll(0.0, Math.PI/4.0, 0.0, 0.0, 0.0, 0.0);
         // robot.qd_wx.val = 0.0;
         // robot.qd_wy.val = 0.0;
         // robot.qd_wz.val = 0.0;
         //
         // doneSecondStep = true;
         // }

         robot.qd_wx.set(wx_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * wx_freq.getDoubleValue() * t.getDoubleValue()));
         robot.qd_wy.set(wy_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * wy_freq.getDoubleValue() * t.getDoubleValue()));
         robot.qd_wz.set(wz_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * wz_freq.getDoubleValue() * t.getDoubleValue()));

         if (robot.qd_wx.getDoubleValue() > 0.0)
            robot.qd_wx.set(wx_amp.getDoubleValue());
         else
            robot.qd_wx.set(-wx_amp.getDoubleValue());

         if (robot.qd_wy.getDoubleValue() > 0.0)
            robot.qd_wy.set(wy_amp.getDoubleValue());
         else
            robot.qd_wy.set(-wy_amp.getDoubleValue());

         if (robot.qd_wz.getDoubleValue() > 0.0)
            robot.qd_wz.set(wz_amp.getDoubleValue());
         else
            robot.qd_wz.set(-wz_amp.getDoubleValue());


         robot.updateYawPitchRoll();

         // Apply upward force to prevent it from falling:
         robot.ef_body_fx.set(fx_amp.getDoubleValue() * Math.cos(2.0 * Math.PI * fx_freq.getDoubleValue() * t.getDoubleValue()));
         robot.ef_body_fy.set(fy_amp.getDoubleValue() * Math.cos(2.0 * Math.PI * fy_freq.getDoubleValue() * t.getDoubleValue()));
         robot.ef_body_fz.set(G);

         computeAccelerationsFromRobot();

         heading_noise.set(alpha_accel_noise.getDoubleValue() * heading_noise.getDoubleValue() + (1.0 - alpha_accel_noise.getDoubleValue()) * (random.nextGaussian() * 1.414 * compass_noise.getDoubleValue()));

         x_accel_noise.set(alpha_accel_noise.getDoubleValue() * x_accel_noise.getDoubleValue() + (1.0 - alpha_accel_noise.getDoubleValue()) * (random.nextGaussian() * 1.414 * accel_noise.getDoubleValue()));
         y_accel_noise.set(alpha_accel_noise.getDoubleValue() * y_accel_noise.getDoubleValue() + (1.0 - alpha_accel_noise.getDoubleValue()) * (random.nextGaussian() * 1.414 * accel_noise.getDoubleValue()));
         z_accel_noise.set(alpha_accel_noise.getDoubleValue() * z_accel_noise.getDoubleValue() + (1.0 - alpha_accel_noise.getDoubleValue()) * (random.nextGaussian() * 1.414 * accel_noise.getDoubleValue()));

         heading_sensor.set(robot.yaw.getDoubleValue() + heading_noise.getDoubleValue() + compass_offset.getDoubleValue());

         x_gyro_noise.set(alpha_gyro_noise.getDoubleValue() * x_gyro_noise.getDoubleValue() + (1.0 - alpha_gyro_noise.getDoubleValue()) * (random.nextGaussian() * 1.414 * gyro_noise.getDoubleValue()));
         y_gyro_noise.set(alpha_gyro_noise.getDoubleValue() * y_gyro_noise.getDoubleValue() + (1.0 - alpha_gyro_noise.getDoubleValue()) * (random.nextGaussian() * 1.414 * gyro_noise.getDoubleValue()));
         z_gyro_noise.set(alpha_gyro_noise.getDoubleValue() * z_gyro_noise.getDoubleValue() + (1.0 - alpha_gyro_noise.getDoubleValue()) * (random.nextGaussian() * 1.414 * gyro_noise.getDoubleValue()));

         x_accel_sensor.set(-x_accel.getDoubleValue() + x_accel_noise.getDoubleValue());
         y_accel_sensor.set(-y_accel.getDoubleValue() + y_accel_noise.getDoubleValue());
         z_accel_sensor.set(-z_accel.getDoubleValue() + z_accel_noise.getDoubleValue());

         accel.set(0, 0, x_accel_sensor.getDoubleValue());
         accel.set(1, 0, y_accel_sensor.getDoubleValue());
         accel.set(2, 0, z_accel_sensor.getDoubleValue());

         // pqr is in order of qd_wx, qd_wy, qd_wz

         x_gyro_sensor.set(x_gyro.getDoubleValue() + x_gyro_bias.getDoubleValue() + x_gyro_noise.getDoubleValue());
         y_gyro_sensor.set(y_gyro.getDoubleValue() + y_gyro_bias.getDoubleValue() + y_gyro_noise.getDoubleValue());
         z_gyro_sensor.set(z_gyro.getDoubleValue() + z_gyro_bias.getDoubleValue() + z_gyro_noise.getDoubleValue());

         pqr.set(0, 0, x_gyro_sensor.getDoubleValue());
         pqr.set(1, 0, y_gyro_sensor.getDoubleValue());
         pqr.set(2, 0, z_gyro_sensor.getDoubleValue());

         // Estimates just from the accelerometer:
         // Matrix eulerAngles = fullIMUKalmanFilter.accel2euler(accel, robot.yaw.val);

         // fullIMUKalmanFilter.accel2euler(accel, robot.yaw.val);
         //
         // roll_from_accel.val = fullIMUKalmanFilter.eulerAngles.get(0, 0);
         // pitch_from_accel.val = fullIMUKalmanFilter.eulerAngles.get(1, 0);
         // yaw_from_accel.val = fullIMUKalmanFilter.eulerAngles.get(2, 0);
         //
         double[] quaternions = new double[4];
         imuKalmanFilter.accel2quaternions(accel, heading_sensor.getDoubleValue(), quaternions);

         // Matrix quaternions = QuaternionTools.euler2quat(fullIMUKalmanFilter.eulerAngles);
         q0_from_accel.set(quaternions[0]);
         q1_from_accel.set(quaternions[1]);
         q2_from_accel.set(quaternions[2]);
         q3_from_accel.set(quaternions[3]);

         double[] yawPitchRoll = new double[3];
         Quaternion quaternion = new Quaternion(quaternions);
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, yawPitchRoll);

         //
         // Matrix euler = QuaternionTools.quat2euler(quaternions);
         //

         roll_from_quat.set(yawPitchRoll[2]);
         pitch_from_quat.set(yawPitchRoll[1]);
         yaw_from_quat.set(yawPitchRoll[0]);

//
//       if(USING_FAST_QUAT)
//       {
////          double[][] accel = new double[3][1];
////          double[][] pqr = new double[3][1];
////          accel[0][0] = x_accel_sensor.val;
////          accel[1][0] = y_accel_sensor.val;
////          accel[2][0] = z_accel_sensor.val;
////
////          pqr[0][0] = x_gyro_sensor.val;
////          pqr[1][0] = y_gyro_sensor.val;
////          pqr[2][0] = z_gyro_sensor.val;
//
////          fastFullIMUKalmanFilter.setNoiseParameters(q_noise.val, r_noise.val);
////          fastFullIMUKalmanFilter.imuUpdate(pqr);
////          fastFullIMUKalmanFilter.compassUpdate(heading_sensor.val, accel);
//
//          fastFullIMUKalmanFilter.setNoiseParameters(q_noise.val, r_noise.val);
//          fastFullIMUKalmanFilter.imuUpdate(pqr);
//          fastFullIMUKalmanFilter.compassUpdate(heading_sensor.val, accel);
//
//
//          estimated_qd_wx_bias.val = fastFullIMUKalmanFilter.getBias(0);
//          estimated_qd_wy_bias.val = fastFullIMUKalmanFilter.getBias(1);
//          estimated_qd_wz_bias.val = fastFullIMUKalmanFilter.getBias(2);
//
//          estimated_q0.val = fastFullIMUKalmanFilter.getQuaternion().get(0, 0);
//          estimated_q1.val = fastFullIMUKalmanFilter.getQuaternion().get(1, 0);
//          estimated_q2.val = fastFullIMUKalmanFilter.getQuaternion().get(2, 0);
//          estimated_q3.val = fastFullIMUKalmanFilter.getQuaternion().get(3, 0);
//
////          estimated_qd_wx_bias.val = fastFullIMUKalmanFilter.bias[0][0];
////          estimated_qd_wy_bias.val = fastFullIMUKalmanFilter.bias[1][0];
////          estimated_qd_wz_bias.val = fastFullIMUKalmanFilter.bias[2][0];
////          estimated_q0.val = fastFullIMUKalmanFilter.q0;
////          estimated_q1.val = fastFullIMUKalmanFilter.q1;
////          estimated_q2.val = fastFullIMUKalmanFilter.q2;
////          estimated_q3.val = fastFullIMUKalmanFilter.q3;
//
//          double[] quat = new double[]
//                          {estimated_q0.val, estimated_q1.val, estimated_q2.val, estimated_q3.val};
//          double[][] eulerMatrix = fastFullIMUKalmanFilter.quat2euler(fastFullIMUKalmanFilter.getQuaternion());
//
//          estimated_yaw.val = eulerMatrix[2][0];
//          estimated_pitch.val = eulerMatrix[1][0];
//          estimated_roll.val = eulerMatrix[0][0];
//       }
//       else
//       {


         quaternionBasedFullIMUKalmanFilterTwo.setNoiseParameters(q_noise.getDoubleValue(), r_noise.getDoubleValue());
         quaternionBasedFullIMUKalmanFilterTwo.imuUpdate(pqr);
         quaternionBasedFullIMUKalmanFilterTwo.compassUpdate(heading_sensor.getDoubleValue(), accel);

         double[] eulerMatrix_2 = new double[3];
         DenseMatrix64F quaternionMatrix = new DenseMatrix64F(4, 1);
         quaternionBasedFullIMUKalmanFilterTwo.getQuaternion(quaternionMatrix);
         quaternion.set(quaternionMatrix);
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, eulerMatrix_2);

         estimated_q0_2.set(quaternionMatrix.get(0, 0));
         estimated_q1_2.set(quaternionMatrix.get(1, 0));
         estimated_q2_2.set(quaternionMatrix.get(2, 0));
         estimated_q3_2.set(quaternionMatrix.get(3, 0));

         estimated_yaw_2.set(eulerMatrix_2[0]);
         estimated_pitch_2.set(eulerMatrix_2[1]);
         estimated_roll_2.set(eulerMatrix_2[0]);



         imuKalmanFilter.setNoiseParameters(q_noise.getDoubleValue(), r_noise.getDoubleValue());
         imuKalmanFilter.imuUpdate(pqr);
         imuKalmanFilter.compassUpdate(heading_sensor.getDoubleValue(), accel);

         estimated_qd_wx_bias.set(imuKalmanFilter.getBias(0));
         estimated_qd_wy_bias.set(imuKalmanFilter.getBias(1));
         estimated_qd_wz_bias.set(imuKalmanFilter.getBias(2));


         DenseMatrix64F quatMatrix = new DenseMatrix64F(4, 1);
         imuKalmanFilter.getQuaternion(quatMatrix);

         estimated_q0.set(quatMatrix.get(0, 0));
         estimated_q1.set(quatMatrix.get(1, 0));
         estimated_q2.set(quatMatrix.get(2, 0));
         estimated_q3.set(quatMatrix.get(3, 0));

         @SuppressWarnings("unused") double[] quat = new double[] {estimated_q0.getDoubleValue(), estimated_q1.getDoubleValue(), estimated_q2.getDoubleValue(), estimated_q3.getDoubleValue()};

         double[] eulerMatrix = new double[3];
         quaternion.set(quatMatrix);
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, eulerMatrix);

         estimated_yaw.set(eulerMatrix[2]);
         estimated_pitch.set(eulerMatrix[1]);
         estimated_roll.set(eulerMatrix[0]);

         // estimated_yaw.val = fullIMUKalmanFilter.eulerAngles.get(2, 0);
         // estimated_pitch.val = fullIMUKalmanFilter.eulerAngles.get(1, 0);
         // estimated_roll.val = fullIMUKalmanFilter.eulerAngles.get(0, 0);

         // // Show AHRS internal variables
         // x_error.val = ahrs.roll_error;
         // y_error.val = ahrs.pitch_error;
         // z_error.val = ahrs.heading_error;

//       }
      }

      if (profiling)
      {
         double msPerKalmanFilterIteration = ((double) ((System.currentTimeMillis() - startTime))) / numberOfTimesToCall;
         System.err.println("TestIMUKalmanFilterControllerJerryOne::doControl: msPerKalmanFilterIteration: " + msPerKalmanFilterIteration);
      }

   }

   public void computeAccelerationsFromRobot()
   {
      // Get the transformation matrices from world coordinates to body coordinate systems:
      RigidBodyTransform transformFromWorldToBody = new RigidBodyTransform();
      RigidBodyTransform transformFromBodyToWorld = new RigidBodyTransform();

      robot.getTransformFromWorld(transformFromBodyToWorld);
      transformFromWorldToBody.set(transformFromBodyToWorld);
      transformFromWorldToBody.invert();

      // Transform accelerations from world coordinates to body coordinates:
      Vector3D qddBody = new Vector3D(robot.qdd_x.getDoubleValue(), robot.qdd_y.getDoubleValue(), (robot.qdd_z.getDoubleValue() + 9.81));
      Vector3D wBody = new Vector3D(robot.qd_wx.getDoubleValue(), robot.qd_wy.getDoubleValue(), robot.qd_wz.getDoubleValue());

      transformFromWorldToBody.transform(qddBody);
      transformFromWorldToBody.transform(wBody);

      // Set the IMUs acceleration and rate gyroscope inputs.
      x_accel.set(qddBody.getX());
      y_accel.set(qddBody.getY());
      z_accel.set(qddBody.getZ());

      x_gyro.set(wBody.getX());
      y_gyro.set(wBody.getY());
      z_gyro.set(wBody.getZ());
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
   
   public String getName()
   {
      return name;
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }
}
