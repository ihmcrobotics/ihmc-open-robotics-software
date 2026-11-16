package us.ihmc.avatar.testTools.scs2.vio;

import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.simulatedSensors.SCS2SensorReader;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Makes the state estimator's kinematic-inertial odometry (KIO) drift in simulation the way it does on hardware, where the
 * simulated sensors are otherwise perfect. Nothing starts until {@link #start()}.
 * <ul>
 * <li><b>Yaw:</b> the estimated pelvis yaw drifts at a constant rate, like an IMU with a gyro bias. It writes the estimator's
 * {@code totalEstimatedYawDrift}, which the estimator removes from the IMU's yaw every tick, after switching off the yaw drift
 * compensation that would otherwise overwrite it. Walking with a heading error then makes the position drift too.</li>
 * <li><b>Leg kinematics:</b> the hip, knee and ankle pitch joint readings are scaled, like an encoder or link length calibration
 * error, so the estimated steps are the wrong length. The scale ramps in over a second, so the estimate doesn't jump.</li>
 * </ul>
 */
public class KIODriftInjector
{
   private static final double SCALE_RAMP_DURATION = 1.0;
   private static final LegJointName[] PITCH_JOINTS = {LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH, LegJointName.ANKLE_PITCH};

   private final double yawDriftRate;
   private final double legPitchScaleError;
   private final DoubleProvider simulationTime;
   private volatile boolean startRequested;
   private volatile double scaleStartTime = Double.NaN;
   private volatile double injectedYawDrift;

   /**
    * @param yawDriftRate       how fast the estimated yaw drifts, in rad/s, positive counterclockwise.
    * @param legPitchScaleError relative error of the leg pitch joint readings, e.g. 0.01 reads 1% more than the joints move.
    */
   public KIODriftInjector(SCS2AvatarTestingSimulation simulation, double yawDriftRate, double legPitchScaleError)
   {
      this.yawDriftRate = yawDriftRate;
      this.legPitchScaleError = legPitchScaleError;
      simulationTime = simulation.getSimulationConstructionSet().getTime();

      AvatarEstimatorThread estimatorThread = simulation.getAvatarSimulation().getEstimatorThread();
      if (!(estimatorThread.getSensorReader() instanceof SCS2SensorReader sensorReader))
         throw new IllegalStateException("The leg kinematics drift needs the simulation's sensor reader");
      HumanoidJointNameMap jointMap = simulation.getRobotModel().getJointMap();
      for (RobotSide side : RobotSide.values)
      {
         for (LegJointName pitchJoint : PITCH_JOINTS)
         {
            String jointName = jointMap.getLegJointName(side, pitchJoint);
            if (jointName == null)
               continue;
            sensorReader.setOneDoFJointConfigurationCorruptor(jointName, (q, joint) -> q * (1.0 + getScaleError()));
            sensorReader.setOneDoFJointVelocityCorruptor(jointName, (qd, joint) -> qd * (1.0 + getScaleError()));
         }
      }
      simulation.getAvatarSimulation().addRobotControllerOnEstimatorThread(new YawDrift(estimatorThread));
   }

   /** Starts drifting from the next estimator tick. */
   public void start()
   {
      startRequested = true;
   }

   /** The yaw drift injected so far: the estimated yaw minus the true one, in rad. */
   public double getInjectedYawDrift()
   {
      return injectedYawDrift;
   }

   /** On the simulation thread, where the joints are read. */
   private double getScaleError()
   {
      if (Double.isNaN(scaleStartTime))
         return 0.0;
      return legPitchScaleError * Math.min(1.0, Math.max(0.0, simulationTime.getValue() - scaleStartTime) / SCALE_RAMP_DURATION);
   }

   /** On the estimator thread, after the estimator. */
   private class YawDrift implements RobotController
   {
      private final AvatarEstimatorThread estimatorThread;
      private YoDouble totalEstimatedYawDrift;
      private long startTime = Long.MIN_VALUE;

      YawDrift(AvatarEstimatorThread estimatorThread)
      {
         this.estimatorThread = estimatorThread;
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return null;
      }

      @Override
      public void doControl()
      {
         if (totalEstimatedYawDrift == null)
         {
            YoRegistry registry = estimatorThread.getYoRegistry();
            YoVariable compensation = registry.findVariable("enableIMUDriftYawCompensation");
            YoVariable yawDrift = registry.findVariable("totalEstimatedYawDrift");
            if (!(compensation instanceof YoBoolean enableCompensation) || !(yawDrift instanceof YoDouble))
               throw new IllegalStateException("The estimator has no IMU yaw drift estimator to inject yaw drift with");
            // The compensation would overwrite the injected drift, or estimate it away while the robot stands.
            enableCompensation.set(false);
            totalEstimatedYawDrift = (YoDouble) yawDrift;
            totalEstimatedYawDrift.set(0.0);
         }

         long now = estimatorThread.getSensorReader().getProcessedSensorOutputMap().getWallTime();
         if (startRequested)
         {
            startRequested = false;
            startTime = now;
            scaleStartTime = simulationTime.getValue();
         }
         if (startTime == Long.MIN_VALUE)
            return;
         injectedYawDrift = yawDriftRate * (now - startTime) * 1.0e-9;
         // The estimator subtracts this from the IMU's yaw.
         totalEstimatedYawDrift.set(-injectedYawDrift);
      }
   }
}
