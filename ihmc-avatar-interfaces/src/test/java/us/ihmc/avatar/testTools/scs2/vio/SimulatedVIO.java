package us.ihmc.avatar.testTools.scs2.vio;

import controller_msgs.RobotConfigurationData;
import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.perception.vio.VIOPelvisPoseProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.kinematics.PoseHistory;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseCorrectionMode;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Function;

/**
 * Emulates the VIO (visual-inertial odometry) of a robot's tracking camera in an SCS2 simulation, for end-to-end tests of the
 * state estimator's external pelvis pose corrections. It reports what the camera's positional tracking would, with the errors
 * of an {@link ErrorModel}: latency, noise, its own drift, dropouts, outliers, tracking loss, relocalization and restarts. The
 * poses reach the estimator as the robot's VIO pipeline sends them, chosen by {@link Delivery}.
 * <p>
 * Ground truth is sampled from the simulated robot on the camera's frame grid. Everything else runs on the estimator thread
 * after the estimator, with one seeded random generator, so a run doesn't depend on how fast the machine simulates. Timestamps
 * are on the estimator's clock: in simulation, the simulation time.
 * </p>
 * <p>
 * Use it in three steps:
 * <ol>
 * <li>Before the simulation is created, give the estimator {@link #getCommunicator()} as its external pelvis pose subscriber
 * (not needed for {@link Delivery#CAMERA_THROUGH_PROVIDER_OVER_ROS2}). The estimator must not use perfect sensors.</li>
 * <li>Once it's created, {@link #attach(SCS2AvatarTestingSimulation)}. VIO starts tracking, and the correction mode set with
 * {@link #setMode(PelvisPoseCorrectionMode)} is applied.</li>
 * <li>{@link #start()} when the test starts, e.g. when the robot starts walking. The scheduled events are timed from then, and the
 * {@link Statistics} start over.</li>
 * </ol>
 * </p>
 */
public class SimulatedVIO
{
   /** How the emulated VIO poses reach the estimator. */
   public enum Delivery
   {
      /**
       * Pelvis poses, anchored to the estimator's pelvis pose at each session's first frame as {@link VIOPelvisPoseProvider} does,
       * handed straight to the estimator. Tests the estimator side alone.
       */
      PELVIS_DIRECT,
      /** Camera poses through the real {@link VIOPelvisPoseProvider}, whose pelvis poses are handed to the estimator in process. */
      CAMERA_THROUGH_PROVIDER,
      /**
       * Like {@link #CAMERA_THROUGH_PROVIDER}, but the provider publishes on ROS 2 and the estimator receives the poses on its
       * standard topic. Delivery then depends on how fast the machine runs, so only loose checks hold.
       */
      CAMERA_THROUGH_PROVIDER_OVER_ROS2
   }

   /** Something that happens to the VIO, at {@code time} seconds after {@link #start()}. */
   public sealed interface Event permits TrackingLoss, Restart, Relocalization, Jump
   {
      double time();
   }

   /** The camera loses tracking from {@code time} to {@code endTime}: its frames carry no pose, and its own drift continues. */
   public record TrackingLoss(double time, double endTime) implements Event
   {
   }

   /** The VIO restarts with a new session, like the operator's Reset VIO. */
   public record Restart(double time) implements Event
   {
   }

   /** A loop closure: VIO's own drift drops by {@code fractionOfDriftRemoved} at once. */
   public record Relocalization(double time, double fractionOfDriftRemoved) implements Event
   {
   }

   /** VIO's world jumps by {@code jump}, applied in the VIO world, and stays there. */
   public record Jump(double time, RigidBodyTransformReadOnly jump) implements Event
   {
   }

   /**
    * The errors of the emulated VIO. Nothing in the repositories characterizes the hardware's VIO, so the presets are starting
    * points to compare with a hardware log.
    *
    * @param frameRate              camera frames per second.
    * @param meanLatency            mean time from a frame's exposure until its pose reaches the provider, or the estimator for
    *                               {@link Delivery#PELVIS_DIRECT}, in s. Never less than one estimator tick.
    * @param latencyJitter          standard deviation of the latency, in s. Frames still arrive in order.
    * @param positionNoise          standard deviation of the white noise on each position axis, in m.
    * @param orientationNoise       standard deviation of the white noise on each rotation axis, in rad.
    * @param positionRandomWalk     VIO's own position drift on each axis, in m/√s.
    * @param yawRandomWalk          VIO's own yaw drift, in rad/√s. Gravity keeps roll and pitch from drifting.
    * @param scaleError             relative error of the distances VIO measures from its session's origin.
    * @param dropoutProbability     probability a frame is dropped entirely.
    * @param outlierProbability     probability a frame is an outlier, moved horizontally by one of {@code outlierMagnitudes}.
    * @param outlierMagnitudes      the outlier offsets, in m, picked at random.
    * @param clockOffset            error of the camera's clock against the estimator's, in s, added to each exposure timestamp.
    * @param extrinsicError         error of the camera mounting the provider assumes: the camera is really at the assumed pose
    *                               times this. Only with a provider.
    * @param randomizeSessionOrigin whether each session's VIO world gets a random origin and heading, which the anchor removes.
    * @param events                 what happens to the VIO, in seconds after {@link #start()}.
    * @param seed                   seed of all the randomness.
    */
   public record ErrorModel(double frameRate,
                            double meanLatency,
                            double latencyJitter,
                            double positionNoise,
                            double orientationNoise,
                            double positionRandomWalk,
                            double yawRandomWalk,
                            double scaleError,
                            double dropoutProbability,
                            double outlierProbability,
                            List<Double> outlierMagnitudes,
                            double clockOffset,
                            RigidBodyTransformReadOnly extrinsicError,
                            boolean randomizeSessionOrigin,
                            List<Event> events,
                            long seed)
   {
      /** Exact poses at 30 Hz, 40 ms late. */
      public static ErrorModel perfect()
      {
         return new ErrorModel(30.0, 0.040, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, List.of(), 0.0, new RigidBodyTransform(), false, List.of(), 1L);
      }

      /** A camera tracking well. */
      public static ErrorModel nominal()
      {
         return new ErrorModel(30.0,
                               0.045,
                               0.010,
                               0.003,
                               Math.toRadians(0.2),
                               0.001,
                               Math.toRadians(0.02),
                               0.005,
                               0.02,
                               0.0,
                               List.of(),
                               0.0,
                               new RigidBodyTransform(),
                               false,
                               List.of(),
                               1L);
      }

      /** A camera tracking badly: slower, noisier, drifting faster, with outliers that pass and fail the estimator's checks. */
      public static ErrorModel degraded()
      {
         return new ErrorModel(30.0,
                               0.090,
                               0.030,
                               0.008,
                               Math.toRadians(0.5),
                               0.003,
                               Math.toRadians(0.05),
                               0.015,
                               0.05,
                               0.02,
                               List.of(0.5, 0.12),
                               0.0,
                               new RigidBodyTransform(),
                               true,
                               List.of(),
                               1L);
      }

      public ErrorModel withLatency(double meanLatency, double latencyJitter)
      {
         return new ErrorModel(frameRate, meanLatency, latencyJitter, positionNoise, orientationNoise, positionRandomWalk, yawRandomWalk, scaleError,
                               dropoutProbability, outlierProbability, outlierMagnitudes, clockOffset, extrinsicError, randomizeSessionOrigin, events, seed);
      }

      public ErrorModel withClockOffset(double clockOffset)
      {
         return new ErrorModel(frameRate, meanLatency, latencyJitter, positionNoise, orientationNoise, positionRandomWalk, yawRandomWalk, scaleError,
                               dropoutProbability, outlierProbability, outlierMagnitudes, clockOffset, extrinsicError, randomizeSessionOrigin, events, seed);
      }

      public ErrorModel withRandomizedSessionOrigin(boolean randomizeSessionOrigin)
      {
         return new ErrorModel(frameRate, meanLatency, latencyJitter, positionNoise, orientationNoise, positionRandomWalk, yawRandomWalk, scaleError,
                               dropoutProbability, outlierProbability, outlierMagnitudes, clockOffset, extrinsicError, randomizeSessionOrigin, events, seed);
      }

      /** These events in addition to the model's own. */
      public ErrorModel withEvents(Event... moreEvents)
      {
         List<Event> allEvents = new ArrayList<>(events);
         allEvents.addAll(List.of(moreEvents));
         return new ErrorModel(frameRate, meanLatency, latencyJitter, positionNoise, orientationNoise, positionRandomWalk, yawRandomWalk, scaleError,
                               dropoutProbability, outlierProbability, outlierMagnitudes, clockOffset, extrinsicError, randomizeSessionOrigin, allEvents, seed);
      }

      public ErrorModel withSeed(long seed)
      {
         return new ErrorModel(frameRate, meanLatency, latencyJitter, positionNoise, orientationNoise, positionRandomWalk, yawRandomWalk, scaleError,
                               dropoutProbability, outlierProbability, outlierMagnitudes, clockOffset, extrinsicError, randomizeSessionOrigin, events, seed);
      }
   }

   private static final long ROBOT_STATE_PERIOD = 20_000_000L;
   private static final long TRUTH_SAMPLE_RETENTION = 2_000_000_000L;
   private static final int POSE_HISTORY_CAPACITY = 1000;

   private final DRCRobotModel robotModel;
   private final Delivery delivery;
   private final ErrorModel errorModel;
   private final Function<HumanoidReferenceFrames, ReferenceFrame> cameraFrameSelector;
   private final RigidBodyTransform sensorOffset = new RigidBodyTransform();
   private final PelvisPoseCorrectionCommunicator communicator;
   private final Statistics statistics = new Statistics();
   private final List<Event> scheduledEvents;

   private final ConcurrentLinkedQueue<TruthSample> truthSamples = new ConcurrentLinkedQueue<>();
   private volatile PelvisPoseCorrectionMode requestedMode = PelvisPoseCorrectionMode.EKF;
   private volatile Boolean requestedApplyCorrection = null;
   private volatile boolean restartRequested;
   private volatile boolean startRequested;
   private volatile boolean statisticsResetRequested;

   private SCS2AvatarTestingSimulation simulation;
   private VIOPelvisPoseProvider provider;

   /**
    * Emulates VIO on the robot's pelvis: {@link Delivery#PELVIS_DIRECT}.
    */
   public static SimulatedVIO pelvisDirect(DRCRobotModel robotModel, ErrorModel errorModel)
   {
      return new SimulatedVIO(robotModel, Delivery.PELVIS_DIRECT, errorModel, null, new RigidBodyTransform());
   }

   /**
    * Emulates VIO on a camera, through the real {@link VIOPelvisPoseProvider}.
    *
    * @param cameraFrameSelector the camera's mounting frame, as the provider is given it on the robot.
    * @param sensorOffset        the transform from the mounting frame to the frame VIO tracks, e.g. a stereo camera's left lens.
    * @param overROS2            whether the provider publishes on ROS 2 instead of handing poses to the estimator in process.
    */
   public static SimulatedVIO cameraThroughProvider(DRCRobotModel robotModel,
                                                    ErrorModel errorModel,
                                                    Function<HumanoidReferenceFrames, ReferenceFrame> cameraFrameSelector,
                                                    RigidBodyTransformReadOnly sensorOffset,
                                                    boolean overROS2)
   {
      Delivery delivery = overROS2 ? Delivery.CAMERA_THROUGH_PROVIDER_OVER_ROS2 : Delivery.CAMERA_THROUGH_PROVIDER;
      return new SimulatedVIO(robotModel, delivery, errorModel, cameraFrameSelector, sensorOffset);
   }

   private SimulatedVIO(DRCRobotModel robotModel,
                        Delivery delivery,
                        ErrorModel errorModel,
                        Function<HumanoidReferenceFrames, ReferenceFrame> cameraFrameSelector,
                        RigidBodyTransformReadOnly sensorOffset)
   {
      this.robotModel = robotModel;
      this.delivery = delivery;
      this.errorModel = errorModel;
      this.cameraFrameSelector = cameraFrameSelector;
      this.sensorOffset.set(sensorOffset);
      communicator = delivery == Delivery.CAMERA_THROUGH_PROVIDER_OVER_ROS2 ? null : new PelvisPoseCorrectionCommunicator(null, null);
      scheduledEvents = new ArrayList<>(errorModel.events());
      scheduledEvents.sort(Comparator.comparingDouble(Event::time));
   }

   /**
    * The estimator's external pelvis pose subscriber, to give the simulation factory before the simulation is created, or
    * {@code null} for {@link Delivery#CAMERA_THROUGH_PROVIDER_OVER_ROS2}, where the estimator uses its own.
    */
   public PelvisPoseCorrectionCommunicator getCommunicator()
   {
      return communicator;
   }

   public Delivery getDelivery()
   {
      return delivery;
   }

   /** Starts sampling ground truth and tracking. Call once the simulation is created and before {@link #start()}. */
   public void attach(SCS2AvatarTestingSimulation simulation)
   {
      if (this.simulation != null)
         throw new IllegalStateException("Already attached");
      this.simulation = simulation;
      AvatarEstimatorThread estimatorThread = simulation.getAvatarSimulation().getEstimatorThread();
      EstimatorLoop estimatorLoop = new EstimatorLoop(estimatorThread);
      if (delivery != Delivery.PELVIS_DIRECT)
      {
         provider = new VIOPelvisPoseProvider(robotModel.getSimpleRobotName(),
                                              robotModel.createFullRobotModel(),
                                              robotModel.getSensorInformation(),
                                              cameraFrameSelector,
                                              delivery == Delivery.CAMERA_THROUGH_PROVIDER_OVER_ROS2 ? simulation.getROS2Node() : null,
                                              estimatorLoop.providerClock::get,
                                              estimatorLoop::receivedProviderPelvisPose);
      }
      simulation.getRobot().getControllerManager().addController(new TruthSampler(simulation));
      simulation.getAvatarSimulation().addRobotControllerOnEstimatorThread(estimatorLoop);
   }

   /** Starts the clock of the scheduled events and the statistics. Takes effect on the next estimator tick. */
   public void start()
   {
      startRequested = true;
   }

   /** Starts the statistics over, on the next estimator tick. */
   public void resetStatistics()
   {
      statisticsResetRequested = true;
   }

   /** Restarts VIO with a new session on the next estimator tick, as the operator's Reset VIO does. */
   public void requestRestart()
   {
      restartRequested = true;
   }

   /** Switches the estimator's pelvis pose corrections on the next estimator tick. The default is {@link PelvisPoseCorrectionMode#EKF}. */
   public void setMode(PelvisPoseCorrectionMode mode)
   {
      requestedMode = mode;
   }

   /** Sets whether the EKF's corrected pose replaces the estimator's pelvis pose, on the next estimator tick. */
   public void setApplyEKFCorrection(boolean apply)
   {
      requestedApplyCorrection = apply;
   }

   public Statistics getStatistics()
   {
      return statistics;
   }

   /** Ground truth at a camera exposure: the pelvis pose and the pose of the frame VIO tracks, in the simulation's world. */
   private record TruthSample(long exposureTime, RigidBodyTransform pelvisPose, RigidBodyTransform trackedFramePose)
   {
   }

   /** A tracked frame on its way to the provider or the estimator. {@code poseInVIOWorld} is null when tracking is lost. */
   private record Frame(long exposureTime, long deliveryTime, int session, RigidBodyTransform poseInVIOWorld, RigidBodyTransform truePelvisPose)
   {
   }

   /** Samples ground truth on the camera's frame grid, on the simulation thread. */
   private class TruthSampler implements Controller
   {
      private final DoubleProvider time;
      private final Robot robot;
      private final long framePeriod = Math.round(1.0e9 / errorModel.frameRate());
      private final FullHumanoidRobotModel truthModel;
      private final HumanoidReferenceFrames truthFrames;
      private final ReferenceFrame cameraFrame;
      private final List<OneDoFJointBasics> truthJoints = new ArrayList<>();
      private final List<OneDoFJointReadOnly> simulatedJoints = new ArrayList<>();
      private long nextExposureTime = Long.MIN_VALUE;

      TruthSampler(SCS2AvatarTestingSimulation simulation)
      {
         time = simulation.getSimulationConstructionSet().getTime();
         robot = simulation.getRobot();
         if (delivery == Delivery.PELVIS_DIRECT)
         {
            truthModel = null;
            truthFrames = null;
            cameraFrame = null;
            return;
         }
         truthModel = robotModel.createFullRobotModel();
         truthFrames = new HumanoidReferenceFrames(truthModel, robotModel.getSensorInformation());
         cameraFrame = cameraFrameSelector.apply(truthFrames);
         for (OneDoFJointBasics joint : truthModel.getOneDoFJoints())
         {
            if (robot.getJoint(joint.getName()) instanceof OneDoFJointReadOnly simulatedJoint)
            {
               truthJoints.add(joint);
               simulatedJoints.add(simulatedJoint);
            }
         }
      }

      @Override
      public void doControl()
      {
         long now = Conversions.secondsToNanoseconds(time.getValue());
         if (now <= 0 || now < nextExposureTime)
            return;
         nextExposureTime = nextExposureTime == Long.MIN_VALUE ? now + framePeriod : Math.max(nextExposureTime + framePeriod, now);

         RigidBodyTransform pelvisPose = new RigidBodyTransform();
         robot.getFloatingRootJoint().getJointPose().get(pelvisPose);
         RigidBodyTransform trackedFramePose;
         if (delivery == Delivery.PELVIS_DIRECT)
         {
            trackedFramePose = pelvisPose;
         }
         else
         {
            truthModel.getRootJoint().getJointPose().set(pelvisPose);
            for (int i = 0; i < truthJoints.size(); i++)
               truthJoints.get(i).setQ(simulatedJoints.get(i).getQ());
            truthModel.updateFrames();
            truthFrames.updateFrames();
            trackedFramePose = new RigidBodyTransform(cameraFrame.getTransformToWorldFrame());
            trackedFramePose.multiply(sensorOffset);
            trackedFramePose.multiply(errorModel.extrinsicError());
         }
         truthSamples.add(new TruthSample(now, pelvisPose, trackedFramePose));
      }
   }

   /**
    * Runs on the estimator thread after the estimator: tracks the sampled frames with the error model, delivers them when their
    * latency is up, plays the camera's part of the provider's protocol, and measures the estimator's errors.
    */
   private class EstimatorLoop implements RobotController
   {
      private final AvatarEstimatorThread estimatorThread;
      private final FullHumanoidRobotModel estimatorModel;
      private final Random random = new Random(errorModel.seed());
      private final AtomicLong providerClock = new AtomicLong();
      private final long minimumLatency;
      private final long clockOffset = Math.round(errorModel.clockOffset() * 1.0e9);

      private final PoseHistory estimatedPelvisPoseHistory = new PoseHistory(POSE_HISTORY_CAPACITY);
      private final PoseHistory kioPelvisPoseHistory = new PoseHistory(POSE_HISTORY_CAPACITY);
      private final PoseHistory ekfOutputPoseHistory = new PoseHistory(POSE_HISTORY_CAPACITY);
      private final ArrayDeque<Frame> framesInFlight = new ArrayDeque<>();
      private final ArrayDeque<TruthSample> recentTruthSamples = new ArrayDeque<>();

      private final RigidBodyTransform estimatedPelvisPose = new RigidBodyTransform();
      private final RigidBodyTransform kioPelvisPose = new RigidBodyTransform();
      private final RigidBodyTransform ekfOutputPose = new RigidBodyTransform();
      private final RigidBodyTransform previousCorrection = new RigidBodyTransform();
      private final RigidBodyTransform correction = new RigidBodyTransform();
      private final Quaternion orientation = new Quaternion();

      // The tracker: the VIO world of the current session and VIO's own drift in it.
      private int session;
      private boolean sessionHasWorld;
      private final RigidBodyTransform vioWorld = new RigidBodyTransform();
      private final RigidBodyTransform drift = new RigidBodyTransform();
      private long lastTrackedExposureTime = Long.MIN_VALUE;
      // The pelvis-direct anchor from the VIO world to the robot's world.
      private boolean anchored;
      private final RigidBodyTransform anchor = new RigidBodyTransform();

      private YoBoolean nativeActive, ekfActive, applyEKFCorrection;
      private YoVariable validCount, rejectedCount, ekfAcceptedCount, ekfRejectedCount;
      private YoVariable[] kioPosition, kioOrientation, ekfOutput;
      private boolean initialized;
      private long startTime = Long.MIN_VALUE;
      private int nextEventIndex;
      private long lastRobotStateTime = Long.MIN_VALUE;
      private long previousTime = Long.MIN_VALUE;
      private RobotConfigurationData robotState;

      EstimatorLoop(AvatarEstimatorThread estimatorThread)
      {
         this.estimatorThread = estimatorThread;
         estimatorModel = estimatorThread.getFullRobotModel();
         minimumLatency = Math.round(robotModel.getEstimatorDT() * 1.0e9);
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         // Added while the simulation runs, so its variables couldn't be logged anyway.
         return null;
      }

      @Override
      public void doControl()
      {
         long now = estimatorThread.getSensorReader().getProcessedSensorOutputMap().getWallTime();
         if (!initialized)
            initialize(now);
         applyRequests(now);
         providerClock.set(now);
         recordEstimates(now);

         if (provider != null)
            takeRestartRequest();
         TruthSample sample;
         while ((sample = truthSamples.peek()) != null && sample.exposureTime() <= now)
         {
            truthSamples.poll();
            applyEventsUntil(sample.exposureTime());
            track(sample);
         }
         applyEventsUntil(now);
         while (!framesInFlight.isEmpty() && framesInFlight.peekFirst().deliveryTime() <= now)
            deliver(framesInFlight.pollFirst());
         if (provider != null && (lastRobotStateTime == Long.MIN_VALUE || now - lastRobotStateTime >= ROBOT_STATE_PERIOD))
            sendRobotState(now);
         statistics.updateCounts(validCount, rejectedCount, ekfAcceptedCount, ekfRejectedCount);
      }

      private void initialize(long now)
      {
         YoRegistry registry = estimatorThread.getYoRegistry();
         nativeActive = (YoBoolean) find(registry, "nativePelvisPoseCorrectionActive");
         ekfActive = (YoBoolean) find(registry, "ekfPelvisPoseCorrectionActive");
         applyEKFCorrection = (YoBoolean) find(registry, "applyEKFPelvisPoseCorrection");
         validCount = find(registry, "validExternalPelvisPoseCount");
         rejectedCount = find(registry, "rejectedExternalPelvisPoseCount");
         ekfAcceptedCount = find(registry, "ekfAcceptedMeasurementCount");
         ekfRejectedCount = find(registry, "ekfRejectedMeasurementCount");
         kioPosition = findAll(registry, "estimatedRootJointPosition", "X", "Y", "Z");
         kioOrientation = findAll(registry, "estimatedRootJoint", "Qx", "Qy", "Qz", "Qs");
         ekfOutput = findAll(registry, "ekfCorrectedPelvisPose", "X", "Y", "Z", "Yaw", "Pitch", "Roll");
         if (provider != null)
         {
            robotState = RobotConfigurationDataFactory.create(estimatorModel.getOneDoFJoints(),
                                                              estimatorModel.getForceSensorDefinitions(),
                                                              estimatorModel.getIMUDefinitions());
         }
         initialized = true;
      }

      private void applyRequests(long now)
      {
         PelvisPoseCorrectionMode mode = requestedMode;
         if (mode != null)
         {
            requestedMode = null;
            nativeActive.set(mode.usesNativeCorrection());
            ekfActive.set(mode.usesEKFCorrection());
         }
         Boolean apply = requestedApplyCorrection;
         if (apply != null)
         {
            requestedApplyCorrection = null;
            applyEKFCorrection.set(apply);
         }
         if (startRequested)
         {
            startRequested = false;
            startTime = now;
            nextEventIndex = 0;
            statistics.reset(validCount, rejectedCount, ekfAcceptedCount, ekfRejectedCount);
         }
         if (statisticsResetRequested)
         {
            statisticsResetRequested = false;
            statistics.reset(validCount, rejectedCount, ekfAcceptedCount, ekfRejectedCount);
         }
         if (restartRequested)
         {
            restartRequested = false;
            restart();
         }
      }

      /** Records the estimator's pelvis pose, its KIO and the EKF's output this tick, and how fast the applied correction changes. */
      private void recordEstimates(long now)
      {
         estimatorModel.getRootJoint().getJointPose().get(estimatedPelvisPose);
         orientation.set(kioOrientation[0].getValueAsDouble(),
                         kioOrientation[1].getValueAsDouble(),
                         kioOrientation[2].getValueAsDouble(),
                         kioOrientation[3].getValueAsDouble());
         kioPelvisPose.getRotation().set(orientation);
         kioPelvisPose.getTranslation().set(kioPosition[0].getValueAsDouble(), kioPosition[1].getValueAsDouble(), kioPosition[2].getValueAsDouble());
         ekfOutputPose.getRotation().setYawPitchRoll(ekfOutput[3].getValueAsDouble(), ekfOutput[4].getValueAsDouble(), ekfOutput[5].getValueAsDouble());
         ekfOutputPose.getTranslation().set(ekfOutput[0].getValueAsDouble(), ekfOutput[1].getValueAsDouble(), ekfOutput[2].getValueAsDouble());
         estimatedPelvisPoseHistory.add(now, estimatedPelvisPose);
         kioPelvisPoseHistory.add(now, kioPelvisPose);
         if (!ekfOutputPose.containsNaN())
            ekfOutputPoseHistory.add(now, ekfOutputPose);

         // The applied correction: the estimator's pelvis pose relative to its KIO, translation in world.
         correction.getRotation().set(estimatedPelvisPose.getRotation());
         correction.getRotation().appendInvertOther(kioPelvisPose.getRotation());
         correction.getTranslation().sub(estimatedPelvisPose.getTranslation(), kioPelvisPose.getTranslation());
         if (previousTime != Long.MIN_VALUE && now > previousTime)
            statistics.addCorrection(previousCorrection, correction, (now - previousTime) * 1.0e-9);
         previousCorrection.set(correction);
         previousTime = now;
      }

      private void applyEventsUntil(long time)
      {
         if (startTime == Long.MIN_VALUE)
            return;
         while (nextEventIndex < scheduledEvents.size())
         {
            Event event = scheduledEvents.get(nextEventIndex);
            if (startTime + Math.round(event.time() * 1.0e9) > time)
               return;
            nextEventIndex++;
            if (event instanceof Restart)
            {
               restart();
            }
            else if (event instanceof Relocalization relocalization)
            {
               Vector3D rotationVector = new Vector3D();
               drift.getRotation().getRotationVector(rotationVector);
               rotationVector.scale(1.0 - relocalization.fractionOfDriftRemoved());
               drift.getRotation().setRotationVector(rotationVector);
               drift.getTranslation().scale(1.0 - relocalization.fractionOfDriftRemoved());
            }
            else if (event instanceof Jump jump)
            {
               drift.preMultiply(jump.jump());
            }
            // Tracking loss is checked per frame.
         }
      }

      private boolean isTrackingLost(long exposureTime)
      {
         if (startTime == Long.MIN_VALUE)
            return false;
         for (Event event : scheduledEvents)
         {
            if (event instanceof TrackingLoss loss && exposureTime >= startTime + Math.round(loss.time() * 1.0e9)
                && exposureTime < startTime + Math.round(loss.endTime() * 1.0e9))
               return true;
         }
         return false;
      }

      /** A new VIO session: a new world at the next frame, without the old drift. The provider's session starts on the camera's restart. */
      private void restart()
      {
         if (provider != null)
         {
            provider.requestTrackingRestart();
            return;
         }
         startNewSession();
      }

      private void startNewSession()
      {
         session++;
         sessionHasWorld = false;
         anchored = false;
         drift.setToZero();
         statistics.sessions++;
      }

      /** The camera's grab thread's part of a restart: the provider asks, the camera restarts tracking, and reports it done. */
      private void takeRestartRequest()
      {
         long token = provider.takePendingTrackingRestartRequest();
         if (token == 0)
            return;
         startNewSession();
         provider.finishTrackingRestart(token, true);
      }

      /** What the camera's tracking reports for a frame, in its session's VIO world, with the error model. */
      private void track(TruthSample sample)
      {
         recentTruthSamples.addLast(sample);
         while (sample.exposureTime() - recentTruthSamples.peekFirst().exposureTime() > TRUTH_SAMPLE_RETENTION)
            recentTruthSamples.pollFirst();
         statistics.addTruthSample(sample, estimatedPelvisPoseHistory, kioPelvisPoseHistory, ekfOutputPoseHistory);
         statistics.framesExposed++;

         if (!sessionHasWorld)
         {
            // VIO's world is gravity aligned, at the tracked frame's pose when the session starts.
            vioWorld.setToZero();
            vioWorld.getRotation().setToYawOrientation(sample.trackedFramePose().getRotation().getYaw());
            vioWorld.getTranslation().set(sample.trackedFramePose().getTranslation());
            if (errorModel.randomizeSessionOrigin())
            {
               RigidBodyTransform origin = new RigidBodyTransform();
               origin.getRotation().setToYawOrientation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
               origin.getTranslation().set(EuclidCoreRandomTools.nextDouble(random, 3.0), EuclidCoreRandomTools.nextDouble(random, 3.0), 0.0);
               vioWorld.multiply(origin);
            }
            sessionHasWorld = true;
            lastTrackedExposureTime = Long.MIN_VALUE;
         }

         // VIO's own drift keeps growing whether or not this frame is tracked.
         if (lastTrackedExposureTime != Long.MIN_VALUE)
         {
            double sqrtDt = Math.sqrt((sample.exposureTime() - lastTrackedExposureTime) * 1.0e-9);
            drift.getRotation().prependYawRotation(errorModel.yawRandomWalk() * sqrtDt * random.nextGaussian());
            drift.getTranslation().add(errorModel.positionRandomWalk() * sqrtDt * random.nextGaussian(),
                                       errorModel.positionRandomWalk() * sqrtDt * random.nextGaussian(),
                                       errorModel.positionRandomWalk() * sqrtDt * random.nextGaussian());
         }
         lastTrackedExposureTime = sample.exposureTime();

         if (random.nextDouble() < errorModel.dropoutProbability())
            return;

         RigidBodyTransform pose = null;
         if (!isTrackingLost(sample.exposureTime()))
         {
            pose = new RigidBodyTransform(vioWorld);
            pose.invert();
            pose.multiply(sample.trackedFramePose());
            pose.getTranslation().scale(1.0 + errorModel.scaleError());
            pose.preMultiply(drift);

            RigidBodyTransform noise = new RigidBodyTransform();
            noise.getRotation().setRotationVector(gaussian(errorModel.orientationNoise()), gaussian(errorModel.orientationNoise()), gaussian(errorModel.orientationNoise()));
            noise.getTranslation().set(gaussian(errorModel.positionNoise()), gaussian(errorModel.positionNoise()), gaussian(errorModel.positionNoise()));
            pose.multiply(noise);

            if (!errorModel.outlierMagnitudes().isEmpty() && random.nextDouble() < errorModel.outlierProbability())
            {
               double magnitude = errorModel.outlierMagnitudes().get(random.nextInt(errorModel.outlierMagnitudes().size()));
               double direction = EuclidCoreRandomTools.nextDouble(random, Math.PI);
               pose.getTranslation().add(magnitude * Math.cos(direction), magnitude * Math.sin(direction), 0.0);
               statistics.outliers++;
            }
         }
         else if (delivery == Delivery.PELVIS_DIRECT)
         {
            return;
         }

         long latency = Math.max(minimumLatency, Math.round((errorModel.meanLatency() + errorModel.latencyJitter() * random.nextGaussian()) * 1.0e9));
         long deliveryTime = sample.exposureTime() + latency;
         if (!framesInFlight.isEmpty())
            deliveryTime = Math.max(deliveryTime, framesInFlight.peekLast().deliveryTime());
         framesInFlight.addLast(new Frame(sample.exposureTime(), deliveryTime, session, pose, sample.pelvisPose()));
      }

      private double gaussian(double standardDeviation)
      {
         return standardDeviation * random.nextGaussian();
      }

      private void deliver(Frame frame)
      {
         if (frame.session() != session)
            return; // Tracked before the restart.
         long timestamp = frame.exposureTime() + clockOffset;
         if (provider != null)
         {
            provider.addCameraTrackingPose(timestamp, frame.poseInVIOWorld(), sensorOffset);
            return;
         }

         if (!anchored)
         {
            // As the provider does: the session's first pose is put on the estimator's pelvis pose at its exposure.
            if (!estimatedPelvisPoseHistory.getPoseAtTime(frame.exposureTime(), anchor))
               return;
            anchor.multiplyInvertOther(frame.poseInVIOWorld());
            anchored = true;
         }
         RigidBodyTransform pelvisPose = new RigidBodyTransform(anchor);
         pelvisPose.multiply(frame.poseInVIOWorld());

         StampedPosePacket packet = new StampedPosePacket();
         packet.setTimestamp(timestamp);
         packet.setConfidenceFactor(1.0);
         packet.getFrameId().append("simulatedVIOSession" + session);
         packet.getPose().set(pelvisPose);
         communicator.receivedPacket(packet);
         statistics.addVIOPelvisPose(pelvisPose, frame.truePelvisPose());
      }

      /** On the estimator thread, when the provider converts a camera pose. */
      void receivedProviderPelvisPose(StampedPosePacket packet)
      {
         if (communicator != null)
            communicator.receivedPacket(packet);
         long exposureTime = packet.getTimestamp() - clockOffset;
         for (TruthSample sample : recentTruthSamples)
         {
            if (sample.exposureTime() == exposureTime)
            {
               RigidBodyTransform pelvisPose = new RigidBodyTransform(packet.getPose().getPose().getOrientation(), packet.getPose().getPose().getPosition());
               statistics.addVIOPelvisPose(pelvisPose, sample.pelvisPose());
               return;
            }
         }
      }

      /** The controller's robot state, as the robot publishes it, to the provider. */
      private void sendRobotState(long now)
      {
         lastRobotStateTime = now;
         RobotConfigurationDataFactory.packJointState(robotState, estimatorModel.getOneDoFJoints());
         robotState.getRootPosition().set(estimatorModel.getRootJoint().getJointPose().getPosition());
         robotState.getRootOrientation().set(estimatorModel.getRootJoint().getJointPose().getOrientation());
         robotState.setWallTime(now);
         robotState.setMonotonicTime(now);
         provider.addRobotState(robotState);
      }
   }

   private static YoVariable find(YoRegistry registry, String name)
   {
      YoVariable variable = registry.findVariable(name);
      if (variable == null)
         throw new IllegalStateException("The estimator has no " + name + " variable");
      return variable;
   }

   private static YoVariable[] findAll(YoRegistry registry, String prefix, String... suffixes)
   {
      YoVariable[] variables = new YoVariable[suffixes.length];
      for (int i = 0; i < suffixes.length; i++)
         variables[i] = find(registry, prefix + suffixes[i]);
      return variables;
   }

   /**
    * How far the estimator is from ground truth since {@link #start()} or {@link #resetStatistics()}, measured at each camera
    * exposure, and what the VIO and the estimator's checks did. Position errors are 3D distances in m; yaw errors are absolute,
    * in rad.
    * <ul>
    * <li>KIO: the estimator's kinematic-inertial odometry, which the corrections never change.</li>
    * <li>Estimated: the estimator's pelvis pose, which the controller uses.</li>
    * <li>EKF output: the EKF correction's corrected pose, whether or not it's applied.</li>
    * <li>VIO: the pelvis poses VIO sends.</li>
    * </ul>
    */
   public static class Statistics
   {
      private final ErrorStatistic kioPosition = new ErrorStatistic(), kioYaw = new ErrorStatistic();
      private final ErrorStatistic estimatedPosition = new ErrorStatistic(), estimatedYaw = new ErrorStatistic();
      private final ErrorStatistic ekfOutputPosition = new ErrorStatistic(), ekfOutputYaw = new ErrorStatistic();
      private final ErrorStatistic vioPosition = new ErrorStatistic(), vioYaw = new ErrorStatistic();
      private double maximumAppliedCorrectionTranslation, maximumAppliedCorrectionRotation;
      private double maximumCorrectionTranslationRate, maximumCorrectionRotationRate;
      private int framesExposed, vioPoses, outliers, sessions;
      private long validAtReset, rejectedAtReset, ekfAcceptedAtReset, ekfRejectedAtReset;
      private long valid, rejected, ekfAccepted, ekfRejected;
      private final RigidBodyTransform truthPose = new RigidBodyTransform(), estimate = new RigidBodyTransform();
      private final Vector3D rotationVector = new Vector3D();

      private synchronized void reset(YoVariable validCount, YoVariable rejectedCount, YoVariable ekfAcceptedCount, YoVariable ekfRejectedCount)
      {
         for (ErrorStatistic statistic : List.of(kioPosition, kioYaw, estimatedPosition, estimatedYaw, ekfOutputPosition, ekfOutputYaw, vioPosition, vioYaw))
            statistic.reset();
         maximumAppliedCorrectionTranslation = maximumAppliedCorrectionRotation = 0.0;
         maximumCorrectionTranslationRate = maximumCorrectionRotationRate = 0.0;
         framesExposed = vioPoses = outliers = sessions = 0;
         validAtReset = (long) validCount.getValueAsDouble();
         rejectedAtReset = (long) rejectedCount.getValueAsDouble();
         ekfAcceptedAtReset = (long) ekfAcceptedCount.getValueAsDouble();
         ekfRejectedAtReset = (long) ekfRejectedCount.getValueAsDouble();
         valid = rejected = ekfAccepted = ekfRejected = 0;
      }

      private synchronized void updateCounts(YoVariable validCount, YoVariable rejectedCount, YoVariable ekfAcceptedCount, YoVariable ekfRejectedCount)
      {
         valid = (long) validCount.getValueAsDouble() - validAtReset;
         rejected = (long) rejectedCount.getValueAsDouble() - rejectedAtReset;
         ekfAccepted = (long) ekfAcceptedCount.getValueAsDouble() - ekfAcceptedAtReset;
         ekfRejected = (long) ekfRejectedCount.getValueAsDouble() - ekfRejectedAtReset;
      }

      private synchronized void addTruthSample(TruthSample sample, PoseHistory estimated, PoseHistory kio, PoseHistory ekfOutput)
      {
         truthPose.set(sample.pelvisPose());
         if (kio.getPoseAtTime(sample.exposureTime(), estimate))
            addError(estimate, kioPosition, kioYaw);
         if (estimated.getPoseAtTime(sample.exposureTime(), estimate))
            addError(estimate, estimatedPosition, estimatedYaw);
         if (ekfOutput.getPoseAtTime(sample.exposureTime(), estimate))
            addError(estimate, ekfOutputPosition, ekfOutputYaw);
      }

      private synchronized void addVIOPelvisPose(RigidBodyTransformReadOnly vioPelvisPose, RigidBodyTransformReadOnly truePelvisPose)
      {
         vioPoses++;
         truthPose.set(truePelvisPose);
         addError(vioPelvisPose, vioPosition, vioYaw);
      }

      private void addError(RigidBodyTransformReadOnly pose, ErrorStatistic position, ErrorStatistic yaw)
      {
         position.add(pose.getTranslation().differenceNorm(truthPose.getTranslation()));
         yaw.add(Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(pose.getRotation().getYaw(), truthPose.getRotation().getYaw())));
      }

      private synchronized void addCorrection(RigidBodyTransformReadOnly previousCorrection, RigidBodyTransformReadOnly correction, double dt)
      {
         maximumAppliedCorrectionTranslation = Math.max(maximumAppliedCorrectionTranslation, correction.getTranslation().norm());
         correction.getRotation().getRotationVector(rotationVector);
         maximumAppliedCorrectionRotation = Math.max(maximumAppliedCorrectionRotation, rotationVector.norm());
         maximumCorrectionTranslationRate = Math.max(maximumCorrectionTranslationRate,
                                                     correction.getTranslation().differenceNorm(previousCorrection.getTranslation()) / dt);
         RigidBodyTransform change = new RigidBodyTransform(correction);
         change.getRotation().appendInvertOther(previousCorrection.getRotation());
         change.getRotation().getRotationVector(rotationVector);
         maximumCorrectionRotationRate = Math.max(maximumCorrectionRotationRate, rotationVector.norm() / dt);
      }

      public synchronized double getMaximumKIOPositionError()
      {
         return kioPosition.maximum;
      }

      public synchronized double getMaximumKIOYawError()
      {
         return kioYaw.maximum;
      }

      public synchronized double getFinalKIOPositionError()
      {
         return kioPosition.last;
      }

      public synchronized double getMaximumEstimatedPositionError()
      {
         return estimatedPosition.maximum;
      }

      public synchronized double getFinalEstimatedPositionError()
      {
         return estimatedPosition.last;
      }

      public synchronized double getMaximumEstimatedYawError()
      {
         return estimatedYaw.maximum;
      }

      public synchronized double getFinalEstimatedYawError()
      {
         return estimatedYaw.last;
      }

      public synchronized double getMaximumEKFOutputPositionError()
      {
         return ekfOutputPosition.maximum;
      }

      public synchronized double getFinalEKFOutputPositionError()
      {
         return ekfOutputPosition.last;
      }

      public synchronized double getMaximumEKFOutputYawError()
      {
         return ekfOutputYaw.maximum;
      }

      public synchronized double getMaximumVIOPositionError()
      {
         return vioPosition.maximum;
      }

      public synchronized double getMaximumVIOYawError()
      {
         return vioYaw.maximum;
      }

      /** The largest the applied correction got, the estimator's pelvis pose relative to its KIO: translation in m. */
      public synchronized double getMaximumAppliedCorrectionTranslation()
      {
         return maximumAppliedCorrectionTranslation;
      }

      /** The largest the applied correction got, the estimator's pelvis pose relative to its KIO: rotation in rad. */
      public synchronized double getMaximumAppliedCorrectionRotation()
      {
         return maximumAppliedCorrectionRotation;
      }

      /** How fast the applied correction changed at most, in m/s, measured every estimator tick. */
      public synchronized double getMaximumCorrectionTranslationRate()
      {
         return maximumCorrectionTranslationRate;
      }

      /** How fast the applied correction changed at most, in rad/s, measured every estimator tick. */
      public synchronized double getMaximumCorrectionRotationRate()
      {
         return maximumCorrectionRotationRate;
      }

      public synchronized int getFramesExposed()
      {
         return framesExposed;
      }

      /** Pelvis poses VIO sent to the estimator. */
      public synchronized int getVIOPoses()
      {
         return vioPoses;
      }

      public synchronized int getOutliers()
      {
         return outliers;
      }

      /** VIO sessions started. */
      public synchronized int getSessions()
      {
         return sessions;
      }

      /** Poses the estimator's validator accepted. */
      public synchronized long getValidPoses()
      {
         return valid;
      }

      /** Poses the estimator's validator rejected. */
      public synchronized long getRejectedPoses()
      {
         return rejected;
      }

      public synchronized long getEKFAcceptedMeasurements()
      {
         return ekfAccepted;
      }

      public synchronized long getEKFRejectedMeasurements()
      {
         return ekfRejected;
      }

      @Override
      public synchronized String toString()
      {
         return ("position error max/final [m]: KIO %.3f/%.3f, estimated %.3f/%.3f, EKF output %.3f/%.3f, VIO %.3f (rms %.3f)%n"
                 + "yaw error max/final [deg]: KIO %.2f/%.2f, estimated %.2f/%.2f, EKF output %.2f/%.2f, VIO %.2f%n"
                 + "applied correction max %.3f m, %.2f deg; changing at most %.3f m/s, %.3f rad/s%n"
                 + "frames %d, VIO poses %d, outliers %d, sessions %d; validator %d valid, %d rejected; EKF %d accepted, %d rejected").formatted(
               kioPosition.maximum, kioPosition.last, estimatedPosition.maximum, estimatedPosition.last, ekfOutputPosition.maximum,
               ekfOutputPosition.last, vioPosition.maximum, vioPosition.rms(),
               Math.toDegrees(kioYaw.maximum), Math.toDegrees(kioYaw.last), Math.toDegrees(estimatedYaw.maximum), Math.toDegrees(estimatedYaw.last),
               Math.toDegrees(ekfOutputYaw.maximum), Math.toDegrees(ekfOutputYaw.last), Math.toDegrees(vioYaw.maximum),
               maximumAppliedCorrectionTranslation, Math.toDegrees(maximumAppliedCorrectionRotation), maximumCorrectionTranslationRate,
               maximumCorrectionRotationRate,
               framesExposed, vioPoses, outliers, sessions, valid, rejected, ekfAccepted, ekfRejected);
      }
   }

   private static class ErrorStatistic
   {
      private double maximum, last, sumOfSquares;
      private int count;

      void reset()
      {
         maximum = last = sumOfSquares = 0.0;
         count = 0;
      }

      void add(double error)
      {
         maximum = Math.max(maximum, error);
         last = error;
         sumOfSquares += error * error;
         count++;
      }

      double rms()
      {
         return count == 0 ? 0.0 : Math.sqrt(sumOfSquares / count);
      }
   }
}
