package us.ihmc.avatar.scs2;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.AvatarStepGeneratorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateBasics;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.RobotController;

public class SCS2AvatarSimulation
{
   private static final double CAMERA_PITCH_FROM_ROBOT = Math.toRadians(-15.0);
   private static final double CAMERA_YAW_FROM_ROBOT = Math.toRadians(15.0);
   private static final double CAMERA_DISTANCE_FROM_ROBOT = 6.0;

   private Robot robot;
   private SimulationConstructionSet2 simulationConstructionSet;
   private HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private YoVariableServer yoVariableServer;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   private DisposableRobotController robotController;
   private HumanoidRobotContextData masterContext;
   private AvatarEstimatorThread estimatorThread;
   private AvatarControllerThread controllerThread;
   private AvatarStepGeneratorThread stepGeneratorThread;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private FullHumanoidRobotModel controllerFullRobotModel;
   private RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private DRCRobotModel robotModel;
   private boolean showGUI;
   private boolean automaticallyStartSimulation;
   private RealtimeROS2Node realtimeROS2Node;

   private boolean systemExitOnDestroy = true;
   private boolean javaFXThreadImplicitExit = true;

   private boolean hasBeenDestroyed = false;

   public void start()
   {
      beforeSessionThreadStart();

      simulationConstructionSet.setJavaFXThreadImplicitExit(javaFXThreadImplicitExit);
      simulationConstructionSet.setVisualizerEnabled(showGUI);
      simulationConstructionSet.startSimulationThread();
      simulationConstructionSet.addVisualizerShutdownListener(this::destroy);

      if (automaticallyStartSimulation)
         simulationConstructionSet.simulate();

      afterSessionThreadStart();
   }

   public void beforeSessionThreadStart()
   {
      if (intraprocessYoVariableLogger != null)
      {
         intraprocessYoVariableLogger.start();
      }
      if (yoVariableServer != null)
      {
         yoVariableServer.start();
      }
   }

   public void afterSessionThreadStart()
   {
      if (realtimeROS2Node != null)
         realtimeROS2Node.spin();
      if (simulationConstructionSet.isVisualizerEnabled())
         simulationConstructionSet.waitUntilVisualizerFullyUp();
   }

   public void destroy()
   {
      if (hasBeenDestroyed)
         return;

      LogTools.info("Destroying simulation");
      hasBeenDestroyed = true;

      if (robotController != null)
      {
         robotController.dispose();
         robotController = null;
      }

      if (yoVariableServer != null)
      {
         yoVariableServer.close();
         yoVariableServer = null;
      }

      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }

      if (simulationConstructionSet != null)
      {
         simulationConstructionSet.shutdownSession();
         simulationConstructionSet = null;
      }

      if (systemExitOnDestroy)
      {
         // TODO Remove this when pub-sub is released with the IntraProcessDomainImpl threads setup as daemon.
         System.exit(0);
      }
   }

   public void resetRobot()
   {
      resetRobot(true);
   }

   public void resetRobot(boolean simulateAfterReset)
   {
      simulationConstructionSet.pause();
      simulationConstructionSet.stopSimulationThread();

      for (SimJointBasics joint : robot.getAllJoints())
      {
         joint.setJointConfigurationToZero();
         joint.setJointTwistToZero();
         joint.setJointAccelerationToZero();
         joint.setJointTauToZero();
      }

      robotInitialSetup.initializeRobot(robot.getRootBody());
      robot.updateFrames();
      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getRootBody().getChildrenJoints().get(0);
      RigidBodyTransform rootJointTransform = new RigidBodyTransform(rootJoint.getJointPose().getOrientation(), rootJoint.getJointPose().getPosition());

      TObjectDoubleMap<String> jointPositions = new TObjectDoubleHashMap<>();
      SubtreeStreams.fromChildren(OneDoFJointBasics.class, robot.getRootBody()).forEach(joint -> jointPositions.put(joint.getName(), joint.getQ()));
      estimatorThread.initializeStateEstimators(rootJointTransform, jointPositions);
      controllerThread.initialize();
      stepGeneratorThread.initialize();
      masterContext.set(estimatorThread.getHumanoidRobotContextData());

      simulationConstructionSet.reinitializeSimulation();
      simulationConstructionSet.startSimulationThread();

      if (simulateAfterReset)
         simulationConstructionSet.simulate();
   }

   private void initializeCamera(Orientation3DReadOnly robotOrientation, Tuple3DReadOnly robotPosition)
   {
      Point3D focusPosition = new Point3D(robotPosition);
      Point3D cameraPosition = new Point3D(10, 0, 0);
      RotationMatrixTools.applyPitchRotation(CAMERA_PITCH_FROM_ROBOT, cameraPosition, cameraPosition);
      RotationMatrixTools.applyYawRotation(CAMERA_YAW_FROM_ROBOT, cameraPosition, cameraPosition);
      RotationMatrixTools.applyYawRotation(robotOrientation.getYaw(), cameraPosition, cameraPosition);
      cameraPosition.scale(CAMERA_DISTANCE_FROM_ROBOT / cameraPosition.distanceFromOrigin());
      cameraPosition.add(focusPosition);

      setCamera(focusPosition, cameraPosition);
   }

   private void checkSimulationSessionAlive()
   {
      if (getSimulationConstructionSet().isSessionShutdown())
         throw new IllegalStateException("Simulation has been shutdown");
   }

   // GUI controls:
   public void setCameraDefaultRobotView()
   {
      checkSimulationSessionAlive();
      SixDoFJointStateBasics initialRootJointState = (SixDoFJointStateBasics) getRobotDefinition().getRootJointDefinitions().get(0).getInitialJointState();
      if (initialRootJointState != null)
         initializeCamera(initialRootJointState.getOrientation(), initialRootJointState.getPosition());
   }

   public void setCameraZoom(double distanceFromFocus)
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().setCameraZoom(distanceFromFocus);
   }

   /**
    * Sets the new focus point the camera is looking at.
    * <p>
    * The camera is rotated during this operation, its position remains unchanged.
    * </p>
    * 
    * @param focus the new focus position.
    */
   public void setCameraFocusPosition(Point3DReadOnly focus)
   {
      checkSimulationSessionAlive();
      setCameraFocusPosition(focus.getX(), focus.getY(), focus.getZ());
   }

   /**
    * Sets the new focus point the camera is looking at.
    * <p>
    * The camera is rotated during this operation, its position remains unchanged.
    * </p>
    *
    * @param x the x-coordinate of the new focus location.
    * @param y the y-coordinate of the new focus location.
    * @param z the z-coordinate of the new focus location.
    */
   public void setCameraFocusPosition(double x, double y, double z)
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().setCameraFocusPosition(x, y, z);
   }

   /**
    * Sets the new camera position.
    * <p>
    * The camera is rotated during this operation such that the focus point remains unchanged.
    * </p>
    * 
    * @param position the new camera position.
    */
   public void setCameraPosition(Point3DReadOnly position)
   {
      setCameraPosition(position.getX(), position.getY(), position.getZ());
   }

   /**
    * Sets the new camera position.
    * <p>
    * The camera is rotated during this operation such that the focus point remains unchanged.
    * </p>
    * 
    * @param x the x-coordinate of the new camera location.
    * @param y the y-coordinate of the new camera location.
    * @param z the z-coordinate of the new camera location.
    */
   public void setCameraPosition(double x, double y, double z)
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().setCameraPosition(x, y, z);
   }

   /**
    * Sets the camera configuration.
    * 
    * @param cameraFocus    the new focus position (where the camera is looking at).
    * @param cameraPosition the new camerate position.
    */
   public void setCamera(Point3DReadOnly cameraFocus, Point3DReadOnly cameraPosition)
   {
      setCameraFocusPosition(cameraFocus);
      setCameraPosition(cameraPosition);
   }

   public void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().requestCameraRigidBodyTracking(robotName, rigidBodyName);
   }

   public boolean hasBeenDestroyed()
   {
      return hasBeenDestroyed;
   }

   public SimulationConstructionSet2 getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   public void setSystemExitOnDestroy(boolean systemExitOnDestroy)
   {
      this.systemExitOnDestroy = systemExitOnDestroy;
   }

   public void setJavaFXThreadImplicitExit(boolean javaFXThreadImplicitExit)
   {
      this.javaFXThreadImplicitExit = javaFXThreadImplicitExit;
   }

   /**
    * For unit testing only
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      controllerThread.addRobotController(controller);
   }

   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return controllerThread.getFullRobotModelCorruptor();
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return highLevelHumanoidControllerFactory;
   }

   public SimulatedDRCRobotTimeProvider getSimulatedRobotTimeProvider()
   {
      return simulatedRobotTimeProvider;
   }

   public void setSimulationConstructionSet(SimulationConstructionSet2 simulationConstructionSet)
   {
      this.simulationConstructionSet = simulationConstructionSet;
   }

   public void setHighLevelHumanoidControllerFactory(HighLevelHumanoidControllerFactory momentumBasedControllerFactory)
   {
      this.highLevelHumanoidControllerFactory = momentumBasedControllerFactory;
   }

   public void setYoVariableServer(YoVariableServer yoVariableServer)
   {
      this.yoVariableServer = yoVariableServer;
   }

   public void setIntraprocessYoVariableLogger(IntraprocessYoVariableLogger intraprocessYoVariableLogger)
   {
      this.intraprocessYoVariableLogger = intraprocessYoVariableLogger;
   }

   public void setRobotController(DisposableRobotController robotController)
   {
      this.robotController = robotController;
   }

   public void setMasterContext(HumanoidRobotContextData masterContext)
   {
      this.masterContext = masterContext;
   }

   public void setEstimatorThread(AvatarEstimatorThread estimatorThread)
   {
      this.estimatorThread = estimatorThread;
   }

   public AvatarEstimatorThread getEstimatorThread()
   {
      return estimatorThread;
   }

   public void setControllerThread(AvatarControllerThread controllerThread)
   {
      this.controllerThread = controllerThread;
   }

   public AvatarControllerThread getControllerThread()
   {
      return controllerThread;
   }

   public void setStepGeneratorThread(AvatarStepGeneratorThread stepGeneratorThread)
   {
      this.stepGeneratorThread = stepGeneratorThread;
   }

   public void setSimulatedRobotTimeProvider(SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider)
   {
      this.simulatedRobotTimeProvider = simulatedRobotTimeProvider;
   }

   public void setFullHumanoidRobotModel(FullHumanoidRobotModel controllerFullRobotModel)
   {
      this.controllerFullRobotModel = controllerFullRobotModel;
   }

   public void addRobotControllerOnEstimatorThread(RobotController controller)
   {
      estimatorThread.addRobotController(controller);
   }

   public void setRobot(Robot robot)
   {
      this.robot = robot;
   }

   public Robot getRobot()
   {
      return robot;
   }

   public RobotDefinition getRobotDefinition()
   {
      return robot.getRobotDefinition();
   }

   public void setRobotInitialSetup(RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup = robotInitialSetup;
   }

   public RobotInitialSetup<HumanoidFloatingRootJointRobot> getRobotInitialSetup()
   {
      return robotInitialSetup;
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public void setShowGUI(boolean showGUI)
   {
      this.showGUI = showGUI;
   }

   public boolean getShowGUI()
   {
      return showGUI;
   }

   public void setAutomaticallyStartSimulation(boolean automaticallyStartSimulation)
   {
      this.automaticallyStartSimulation = automaticallyStartSimulation;
   }

   public void setRealTimeROS2Node(RealtimeROS2Node realtimeROS2Node)
   {
      this.realtimeROS2Node = realtimeROS2Node;
   }
}
