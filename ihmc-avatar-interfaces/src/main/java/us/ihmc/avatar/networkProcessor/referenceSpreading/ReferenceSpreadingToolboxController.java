package us.ihmc.avatar.networkProcessor.referenceSpreading;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import toolbox_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.ContactParticleFilter;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.ForceEstimatorDynamicMatrixUpdater;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.PredefinedContactExternalForceSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI.ExternalForceEstimationToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.ToIntFunction;

public class ReferenceSpreadingToolboxController extends ToolboxController
{
   private final YoBoolean waitingForRobotConfigurationData = new YoBoolean("waitingForRobotConfigurationData", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   public final ReplayHandHybridTrajectoryRS hybridReplayer;

   private final HumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();


   private final CommandInputManager commandInputManager;
   private final ExternalForceEstimationOutputStatus outputStatus = new ExternalForceEstimationOutputStatus();

   public ReferenceSpreadingToolboxController(DRCRobotModel robotModel,
                                                   FullHumanoidRobotModel fullRobotModel,
                                                   CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   YoGraphicsListRegistry graphicsListRegistry,
                                                   int updateRateMillis,
                                                   YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);
      this.rootJoint = fullRobotModel.getRootJoint();

      MultiBodySystemTools.getRootBody(fullRobotModel.getElevator())
                          .subtreeIterable()
                          .forEach(rigidBody -> rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody));

      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);

      double updateDT = Conversions.millisecondsToSeconds(updateRateMillis);
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(robotModel.getControllerDT(),
                                                                                       9.81,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       joints,
                                                                                       referenceFrames.getCenterOfMassFrame(),
                                                                                       robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings(),
                                                                                       graphicsListRegistry,
                                                                                       parentRegistry);

      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }
      controlCoreToolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
      List<Collidable> collidables = collisionModel.getRobotCollidables(fullRobotModel.getRootBody());

      String demoDirectory = Objects.requireNonNull(new WorkspaceDirectory("nadia",
                                                                           "nadia-hardware-drivers/src/test/resources/hybridPlaybackCSVs").getFilesystemDirectory()).toString();
      String filePath = demoDirectory + "/simplePlayback.csv";

      hybridReplayer = new ReplayHandHybridTrajectoryRS(filePath, 1);
   }

   @Override
   public boolean initialize()
   {
      waitingForRobotConfigurationData.set(true);
      hybridReplayer.start();
      isDone.set(false);

      return true;
   }

   @Override
   public void updateInternal()
   {
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if (robotConfigurationData != null)
      {
         referenceFrames.updateFrames();
         waitingForRobotConfigurationData.set(false);
      }

      if (waitingForRobotConfigurationData.getBooleanValue())
         return;

      hybridReplayer.doAction(robotConfigurationData.getMonotonicTime());
   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      this.robotConfigurationData.set(robotConfigurationData);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }
}
