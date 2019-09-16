package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import java.awt.Color;
import java.util.List;

import org.junit.jupiter.api.AfterEach;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.HumanoidRobotKinematicsCollisionModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidable;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class KinematicsStreamingToolboxControllerTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   protected static final boolean visualize = simulationTestingParameters.getCreateGUI();

   protected CommandInputManager commandInputManager;
   protected YoVariableRegistry mainRegistry;
   protected YoGraphicsListRegistry yoGraphicsListRegistry;
   protected FullHumanoidRobotModel desiredFullRobotModel;
   protected KinematicsStreamingToolboxController toolboxController;

   protected SimulationConstructionSet scs;
   protected HumanoidFloatingRootJointRobot robot, ghost;

   /**
    * Returns a <b>new</b> instance of the robot model that will be modified in this test to create
    * ghost robots.
    */
   public abstract DRCRobotModel newRobotModel();

   public void setup(HumanoidRobotKinematicsCollisionModel collisionModel)
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DRCRobotModel robotModel = newRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      desiredFullRobotModel = robotModel.createFullRobotModel();

      mainRegistry = new YoVariableRegistry("main");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsStreamingToolboxCommandConverter(desiredFullRobotModel));
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());

      double walkingControllerPeriod = robotModel.getControllerDT();
      double toolboxControllerPeriod = 5.0e-3;
      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   walkingControllerPeriod,
                                                                   toolboxControllerPeriod,
                                                                   yoGraphicsListRegistry,
                                                                   mainRegistry);
      toolboxController.setCollisionModel(collisionModel);

      RobotDescription robotDescription = robotModel.getRobotDescription();
      robot = new HumanoidFloatingRootJointRobot(robotDescription, robotModel.getJointMap());

      if (collisionModel != null)
      {
         List<KinematicsCollidable> collidables = collisionModel.getRobotCollidables(desiredFullRobotModel);

         for (KinematicsCollidable collidable : collidables)
         {
            Graphics3DObject graphics = getGraphics(collidable);
            robot.getLink(collidable.getRigidBody().getName()).getLinkGraphics().combine(graphics);
         }
      }

      robot.setDynamic(false);
      robot.setGravity(0);

      ghost = createSCSRobot(newRobotModel(), "ghost", ghostApperance);
      hideRobot(ghost);
      ghost.setDynamic(false);
      ghost.setGravity(0);

      if (visualize)
      {
         Robot[] robots = ghost != null ? new Robot[] {robot, ghost} : new Robot[] {robot};
         scs = new SimulationConstructionSet(robots, simulationTestingParameters);
         scs.addYoVariableRegistry(mainRegistry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.startOnAThread();
      }
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }

      desiredFullRobotModel = null;
      mainRegistry = null;
      yoGraphicsListRegistry = null;
      commandInputManager = null;
      toolboxController = null;
      robot = null;
      ghost = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   protected static HumanoidFloatingRootJointRobot createSCSRobot(DRCRobotModel ghostRobotModel, String robotName, AppearanceDefinition robotAppearance)
   {
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName(robotName);
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), robotAppearance);
      HumanoidFloatingRootJointRobot scsRobot = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      scsRobot.getRootJoint().setPinned(true);
      scsRobot.setDynamic(false);
      scsRobot.setGravity(0);
      return scsRobot;
   }

   protected static void hideRobot(HumanoidFloatingRootJointRobot robot)
   {
      robot.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   protected static void snapSCSRobotToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel, HumanoidFloatingRootJointRobot robotToSnap)
   {
      JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robotToSnap, fullHumanoidRobotModel);
      jointAnglesWriter.setWriteJointVelocities(false);
      jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
   }

   public static void copyFullRobotModelState(FullHumanoidRobotModel source, FullHumanoidRobotModel destination)
   {
      for (JointStateType stateSelection : JointStateType.values())
         MultiBodySystemTools.copyJointsState(source.getRootJoint().subtreeList(), destination.getRootJoint().subtreeList(), stateSelection);
   }

   public static Graphics3DObject getGraphics(KinematicsCollidable collidable)
   {
      Shape3DReadOnly shape = collidable.getShape();
      RigidBodyTransform transformToParentJoint = collidable.getShapeFrame()
                                                            .getTransformToDesiredFrame(collidable.getRigidBody().getParentJoint().getFrameAfterJoint());
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.transform(transformToParentJoint);
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.5);

      if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         graphics.translate(sphere.getPosition());
         graphics.addSphere(sphere.getRadius(), appearance);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         RigidBodyTransform transform = new RigidBodyTransform();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), transform.getRotation());
         transform.setTranslation(capsule.getPosition());
         graphics.transform(transform);
         graphics.addCapsule(capsule.getRadius(),
                             capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
                             appearance);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }
}