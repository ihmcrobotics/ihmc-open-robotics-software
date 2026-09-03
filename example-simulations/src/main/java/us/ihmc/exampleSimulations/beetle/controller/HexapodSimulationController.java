package us.ihmc.exampleSimulations.beetle.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.exampleSimulations.beetle.footContact.SimulatedPlaneContactStateUpdater;
import us.ihmc.exampleSimulations.beetle.parameters.HexapodControllerParameters;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleJointNameMapAndContactDefinition;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetlePhysicalProperties;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointReadOnly;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SCS2SensorReader;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class HexapodSimulationController implements Controller
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = -9.81;
   private final double controllerDt;
   private final int controlTicksPerSimulationTick;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoBoolean useInverseDynamics = new YoBoolean("useInverseDynamics", registry);

   private final SimControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final SCS2SensorReader sensorReader;
   private final JointDesiredOutputList lowLevelControllerCoreOutput;
   private final SegmentDependentList<RobotSextant, RigidBodyBasics> footRigidBodies = new SegmentDependentList<>(RobotSextant.class);
   private final SegmentDependentList<RobotSextant, SimulatedPlaneContactStateUpdater> contactStateUpdaters = new SegmentDependentList<>(RobotSextant.class);

   private final FullRobotModel fullRobotModel;
   private final WholeBodyControllerCore controllerCore;
   private final HexapodHighLevelControlManager highLevelController;

   private final HexapodReferenceFrames referenceFrames;
   private YoLong totalTimeToCompleteTick = new YoLong("totalTimeToCompleteTick", registry);
   private YoDouble totalTimeToCompleteTickInSeconds = new YoDouble("totalTimeToCompleteTickInSeconds", registry);

   private final ArrayList<YoGraphicReferenceFrame> referenceFrameGraphics = new ArrayList<>();
   private ContactPointVisualizer contactPointVisualizer;
   private final YoFramePoint2D bodyPosition2D = new YoFramePoint2D("bodyPosition", worldFrame, registry);

   public HexapodSimulationController(ControllerInput controllerInput,
                                       ControllerOutput controllerOutput,
                                       FullRobotModel fullRobotModel,
                                       ArrayList<String> jointsToControl,
                                       HexapodControllerParameters idParameters,
                                       HexapodControllerParameters vmcParameters,
                                       YoGraphicsListRegistry yoGraphicsListRegistry,
                                       double controllerDt,
                                       int controlTicksPerSimulationTick)
   {
      this.controllerDt = controllerDt;
      this.controlTicksPerSimulationTick = controlTicksPerSimulationTick;
      this.controllerInput = (SimControllerInput) controllerInput;
      this.controllerOutput = controllerOutput;
      this.fullRobotModel = fullRobotModel;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.sensorReader = SCS2SensorReader.newPerfectSensorReader(this.controllerInput, fullRobotModel.getRootJoint(), null);
      this.referenceFrames = new HexapodReferenceFrames(fullRobotModel, RhinoBeetlePhysicalProperties.getOffsetsFromJointBeforeFootToSoleAlignedWithWorld());
      setupPlaneContactStateUpdaters(fullRobotModel);

      lowLevelControllerCoreOutput = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());

      highLevelController = new HexapodHighLevelControlManager(fullRobotModel, referenceFrames, contactStateUpdaters, jointsToControl, idParameters, vmcParameters, yoGraphicsListRegistry, controllerDt, registry);

      FeedbackControlCommandList feedbackControlCommandList = createFeedbackControlTemplate();
      WholeBodyControlCoreToolbox toolbox = makeControllerToolbox();
      this.controllerCore = new WholeBodyControllerCore(toolbox, new FeedbackControllerTemplate(feedbackControlCommandList), lowLevelControllerCoreOutput, registry);

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         YoGraphicReferenceFrame frame = new YoGraphicReferenceFrame(joint.getFrameBeforeJoint(), registry, true, 0.1);
         referenceFrameGraphics.add(frame);
         yoGraphicsListRegistry.registerYoGraphic("frames", frame);
      }
   }

   private void setupPlaneContactStateUpdaters(FullRobotModel fullRobotModel)
   {
      ArrayList<SimulatedPlaneContactStateUpdater> contactStateUpdatersList = new ArrayList<>();
      RhinoBeetleJointNameMapAndContactDefinition jointMap = new RhinoBeetleJointNameMapAndContactDefinition();
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         RigidBodyBasics endEffector = fullRobotModel.getEndEffector(robotSextant);
         footRigidBodies.set(robotSextant, endEffector);

         String jointNameBeforeFoot = jointMap.getJointNameBeforeFoot(robotSextant);
         ReferenceFrame soleFrame = referenceFrames.getFootFrame(robotSextant);

         SimJointReadOnly simJoint = controllerInput.getInput().findJoint(jointNameBeforeFoot);
         GroundContactPoint groundContactPoint = simJoint.getAuxiliaryData().getGroundContactPoints().get(0);

         SimulatedPlaneContactStateUpdater contactStateUpdater = new SimulatedPlaneContactStateUpdater(groundContactPoint, endEffector, soleFrame);
         contactStateUpdaters.set(robotSextant, contactStateUpdater);
         contactStateUpdatersList.add(contactStateUpdater);
      }
      contactPointVisualizer = new ContactPointVisualizer(contactStateUpdatersList, yoGraphicsListRegistry, registry);
   }

   private FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
      feedbackControlCommandList.addCommand(highLevelController.createFeedbackControlTemplate());
      return feedbackControlCommandList;
   }

   private WholeBodyControlCoreToolbox makeControllerToolbox()
   {
      //point feet contact points
      ArrayList<Point2D> contactPointsInSoleFrame = new ArrayList<>();
      contactPointsInSoleFrame.add(new Point2D());

      //Rigid Bodies
      List<ContactablePlaneBody> footContactableBodies = new ArrayList<>();
      RigidBodyBasics[] controlledBodies = new RigidBodyBasics[7];

      int i = 0;
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         RigidBodyBasics endEffector = fullRobotModel.getEndEffector(robotSextant);
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         ;
         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(endEffector, footFrame, contactPointsInSoleFrame);
         footContactableBodies.add(footContactableBody);
         controlledBodies[i] = endEffector;
         i++;
      }
      controlledBodies[i] = fullRobotModel.getRootBody();

      //Joints to Control
      JointBasics[] controlledJoints = MultiBodySystemTools.collectSubtreeJoints(fullRobotModel.getElevator());

      ControllerCoreOptimizationSettings momentumOptimizationSettings = getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(() -> controllerDt, -gravity, rootJoint, controlledJoints, centerOfMassFrame,
                                                                            momentumOptimizationSettings, registry);
      toolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);
      toolbox.setupForInverseDynamicsSolver(footContactableBodies);
      toolbox.setupForInverseKinematicsSolver();
      toolbox.setupForVirtualModelControlSolver(fullRobotModel.getRootBody(), footContactableBodies);

      return toolbox;
   }

   private ControllerCoreOptimizationSettings getMomentumOptimizationSettings()
   {
      return new HexapodMomentumOptimizationSettings();
   }

   @Override
   public void initialize()
   {
      controllerCore.initialize();
      highLevelController.initialize();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   private boolean firstTick = true;
   private int simulationTickCounter = 0;

   @Override
   public void doControl()
   {
      long startTime = System.nanoTime();
      sensorReader.read(null);
      contactPointVisualizer.update(0.0);
      for (YoGraphicReferenceFrame frame : referenceFrameGraphics)
      {
         frame.update();
      }

      referenceFrames.updateFrames();
      bodyPosition2D.set(fullRobotModel.getRootJoint().getJointPose().getPosition().getX(),
                          fullRobotModel.getRootJoint().getJointPose().getPosition().getY());

      if (firstTick)
      {
         initialize();
         firstTick = false;
      }

      // The whole-body controller core is tuned for controllerDt, which is coarser than the simulation's
      // integration step (the contact spring-damper model needs the finer step to stay stable). Only
      // recompute torques every controlTicksPerSimulationTick ticks; the physics engine holds the
      // previously written torques on the ticks in between, mirroring SCS1's setController(controller, ticksPerControl).
      if (simulationTickCounter == 0)
      {
         highLevelController.doControl();
         ControllerCoreCommand controllerCoreCommandList = highLevelController.getControllerCoreCommand();
         controllerCore.compute(controllerCoreCommandList);
         writeControllerOutput();
      }
      simulationTickCounter = (simulationTickCounter + 1) % controlTicksPerSimulationTick;

      totalTimeToCompleteTick.set(System.nanoTime() - startTime);
      totalTimeToCompleteTickInSeconds.set(Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
   }

   private void writeControllerOutput()
   {
      for (int i = 0; i < lowLevelControllerCoreOutput.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly joint = lowLevelControllerCoreOutput.getOneDoFJoint(i);
         JointDesiredOutputReadOnly jointDesiredOutput = lowLevelControllerCoreOutput.getJointDesiredOutput(i);
         OneDoFJointStateBasics jointOutput = controllerOutput.getOneDoFJointOutput(joint.getName());
         jointOutput.setEffort(jointDesiredOutput.getDesiredTorque());
      }
   }

   public YoFramePoint2D getBodyPosition2D()
   {
      return bodyPosition2D;
   }

   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("bodyPosition", bodyPosition2D, 0.02, ColorDefinitions.Black(), DefaultPoint2DGraphic.CIRCLE));
      group.addChild(highLevelController.getSCS2YoGraphics());
      return group;
   }
}
