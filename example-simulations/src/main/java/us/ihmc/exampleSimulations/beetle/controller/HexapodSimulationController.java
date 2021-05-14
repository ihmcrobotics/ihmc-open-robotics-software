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
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class HexapodSimulationController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = -9.81;
   private final double controllerDt;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoBoolean useInverseDynamics = new YoBoolean("useInverseDynamics", registry);

   private final SDFPerfectSimulatedSensorReader sensorReader;
   private final PerfectSimulatedOutputWriter outputWriter;
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

   public HexapodSimulationController(FullRobotModel fullRobotModel, FloatingRootJointRobot sdfRobot, ArrayList<String> jointsToControl, HexapodControllerParameters idParameters, HexapodControllerParameters vmcParameters, YoGraphicsListRegistry yoGraphicsListRegistry, double controllerDt)
   {
      this.controllerDt = controllerDt;
      this.fullRobotModel = fullRobotModel;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.sensorReader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, null);
      this.referenceFrames = new HexapodReferenceFrames(fullRobotModel, RhinoBeetlePhysicalProperties.getOffsetsFromJointBeforeFootToSoleAlignedWithWorld());
      setupPlaneContactStateUpdaters(fullRobotModel, sdfRobot);

      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      this.outputWriter = new PerfectSimulatedOutputWriter(sdfRobot, fullRobotModel, lowLevelControllerCoreOutput);
      
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

   private void setupPlaneContactStateUpdaters(FullRobotModel fullRobotModel, FloatingRootJointRobot sdfRobot)
   {
      List<GroundContactPoint> groundContactPoints = sdfRobot.getAllGroundContactPoints();
      ArrayList<SimulatedPlaneContactStateUpdater> contactStateUpdatersList = new ArrayList<>();
      RhinoBeetleJointNameMapAndContactDefinition jointMap = new RhinoBeetleJointNameMapAndContactDefinition();
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         RigidBodyBasics endEffector = fullRobotModel.getEndEffector(robotSextant);
         footRigidBodies.set(robotSextant, endEffector);

         String jointNameBeforeFoot = jointMap.getJointNameBeforeFoot(robotSextant);
         OneDoFJointBasics oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointNameBeforeFoot);
         ReferenceFrame soleFrame = referenceFrames.getFootFrame(robotSextant);
         for (GroundContactPoint groundContactPoint : groundContactPoints)
         {
            if (groundContactPoint.getParentJoint().getName().equals(oneDoFJoint.getName()))
            {

               SimulatedPlaneContactStateUpdater contactStateUpdater = new SimulatedPlaneContactStateUpdater(groundContactPoint, endEffector, soleFrame);
               contactStateUpdaters.set(robotSextant, contactStateUpdater);
               contactStateUpdatersList.add(contactStateUpdater);
            }
         }
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
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controllerDt, -gravity, rootJoint, controlledJoints, centerOfMassFrame,
                                                                            momentumOptimizationSettings, yoGraphicsListRegistry, registry);
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

   @Override
   public String getDescription()
   {
      return null;
   }

   private boolean firstTick = true;

   @Override
   public void doControl()
   {
      long startTime = System.nanoTime();
      sensorReader.read();
      contactPointVisualizer.update(0.0);
      for (YoGraphicReferenceFrame frame : referenceFrameGraphics)
      {
         frame.update();
      }

      referenceFrames.updateFrames();

      if (firstTick)
      {
         initialize();
         firstTick = false;
      }

      highLevelController.doControl();
      ControllerCoreCommand controllerCoreCommandList = highLevelController.getControllerCoreCommand();
      controllerCore.submitControllerCoreCommand(controllerCoreCommandList);
      controllerCore.compute();
      outputWriter.write();

      totalTimeToCompleteTick.set(System.nanoTime() - startTime);
      totalTimeToCompleteTickInSeconds.set(Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
   }
}
