package us.ihmc.exampleSimulations.beetle.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
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
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class HexapodSimulationController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = -9.81;
   private final double controllerDt;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoBoolean useInverseDynamics = new YoBoolean("useInverseDynamics", registry);

   private final SDFPerfectSimulatedSensorReader sensorReader;
   private final PerfectSimulatedOutputWriter outputWriter;
   private final SegmentDependentList<RobotSextant, RigidBody> footRigidBodies = new SegmentDependentList<>(RobotSextant.class);
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
      this.outputWriter = new PerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);
      this.referenceFrames = new HexapodReferenceFrames(fullRobotModel, RhinoBeetlePhysicalProperties.getOffsetsFromJointBeforeFootToSoleAlignedWithWorld());
      setupPlaneContactStateUpdaters(fullRobotModel, sdfRobot);

      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      
      highLevelController = new HexapodHighLevelControlManager(fullRobotModel, referenceFrames, contactStateUpdaters, jointsToControl, idParameters, vmcParameters, yoGraphicsListRegistry, controllerDt, registry);

      FeedbackControlCommandList feedbackControlCommandList = createFeedbackControlTemplate();
      WholeBodyControlCoreToolbox toolbox = makeControllerToolbox();
      this.controllerCore = new WholeBodyControllerCore(toolbox, feedbackControlCommandList, lowLevelControllerCoreOutput, registry);

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         YoGraphicReferenceFrame frame = new YoGraphicReferenceFrame(joint.getFrameBeforeJoint(), registry, 0.1);
         referenceFrameGraphics.add(frame);
         yoGraphicsListRegistry.registerYoGraphic("frames", frame);
      }
   }

   private void setupPlaneContactStateUpdaters(FullRobotModel fullRobotModel, FloatingRootJointRobot sdfRobot)
   {
      ArrayList<GroundContactPoint> groundContactPoints = sdfRobot.getAllGroundContactPoints();
      ArrayList<SimulatedPlaneContactStateUpdater> contactStateUpdatersList = new ArrayList<>();
      RhinoBeetleJointNameMapAndContactDefinition jointMap = new RhinoBeetleJointNameMapAndContactDefinition();
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         RigidBody endEffector = fullRobotModel.getEndEffector(robotSextant);
         footRigidBodies.set(robotSextant, endEffector);

         String jointNameBeforeFoot = jointMap.getJointNameBeforeFoot(robotSextant);
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointNameBeforeFoot);
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
      RigidBody[] controlledBodies = new RigidBody[7];

      int i = 0;
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         RigidBody endEffector = fullRobotModel.getEndEffector(robotSextant);
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         ;
         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(endEffector, footFrame, contactPointsInSoleFrame);
         footContactableBodies.add(footContactableBody);
         controlledBodies[i] = endEffector;
         i++;
      }
      controlledBodies[i] = fullRobotModel.getPelvis();

      //Joints to Control
      InverseDynamicsJoint[] controlledJoints = ScrewTools.computeSubtreeJoints(fullRobotModel.getElevator());

      ControllerCoreOptimizationSettings momentumOptimizationSettings = getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(getClass().getSimpleName(), controllerDt, -gravity, rootJoint, controlledJoints, centerOfMassFrame,
                                                                            momentumOptimizationSettings, yoGraphicsListRegistry, registry);
      toolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);
      toolbox.setupForInverseDynamicsSolver(footContactableBodies);
      toolbox.setupForInverseKinematicsSolver();
      toolbox.setupForVirtualModelControlSolver(fullRobotModel.getPelvis(), controlledBodies, footContactableBodies);

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
   public YoVariableRegistry getYoVariableRegistry()
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
