package us.ihmc.valkyrie.torquespeedcurve;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class MultiContactStaticController implements RobotController
{
   private static final FrameVector3DReadOnly zeroVector = new FrameVector3D();
   private static final QuaternionReadOnly zeroOrientation = new Quaternion();
   private static final FrameVector3DReadOnly zUpWorld = new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis.Z);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final HumanoidReferenceFrames referenceFrames;
   private final OneDoFJointBasics[] controlledJoints;
   private final WholeBodyControllerCore wholeBodyControllerCore;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final YoDouble[] desiredJointPositions;
   private final YoFrameQuaternion desiredHeadOrientation = new YoFrameQuaternion("desiredHeadOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameQuaternion desiredChestOrientation = new YoFrameQuaternion("desiredChestOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameQuaternion desiredPelvisOrientation = new YoFrameQuaternion("desiredPelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D desiredCenterOfMassPosition = new YoFramePoint3D("desiredCenterOfMassPosition", ReferenceFrame.getWorldFrame(), registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final OneDoFJointFeedbackControlCommand[] jointFeedbackControlCommands;
   private final OrientationFeedbackControlCommand headOrientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final OrientationFeedbackControlCommand chestOrientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final OrientationFeedbackControlCommand pelvisOrientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final CenterOfMassFeedbackControlCommand centerOfMassFeedbackControlCommand = new CenterOfMassFeedbackControlCommand();
   private final PlaneContactStateCommand[] planeContactStateCommands;
   private final SpatialAccelerationCommand[] contactAccelerationCommands;

   private final YoPDGains jointGains = new YoPDGains("jointGains", registry);
   private final YoPID3DGains taskspaceAccelerationGains = new SymmetricYoPIDSE3Gains("taskspaceAccelerationGains", false, registry);
   private final YoPID3DGains centerOfMassGains = new SymmetricYoPIDSE3Gains("CenterOfMass", false, registry);

   private final YoDouble jointWeight = new YoDouble("jointWeight", registry);
   private final YoFrameVector3D centerOfMassWeight = new YoFrameVector3D("centerOfMassWeight", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble contactAccelerationWeight = new YoDouble("contactAccelerationWeight", registry);

   private final YoDouble coefficientOfFriction = new YoDouble("coefficientOfFriction", registry);

   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;

   private final YoFramePoint2D centerOfMass2D = new YoFramePoint2D("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameConvexPolygon2D supportPolygon = new YoFrameConvexPolygon2D("supportPolygon", ReferenceFrame.getWorldFrame(), 10, registry);
   private final List<YoFramePoint3D> yoContactPoints = new ArrayList<>();
   private final FullHumanoidRobotModel fullRobotModel;

   public MultiContactStaticController(double controlDT, double gravityZ, FullHumanoidRobotModel fullRobotModel,
                                       ControllerCoreOptimizationSettings optimizationSettings, WholeBodyControllerParameters<RobotSide> controllerParameters,
                                       List<? extends ContactablePlaneBody> contactablePlaneBodies, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.contactablePlaneBodies = contactablePlaneBodies;

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      controlledJoints = fullRobotModel.getControllableOneDoFJoints();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                                            gravityZ,
                                                                            rootJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame,
                                                                            optimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      toolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);
      toolbox.setJointPrivilegedConfigurationParameters(controllerParameters.getWalkingControllerParameters().getJointPrivilegedConfigurationParameters());
      FeedbackControlCommandList template = new FeedbackControlCommandList();
      template.addCommand(centerOfMassFeedbackControlCommand);
      headOrientationFeedbackControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getHead());
      template.addCommand(headOrientationFeedbackControlCommand);
      chestOrientationFeedbackControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getChest());
      template.addCommand(chestOrientationFeedbackControlCommand);
      pelvisOrientationFeedbackControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getPelvis());
      template.addCommand(pelvisOrientationFeedbackControlCommand);

      for (OneDoFJointBasics joint : controlledJoints)
      {
         OneDoFJointFeedbackControlCommand command = new OneDoFJointFeedbackControlCommand();
         command.setJoint(joint);
         template.addCommand(command);
      }
      wholeBodyControllerCore = new WholeBodyControllerCore(toolbox, template, registry);

      desiredJointPositions = new YoDouble[controlledJoints.length];
      jointFeedbackControlCommands = new OneDoFJointFeedbackControlCommand[controlledJoints.length];

      for (int i = 0; i < desiredJointPositions.length; i++)
      {
         desiredJointPositions[i] = new YoDouble("qDesired" + controlledJoints[i].getName(), registry);
         jointFeedbackControlCommands[i] = new OneDoFJointFeedbackControlCommand();
         jointFeedbackControlCommands[i].setJoint(controlledJoints[i]);
      }

      jointDesiredOutputList = new JointDesiredOutputList(controlledJoints);

      jointGains.setPDGains(10.0, 10.0);
      jointGains.createDerivativeGainUpdater(true);
      taskspaceAccelerationGains.setProportionalGains(10.0);
      taskspaceAccelerationGains.setDerivativeGains(30.0);
      centerOfMassGains.setProportionalGains(100.0);
      centerOfMassGains.setDerivativeGains(40.0);

      jointWeight.set(2.5);
      centerOfMassWeight.set(1.0, 1.0, 1.0);
      contactAccelerationWeight.set(100.0);
      coefficientOfFriction.set(0.7);

      planeContactStateCommands = new PlaneContactStateCommand[contactablePlaneBodies.size()];
      contactAccelerationCommands = new SpatialAccelerationCommand[contactablePlaneBodies.size()];

      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);

         PlaneContactStateCommand contactCommand = new PlaneContactStateCommand();
         contactCommand.setContactingRigidBody(contactablePlaneBody.getRigidBody());
         planeContactStateCommands[i] = contactCommand;

         SpatialAccelerationCommand accelerationCommand = new SpatialAccelerationCommand();
         accelerationCommand.set(fullRobotModel.getElevator(), contactablePlaneBody.getRigidBody());
         accelerationCommand.setSelectionMatrixForLinearControl();
         contactAccelerationCommands[i] = accelerationCommand;
      }

      yoGraphicsListRegistry.registerArtifact("Balance",
                                              new YoGraphicPosition("centerOfMass",
                                                                    centerOfMass2D,
                                                                    0.03,
                                                                    YoAppearance.Black(),
                                                                    GraphicType.BALL_WITH_CROSS).createArtifact());
      yoGraphicsListRegistry.registerArtifact("Balance", new YoArtifactPolygon("supportPolygon", supportPolygon, Color.BLUE, false));

      for (int i = 0; i < 30; i++)
      {
         YoFramePoint3D yoContactPoint = new YoFramePoint3D("contactPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         yoGraphicsListRegistry.registerArtifact("Balance", new YoArtifactPosition("contactPoint" + i, yoContactPoint.getYoX(), yoContactPoint.getYoY(), GraphicType.BALL, Color.CYAN, 0.02));
         yoContactPoints.add(yoContactPoint);
      }
   }

   @Override
   public void initialize()
   {
      referenceFrames.updateFrames();
      desiredHeadOrientation.setFromReferenceFrame(fullRobotModel.getHead().getBodyFixedFrame());
      desiredChestOrientation.setFromReferenceFrame(fullRobotModel.getChest().getBodyFixedFrame());
      desiredPelvisOrientation.setFromReferenceFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
      desiredCenterOfMassPosition.setFromReferenceFrame(referenceFrames.getCenterOfMassFrame());

      for (int i = 0; i < controlledJoints.length; i++)
      {
         desiredJointPositions[i].set(controlledJoints[i].getQ());
      }
   }

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   @Override
   public void doControl()
   {
      referenceFrames.updateFrames();

      centerOfMass2D.setFromReferenceFrame(referenceFrames.getCenterOfMassFrame());
      supportPolygon.clear();
      yoContactPoints.forEach(YoFramePoint3D::setToNaN);
      int contactPointIndex = 0;

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         List<FramePoint3D> contactPoints = contactablePlaneBody.getContactPointsCopy();
         contactPoints.forEach(point -> point.changeFrame(ReferenceFrame.getWorldFrame()));
         contactPoints.forEach(point -> supportPolygon.addVertex(point));
         for (FramePoint3D contactPoint : contactPoints)
         {
            yoContactPoints.get(contactPointIndex++).set(contactPoint);
         }
      }
      supportPolygon.update();

      controllerCoreCommand.clear();

      privilegedConfigurationCommand.clear();

      for (int i = 0; i < controlledJoints.length; i++)
      {
         YoDouble desiredJointPosition = desiredJointPositions[i];
         OneDoFJointFeedbackControlCommand command = jointFeedbackControlCommands[i];
         command.setInverseDynamics(desiredJointPosition.getValue(), 0.0, 0.0);
         command.setGains(jointGains);
         command.setWeightForSolver(jointWeight.getValue());
         controllerCoreCommand.addFeedbackControlCommand(command);
         privilegedConfigurationCommand.addJoint(controlledJoints[i], desiredJointPositions[i].getValue());
      }

      headOrientationFeedbackControlCommand.setInverseDynamics(desiredHeadOrientation, zeroVector, zeroVector);
      headOrientationFeedbackControlCommand.setGains(taskspaceAccelerationGains);
      headOrientationFeedbackControlCommand.setWeightForSolver(jointWeight.getValue());
      controllerCoreCommand.addFeedbackControlCommand(headOrientationFeedbackControlCommand);

      chestOrientationFeedbackControlCommand.setInverseDynamics(desiredChestOrientation, zeroVector, zeroVector);
      chestOrientationFeedbackControlCommand.setGains(taskspaceAccelerationGains);
      chestOrientationFeedbackControlCommand.setWeightForSolver(jointWeight.getValue());
      controllerCoreCommand.addFeedbackControlCommand(chestOrientationFeedbackControlCommand);

      pelvisOrientationFeedbackControlCommand.setInverseDynamics(desiredPelvisOrientation, zeroVector, zeroVector);
      pelvisOrientationFeedbackControlCommand.setGains(taskspaceAccelerationGains);
      pelvisOrientationFeedbackControlCommand.setWeightForSolver(jointWeight.getValue());
      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationFeedbackControlCommand);

      centerOfMassFeedbackControlCommand.setInverseDynamics(desiredCenterOfMassPosition, zeroVector, zeroVector);
      centerOfMassFeedbackControlCommand.setGains(centerOfMassGains);
      centerOfMassFeedbackControlCommand.setWeightsForSolver(centerOfMassWeight);
      controllerCoreCommand.addFeedbackControlCommand(centerOfMassFeedbackControlCommand);

      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         List<FramePoint3D> contactPoints = contactablePlaneBody.getContactPointsCopy();
         RigidBodyBasics rigidBody = contactablePlaneBody.getRigidBody();
         MovingReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

         for (FramePoint3D contactPoint : contactPoints)
         {
            contactPoint.changeFrame(bodyFixedFrame);
         }

         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommands[i];
         planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getValue());
         planeContactStateCommand.setContactNormal(zUpWorld);
         planeContactStateCommand.setPointsInContact(contactPoints);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);

         SpatialAccelerationCommand contactAccelerationCommand = contactAccelerationCommands[i];
         contactAccelerationCommand.setWeight(contactAccelerationWeight.getValue());
         ReferenceFrame controlFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("contact",
                                                                                                         bodyFixedFrame,
                                                                                                         new RigidBodyTransform(zeroOrientation,
                                                                                                                                contactPoints.get(0)));
         contactAccelerationCommand.setLinearAcceleration(controlFrame, new FrameVector3D(controlFrame));
         controllerCoreCommand.addInverseDynamicsCommand(contactAccelerationCommand);
      }

      wholeBodyControllerCore.submitControllerCoreCommand(controllerCoreCommand);
      wholeBodyControllerCore.compute();
      jointDesiredOutputList.overwriteWith(wholeBodyControllerCore.getOutputForLowLevelController());
   }

   public JointDesiredOutputList getJointDesiredOutputList()
   {
      return jointDesiredOutputList;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public YoFrameConvexPolygon2D getSupportPolygon()
   {
      return supportPolygon;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
}
