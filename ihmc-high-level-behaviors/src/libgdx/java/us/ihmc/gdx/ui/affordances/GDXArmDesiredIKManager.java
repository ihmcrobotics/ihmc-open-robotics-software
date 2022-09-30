package us.ihmc.gdx.ui.affordances;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.InverseKinematicsCalculator;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.yoVariables.registry.YoRegistry;

public class GDXArmDesiredIKManager
{
   private final RobotSide side;

   // returned as output
   private GeometricJacobian desiredArmJacobian;
   // passed to IK solvers
   private GeometricJacobian workArmJacobian;
   private GeometricJacobian actualArmJacobian;

   private final FramePose3D correctedDesiredHandControlFramePose = new FramePose3D();
   private final FramePose3D lastCorrectedDesiredHandControlFramePose = new FramePose3D();
   private boolean ikFoundASolution = false;
   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 5;
   private int maxIterations = 500;
   private InverseKinematicsCalculator inverseKinematicsCalculator;
   private final RigidBodyTransform controlToWristTransform = new RigidBodyTransform();
   private final ModifiableReferenceFrame temporaryFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());

   private KinematicsToolboxController toolboxController;
   private CommandInputManager commandInputManager;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoRegistry mainRegistry;

   private FullHumanoidRobotModel actualRobot;
   private FullHumanoidRobotModel desiredRobot;
   private FullHumanoidRobotModel workingRobot;
   private KinematicsToolboxOutputStatus ikOutputStatus;

   public GDXArmDesiredIKManager(RobotSide side)
   {
      this.side = side;
   }

   public void create(DRCRobotModel robotModel,
                      FullHumanoidRobotModel actualRobot,
                      FullHumanoidRobotModel desiredRobot,
                      FullHumanoidRobotModel workingRobot)
   {

      this.actualRobot = actualRobot;
      this.desiredRobot = desiredRobot;
      this.workingRobot = workingRobot;
      actualArmJacobian = new GeometricJacobian(actualRobot.getChest(),
                                                actualRobot.getHand(side),
                                                actualRobot.getHand(side).getBodyFixedFrame());
      desiredArmJacobian = new GeometricJacobian(desiredRobot.getChest(),
                                                 desiredRobot.getHand(side),
                                                 desiredRobot.getHand(side).getBodyFixedFrame());
      workArmJacobian = new GeometricJacobian(workingRobot.getChest(),
                                              workingRobot.getHand(side),
                                              workingRobot.getHand(side).getBodyFixedFrame());

      double convergeTolerance = 4.0e-6; //1e-12;
      double parameterChangePenalty = 0.1;
      double positionCost = 1.0;
      double orientationCost = 0.2;
      boolean solveOrientation = true;
      double toleranceForPositionError = 0.005;
      double toleranceForOrientationError = 0.02;
      inverseKinematicsCalculator = new DdoglegInverseKinematicsCalculator(workArmJacobian,
                                                                           positionCost,
                                                                           orientationCost,
                                                                           maxIterations,
                                                                           solveOrientation,
                                                                           convergeTolerance,
                                                                           toleranceForPositionError,
                                                                           toleranceForOrientationError,
                                                                           parameterChangePenalty);

      controlToWristTransform.set(robotModel.getJointMap().getHandControlFrameToWristTransform(side));
      controlToWristTransform.invert();

//      // TODO: need to replace ddogleg
//      mainRegistry = new YoRegistry("main");
//      yoGraphicsListRegistry = new YoGraphicsListRegistry();
//
//      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
//      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
//      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel.getRootBody()));
//
//      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
//
//      double updateDT = 1.0e-3;

//      JointBasics[] joints = workArmJacobian.getJointsInOrder();
//      OneDoFJointBasics[] oneDoFJoints = MultiBodySystemTools.filterJoints(joints, OneDoFJointBasics.class);
//      toolboxController = new KinematicsToolboxController(commandInputManager,
//                                                          statusOutputManager,
//                                                          null,
//                                                          oneDoFJoints,
//                                                          null,
//                                                          updateDT,
//                                                          yoGraphicsListRegistry,
//                                                          mainRegistry);
//      toolboxController.initialize();
//      toolboxController.getSolution().getDesiredJointAngles();
      // track solution quality.
   }

   public void update(GDXHandInteractable handInteractable, FullHumanoidRobotModel desiredRobot)
   {
      desiredArmJacobian.compute();
      actualArmJacobian.compute();

      correctedDesiredHandControlFramePose.setToZero(handInteractable.getControlReferenceFrame());
      correctedDesiredHandControlFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      correctedDesiredHandControlFramePose.get(temporaryFrame.getTransformToParent());
      temporaryFrame.getReferenceFrame().update();
      correctedDesiredHandControlFramePose.setToZero(temporaryFrame.getReferenceFrame());
      correctedDesiredHandControlFramePose.set(controlToWristTransform);
      // IDK where these come from; I tuned using JRebel
      correctedDesiredHandControlFramePose.getTranslation().subZ(.045);
      correctedDesiredHandControlFramePose.getTranslation().subX(.007);
      correctedDesiredHandControlFramePose.getTranslation().subY(side.negateIfLeftSide(.0015));

      // TODO: Frame??
      correctedDesiredHandControlFramePose.changeFrame(desiredRobot.getChest().getBodyFixedFrame());
   }

   public boolean getArmDesiredChanged()
   {
      boolean desiredHandsChanged = !lastCorrectedDesiredHandControlFramePose.geometricallyEquals(correctedDesiredHandControlFramePose, 0.0001);
      lastCorrectedDesiredHandControlFramePose.setIncludingFrame(correctedDesiredHandControlFramePose);
      return desiredHandsChanged;
   }

   public void copyActualToWork()
   {
      copyOneDofJoints(actualArmJacobian.getJointsInOrder(), workArmJacobian.getJointsInOrder());
   }

   public void solve()
   {
      workArmJacobian.compute();
      ikFoundASolution = false;

      // TODO:
      // need to pass in where the actual robot is.

//      List<? extends JointBasics> workingJoints = workingRobot.getRootJoint().subtreeList();
//      List<? extends JointBasics> actualJoints = actualRobot.getRootJoint().subtreeList();
//
//      for (JointStateType jointStateType : JointStateType.values())
//         MultiBodySystemTools.copyJointsState(actualJoints, workingJoints, jointStateType);
//      workingRobot.updateFrames();

//      toolboxController.initialize();
      for (int i = 0; i < INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE && !ikFoundASolution; i++)
      {
         ikFoundASolution = inverseKinematicsCalculator.solve(lastCorrectedDesiredHandControlFramePose);
      }
   }

   public void copyWorkToDesired()
   {
      copyOneDofJoints(workArmJacobian.getJointsInOrder(), desiredArmJacobian.getJointsInOrder());
   }

   public void setDesiredToCurrent()
   {
      copyOneDofJoints(actualArmJacobian.getJointsInOrder(), desiredArmJacobian.getJointsInOrder());
   }

   private void copyOneDofJoints(JointBasics[] inverseDynamicsJoints1, JointBasics[] inverseDynamicsJoints2)
   {
      OneDoFJointBasics[] oneDofJoints1 = MultiBodySystemTools.filterJoints(inverseDynamicsJoints1, OneDoFJointBasics.class);
      OneDoFJointBasics[] oneDofJoints2 = MultiBodySystemTools.filterJoints(inverseDynamicsJoints2, OneDoFJointBasics.class);

      for (int i = 0; i < oneDofJoints1.length; i++)
      {
         oneDofJoints2[i].setQ(oneDofJoints1[i].getQ());
      }
   }

   public FramePose3D getDesiredControlFramePose()
   {
      return lastCorrectedDesiredHandControlFramePose;
   }

   public GeometricJacobian getDesiredArmJacobian()
   {
      return desiredArmJacobian;
   }

   public GeometricJacobian getActualArmJacobian()
   {
      return actualArmJacobian;
   }

   public GeometricJacobian getWorkArmJacobian()
   {
      return workArmJacobian;
   }
}
