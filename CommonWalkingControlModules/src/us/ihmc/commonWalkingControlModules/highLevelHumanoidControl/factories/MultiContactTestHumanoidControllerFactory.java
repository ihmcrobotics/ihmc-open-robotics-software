package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MultiContactTestHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OldMomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAngularAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FingerForceSensors;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointGroundReactionWrenchDistributor;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromControllerSink;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class MultiContactTestHumanoidControllerFactory implements HighLevelHumanoidControllerFactory
{
   private final SideDependentList<String> namesOfJointsBeforeHands;
   private final SideDependentList<Transform3D> handContactPointTransforms;
   private final SideDependentList<List<Point2d>> handContactPoints;
   private final RobotSide[] footContactSides;
   private final RobotSide[] handContactSides;

   public MultiContactTestHumanoidControllerFactory(SideDependentList<String> namesOfJointsBeforeHands,
         SideDependentList<Transform3D> handContactPointTransforms, SideDependentList<List<Point2d>> handContactPoints, RobotSide[] footContactSides,
         RobotSide[] handContactSides)
   {
      this.namesOfJointsBeforeHands = namesOfJointsBeforeHands;
      this.handContactPointTransforms = handContactPointTransforms;
      this.handContactPoints = handContactPoints;
      this.footContactSides = footContactSides;
      this.handContactSides = handContactSides;
   }

   public RobotController create(RigidBody estimationLink, ReferenceFrame estimationFrame, FullRobotModel fullRobotModel,
         CommonWalkingReferenceFrames referenceFrames, FingerForceSensors fingerForceSensors, DoubleYoVariable yoTime, double gravityZ,
         TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian, SideDependentList<ContactablePlaneBody> feet, double controlDT,
         SideDependentList<FootSwitchInterface> footSwitches, SideDependentList<HandControllerInterface> handControllers,
         LidarControllerInterface lidarControllerInterface, StateEstimationDataFromControllerSink stateEstimationDataFromControllerSink,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry registry, GUISetterUpperRegistry guiSetterUpperRegistry,
         ProcessedOutputsInterface processedOutputs)
   {
      LinkedHashMap<ContactablePlaneBody, RigidBody> contactablePlaneBodiesAndBases = new LinkedHashMap<ContactablePlaneBody, RigidBody>();

      // TODO: code duplication from driving controller
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      SideDependentList<ContactablePlaneBody> hands = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values())
      {
         InverseDynamicsJoint[] jointBeforeHandArray = ScrewTools.findJointsWithNames(allJoints, namesOfJointsBeforeHands.get(robotSide));
         if (jointBeforeHandArray.length != 1)
            throw new RuntimeException("Incorrect number of joints before hand found: " + jointBeforeHandArray.length);

         RigidBody handBody = jointBeforeHandArray[0].getSuccessor();

         ReferenceFrame afterHipFrame = handBody.getParentJoint().getFrameAfterJoint();
         ReferenceFrame handContactsFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression()
               + "HandContact", afterHipFrame, handContactPointTransforms.get(robotSide));

         ContactablePlaneBody hand = new ListOfPointsContactablePlaneBody(handBody, handContactsFrame, handContactPoints.get(robotSide));
         hands.put(robotSide, hand);
      }

      for (ContactablePlaneBody contactablePlaneBody : feet.values())
      {
         contactablePlaneBodiesAndBases.put(contactablePlaneBody, fullRobotModel.getPelvis());
      }

      for (ContactablePlaneBody contactablePlaneBody : hands.values())
      {
         contactablePlaneBodiesAndBases.put(contactablePlaneBody, fullRobotModel.getChest());
      }

      ContactPointGroundReactionWrenchDistributor groundReactionWrenchDistributor = new ContactPointGroundReactionWrenchDistributor(
            referenceFrames.getCenterOfMassFrame(), registry);
      double[] diagonalCWeights = new double[] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
      groundReactionWrenchDistributor.setWeights(diagonalCWeights, 1.0, 0.001);

      CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule = new CoMBasedMomentumRateOfChangeControlModule(
            referenceFrames.getCenterOfMassFrame(), centerOfMassJacobian, registry);

      double comProportionalGain = 100.0;
      double dampingRatio = 1.0;
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double comDerivativeGain = GainCalculator.computeDampingForSecondOrderSystem(totalMass, comProportionalGain, dampingRatio);
      momentumRateOfChangeControlModule.setProportionalGains(comProportionalGain, comProportionalGain, comProportionalGain);
      momentumRateOfChangeControlModule.setDerivativeGains(comDerivativeGain, comDerivativeGain, comDerivativeGain);

      RootJointAngularAccelerationControlModule rootJointAccelerationControlModule = new RootJointAngularAccelerationControlModule(
            fullRobotModel.getRootJoint(), twistCalculator, null, registry);
      rootJointAccelerationControlModule.setProportionalGains(100.0, 100.0, 100.0);
      rootJointAccelerationControlModule.setDerivativeGains(20.0, 20.0, 20.0);

      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);

      OldMomentumControlModule momentumControlModule = new OldMomentumControlModule(fullRobotModel.getRootJoint(), contactablePlaneBodiesAndBases.keySet(),
            gravityZ, groundReactionWrenchDistributor, referenceFrames.getCenterOfMassFrame(), controlDT, twistCalculator, jacobianSolver, registry);
      double groundReactionWrenchBreakFrequencyHertz = 7.0;
      momentumControlModule.setGroundReactionWrenchBreakFrequencyHertz(groundReactionWrenchBreakFrequencyHertz);

      // The controllers do not extend the MomentumBasedController anymore. Instead, it is passed through the constructor.
      MomentumBasedController momentumBasedController = new MomentumBasedController(estimationLink, estimationFrame, fullRobotModel, centerOfMassJacobian,
            referenceFrames, yoTime, gravityZ, twistCalculator, contactablePlaneBodiesAndBases.keySet(), controlDT, processedOutputs, momentumControlModule,
            null, momentumRateOfChangeControlModule, rootJointAccelerationControlModule, stateEstimationDataFromControllerSink,
            dynamicGraphicObjectsListRegistry);

      MultiContactTestHumanoidController multiContactBehavior = new MultiContactTestHumanoidController(fullRobotModel, yoTime, twistCalculator,
            contactablePlaneBodiesAndBases, momentumRateOfChangeControlModule.getDesiredCoMPositionInputPort(),
            rootJointAccelerationControlModule.getDesiredPelvisOrientationTrajectoryInputPort(), momentumBasedController);

      double coefficientOfFriction = 1.0;

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodiesAndBases.keySet())
      {
         multiContactBehavior.setContactablePlaneBodiesInContact(contactablePlaneBody, false, coefficientOfFriction);
      }

      for (RobotSide robotSide : footContactSides)
      {
         multiContactBehavior.setContactablePlaneBodiesInContact(feet.get(robotSide), true, coefficientOfFriction);
      }

      for (RobotSide robotSide : handContactSides)
      {
         multiContactBehavior.setContactablePlaneBodiesInContact(hands.get(robotSide), true, coefficientOfFriction);
      }

      // Creation of the "highest level" state machine.
      StateMachine<HighLevelState> highLevelStateMachine = new StateMachine<HighLevelState>("highLevelStateMachine", "switchTimeName", HighLevelState.class,
            yoTime, registry);
      // Creating a dummy StateTransition to remain in the multi-contact controller
      StateTransition<HighLevelState> noStateTransition = new StateTransition<HighLevelState>(null, new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return false;
         }
      });

      multiContactBehavior.addStateTransition(noStateTransition);
      highLevelStateMachine.addState(multiContactBehavior);

      ArrayList<YoVariableRegistry> multiContactControllerRegistry = new ArrayList<YoVariableRegistry>();
      multiContactControllerRegistry.add(multiContactBehavior.getYoVariableRegistry());
      multiContactControllerRegistry.add(momentumBasedController.getYoVariableRegistry()); //TODO: no sure if that should be done here or inside the multiContactBehavior controller...

      // This is the "highest level" controller that enables switching between the different controllers (walking, multi-contact, driving, etc.)
      HighLevelHumanoidControllerManager ret = new HighLevelHumanoidControllerManager(highLevelStateMachine, HighLevelState.MULTI_CONTACT,
            multiContactControllerRegistry);

      return ret;
   }

}
