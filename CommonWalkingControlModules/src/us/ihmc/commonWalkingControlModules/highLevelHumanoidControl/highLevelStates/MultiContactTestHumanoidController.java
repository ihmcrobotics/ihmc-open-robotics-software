package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.trajectories.FixedOrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.FixedPositionTrajectoryGenerator;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.statemachines.State;

public class MultiContactTestHumanoidController extends State<HighLevelState>
{
   public final static HighLevelState controllerState = HighLevelState.MULTI_CONTACT;
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final ControlFlowInputPort<FramePoint> desiredCoMPositionPort;
   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoM", worldFrame, registry);

   private final GeometricJacobian spineJacobian;
   private final ChestOrientationControlModule chestOrientationControlModule;

   private final List<OneDoFJoint> positionControlJoints;
   private final LinkedHashMap<OneDoFJoint, Double> desiredJointPositions = new LinkedHashMap<OneDoFJoint, Double>();

   private final ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationPort;
   private final YoFrameOrientation desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", worldFrame, registry);

   private final ReferenceFrame pelvisFrame;

   private final LinkedHashMap<ContactablePlaneBody, EndEffectorControlModule> footEndEffectorControlModules = new LinkedHashMap<ContactablePlaneBody, EndEffectorControlModule>();
   private final LinkedHashMap<ContactablePlaneBody, EndEffectorControlModule> handEndEffectorControlModules = new LinkedHashMap<ContactablePlaneBody, EndEffectorControlModule>();
   
   private final LinkedHashMap<ContactablePlaneBody, GeometricJacobian> jacobians = new LinkedHashMap<ContactablePlaneBody, GeometricJacobian>();
   private final LinkedHashMap<ContactablePlaneBody, FixedPositionTrajectoryGenerator> swingPositionTrajectoryGenerators = new LinkedHashMap<ContactablePlaneBody, FixedPositionTrajectoryGenerator>();
   private final LinkedHashMap<ContactablePlaneBody, FixedOrientationTrajectoryGenerator> swingOrientationTrajectoryGenerators = new LinkedHashMap<ContactablePlaneBody, FixedOrientationTrajectoryGenerator>();

   private final MomentumBasedController momentumBasedController;
   private final FullRobotModel fullRobotModel;
   private final DoubleYoVariable yoTime;
   private final TwistCalculator twistCalculator;
   
   private final SideDependentList<? extends ContactablePlaneBody> feet, hands;

   public MultiContactTestHumanoidController(SideDependentList<? extends ContactablePlaneBody> feet, SideDependentList<? extends ContactablePlaneBody> hands,
                                             HashMap<ContactablePlaneBody, RigidBody> contactablePlaneBodiesAndBases, ControlFlowInputPort<FramePoint> desiredCoMPositionPort,
                                             ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationPort, MomentumBasedController momentumBasedController)
   {
      super(controllerState);

      // Getting parameters from the momentumBasedController
      this.momentumBasedController = momentumBasedController;
      fullRobotModel = momentumBasedController.getFullRobotModel();
      yoTime = momentumBasedController.getYoTime();
      twistCalculator = momentumBasedController.getTwistCalculator();
      
      
      this.desiredCoMPositionPort = desiredCoMPositionPort;
      this.desiredPelvisOrientationPort = desiredPelvisOrientationPort;
      
      this.hands = hands;
      this.feet = feet;
      
      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      OneDoFJoint[] positionControlJointArray = new OneDoFJoint[ScrewTools.computeNumberOfJointsOfType(OneDoFJoint.class, joints)];
      ScrewTools.filterJoints(joints, positionControlJointArray, OneDoFJoint.class);
      positionControlJoints = new ArrayList<OneDoFJoint>(Arrays.asList(positionControlJointArray));

      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      spineJacobian = new GeometricJacobian(pelvis, chest, chest.getBodyFixedFrame());
      chestOrientationControlModule = new ChestOrientationControlModule(pelvis, chest, spineJacobian, twistCalculator, registry);
      chestOrientationControlModule.setProportionalGains(100.0, 100.0, 100.0);
      chestOrientationControlModule.setDerivativeGains(20.0, 20.0, 20.0);
      positionControlJoints.removeAll(Arrays.asList(spineJacobian.getJointsInOrder()));

      pelvisFrame = pelvis.getBodyFixedFrame();

      for (RobotSide robotSide : RobotSide.values)
      {
         // Creating foot trajectory generators and end-effector control modules:
         ContactablePlaneBody foot = feet.get(robotSide);
         GeometricJacobian jacobian = new GeometricJacobian(pelvis, foot.getRigidBody(), foot.getBodyFrame());
         jacobians.put(feet.get(robotSide), jacobian);

         String bodyName = foot.getRigidBody().getName();
         FixedPositionTrajectoryGenerator swingPositionTrajectoryGenerator =
               new FixedPositionTrajectoryGenerator(bodyName + "DesiredPosition", worldFrame, registry);
         swingPositionTrajectoryGenerators.put(foot, swingPositionTrajectoryGenerator);

         FixedOrientationTrajectoryGenerator swingOrientationTrajectoryGenerator =
               new FixedOrientationTrajectoryGenerator(bodyName + "DesiredOrientation", worldFrame, registry);
         swingOrientationTrajectoryGenerators.put(foot, swingOrientationTrajectoryGenerator);

         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(foot, jacobian, swingPositionTrajectoryGenerator,
               null, swingOrientationTrajectoryGenerator, null, yoTime, twistCalculator, registry);
         footEndEffectorControlModules.put(foot, endEffectorControlModule);

         positionControlJoints.removeAll(Arrays.asList(jacobian.getJointsInOrder()));

         // Creating hand trajectory generators and end-effector control modules:
         ContactablePlaneBody hand = hands.get(robotSide);
         jacobian = new GeometricJacobian(chest, hand.getRigidBody(), hand.getBodyFrame());
         jacobians.put(hand, jacobian);

         bodyName = hand.getRigidBody().getName();
         swingPositionTrajectoryGenerator =
               new FixedPositionTrajectoryGenerator(bodyName + "DesiredPosition", worldFrame, registry);
         swingPositionTrajectoryGenerators.put(hand, swingPositionTrajectoryGenerator);

         swingOrientationTrajectoryGenerator =
               new FixedOrientationTrajectoryGenerator(bodyName + "DesiredOrientation", worldFrame, registry);
         swingOrientationTrajectoryGenerators.put(hand, swingOrientationTrajectoryGenerator);

         endEffectorControlModule = new EndEffectorControlModule(hand, jacobian, swingPositionTrajectoryGenerator,
               null, swingOrientationTrajectoryGenerator, null, yoTime, twistCalculator, registry);
         handEndEffectorControlModules.put(hand, endEffectorControlModule);

         positionControlJoints.removeAll(Arrays.asList(jacobian.getJointsInOrder()));
      }
   }

   public void initialize()
   {
      momentumBasedController.initialize();
      
      for (OneDoFJoint oneDoFJoint : positionControlJoints)
      {
         desiredJointPositions.put(oneDoFJoint, oneDoFJoint.getQ());
      }

      FramePoint currentCoM = new FramePoint(momentumBasedController.getCenterOfMassFrame());
      currentCoM.changeFrame(desiredCoMPosition.getReferenceFrame());
      desiredCoMPosition.set(currentCoM);

      FrameOrientation currentPelvisOrientaton = new FrameOrientation(pelvisFrame);
      currentPelvisOrientaton.changeFrame(desiredPelvisOrientation.getReferenceFrame());
      desiredPelvisOrientation.set(currentPelvisOrientaton);

      for (ContactablePlaneBody contactablePlaneBody : footEndEffectorControlModules.keySet())
      {
         ReferenceFrame endEffectorFrame = footEndEffectorControlModules.get(contactablePlaneBody).getEndEffectorFrame();
         swingPositionTrajectoryGenerators.get(contactablePlaneBody).setPosition(new FramePoint(endEffectorFrame));
         swingOrientationTrajectoryGenerators.get(contactablePlaneBody).setOrientation(new FrameOrientation(endEffectorFrame));
      }
      
      for (ContactablePlaneBody contactablePlaneBody : handEndEffectorControlModules.keySet())
      {
         ReferenceFrame endEffectorFrame = handEndEffectorControlModules.get(contactablePlaneBody).getEndEffectorFrame();
         swingPositionTrajectoryGenerators.get(contactablePlaneBody).setPosition(new FramePoint(endEffectorFrame));
         swingOrientationTrajectoryGenerators.get(contactablePlaneBody).setOrientation(new FrameOrientation(endEffectorFrame));
      }
   }

   public void doMotionControl()
   {
      momentumBasedController.doPrioritaryControl();
      
      doLegControl();
      doArmControl();
      doChestcontrol();
      doCoMControl();
      doPelvisControl();
      doJointPositionControl();
      
      momentumBasedController.doSecondaryControl();
   }

   private void doChestcontrol()
   {
      chestOrientationControlModule.compute();
      momentumBasedController.setDesiredSpatialAcceleration(spineJacobian, chestOrientationControlModule.getTaskspaceConstraintData());
   }

   private void doLegControl()
   {
      for (ContactablePlaneBody contactablePlaneBody : footEndEffectorControlModules.keySet())
      {
         EndEffectorControlModule endEffectorControlModule = footEndEffectorControlModules.get(contactablePlaneBody);
         List<FramePoint2d> contactPoints = momentumBasedController.getContactStates().get(contactablePlaneBody).getContactPoints2d();
         ConstraintType constraintType = EndEffectorControlModule.getUnconstrainedForZeroAndFullyConstrainedForFourContactPoints(contactPoints.size());
         endEffectorControlModule.setContactPoints(contactPoints, constraintType);
         endEffectorControlModule.setCenterOfPressure(momentumBasedController.getCoP(contactablePlaneBody));
         endEffectorControlModule.startComputation();
         endEffectorControlModule.waitUntilComputationIsDone();
         TaskspaceConstraintData taskspaceConstraintData = endEffectorControlModule.getTaskSpaceConstraintOutputPort().getData();
         GeometricJacobian jacobian = endEffectorControlModule.getJacobian();
         momentumBasedController.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
   }
   
   private void doArmControl()
   {
      for (ContactablePlaneBody contactablePlaneBody : handEndEffectorControlModules.keySet())
      {
         EndEffectorControlModule endEffectorControlModule = handEndEffectorControlModules.get(contactablePlaneBody);
         List<FramePoint2d> contactPoints = momentumBasedController.getContactStates().get(contactablePlaneBody).getContactPoints2d();
         ConstraintType constraintType = EndEffectorControlModule.getUnconstrainedForZeroAndFullyConstrainedForFourContactPoints(contactPoints.size());
         endEffectorControlModule.setContactPoints(contactPoints, constraintType);
         endEffectorControlModule.setCenterOfPressure(momentumBasedController.getCoP(contactablePlaneBody));
         endEffectorControlModule.startComputation();
         endEffectorControlModule.waitUntilComputationIsDone();
         TaskspaceConstraintData taskspaceConstraintData = endEffectorControlModule.getTaskSpaceConstraintOutputPort().getData();
         GeometricJacobian jacobian = endEffectorControlModule.getJacobian();
         momentumBasedController.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
   }

   private void doCoMControl()
   {
      desiredCoMPositionPort.setData(desiredCoMPosition.getFramePointCopy());
   }

   private void doPelvisControl()
   {
      OrientationTrajectoryData pelvisOrientationTrajectoryData = new OrientationTrajectoryData();
      FrameOrientation desiredPelvisOrientation = this.desiredPelvisOrientation.getFrameOrientationCopy();
      pelvisOrientationTrajectoryData.set(desiredPelvisOrientation, new FrameVector(pelvisFrame), new FrameVector(pelvisFrame));
      desiredPelvisOrientationPort.setData(pelvisOrientationTrajectoryData);
   }

   private void doJointPositionControl()
   {
      for (OneDoFJoint oneDoFJoint : positionControlJoints)
      {
         momentumBasedController.doPDControl(oneDoFJoint, 100.0, 20.0, desiredJointPositions.get(oneDoFJoint), 0.0);
      }
   }

   public void setContactablePlaneBodiesInContact(ContactablePlaneBody contactablePlaneBody, boolean inContact, double coefficientOfFriction)
   {
      YoPlaneContactState contactState = momentumBasedController.getContactStates().get(contactablePlaneBody);
      if (inContact)
      {
         contactState.set(contactablePlaneBody.getContactPoints2d(), coefficientOfFriction);
      }
      else
         contactState.set(new ArrayList<FramePoint2d>(), coefficientOfFriction);
   }

   //TODO: New methods coming from extending State class
   public void doAction()
   {
      doMotionControl();
   }

   public void doTransitionIntoAction()
   {
      initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

}
