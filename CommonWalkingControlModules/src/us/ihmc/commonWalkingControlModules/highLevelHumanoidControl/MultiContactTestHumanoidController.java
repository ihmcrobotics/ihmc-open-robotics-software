package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.FixedOrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.FixedPositionTrajectoryGenerator;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class MultiContactTestHumanoidController extends MomentumBasedController
{
   private static final long serialVersionUID = 8199779680768038690L;

   private final ControlFlowInputPort<FramePoint> desiredCoMPositionPort;
   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoM", worldFrame, registry);

   private final GeometricJacobian spineJacobian;
   private final ChestOrientationControlModule chestOrientationControlModule;

   private final List<OneDoFJoint> positionControlJoints;
   private final HashMap<OneDoFJoint, Double> desiredJointPositions = new HashMap<OneDoFJoint, Double>();

   private final ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationPort;
   private final YoFrameOrientation desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", worldFrame, registry);

   private final ReferenceFrame pelvisFrame;

   private final HashMap<ContactablePlaneBody, EndEffectorControlModule> endEffectorControlModules = new HashMap<ContactablePlaneBody,
                                                                                                        EndEffectorControlModule>();
   private final HashMap<ContactablePlaneBody, GeometricJacobian> jacobians = new HashMap<ContactablePlaneBody, GeometricJacobian>();
   private final HashMap<ContactablePlaneBody, FixedPositionTrajectoryGenerator> swingPositionTrajectoryGenerators = new HashMap<ContactablePlaneBody,
                                                                                                                        FixedPositionTrajectoryGenerator>();
   private final HashMap<ContactablePlaneBody, FixedOrientationTrajectoryGenerator> swingOrientationTrajectoryGenerators =
      new HashMap<ContactablePlaneBody, FixedOrientationTrajectoryGenerator>();


   public MultiContactTestHumanoidController(FullRobotModel fullRobotModel, CenterOfMassJacobian centerOfMassJacobian,
           CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable yoTime, double gravityZ, TwistCalculator twistCalculator,
           HashMap<ContactablePlaneBody, RigidBody> contactablePlaneBodiesAndBases, double controlDT, ProcessedOutputsInterface processedOutputs,
           GroundReactionWrenchDistributor groundReactionWrenchDistributor, ArrayList<Updatable> updatables,
           MomentumRateOfChangeControlModule momentumRateOfChangeControlModule, RootJointAccelerationControlModule rootJointAccelerationControlModule,
           ControlFlowInputPort<FramePoint> desiredCoMPositionPort, ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationPort,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(fullRobotModel, centerOfMassJacobian, referenceFrames, yoTime, gravityZ, twistCalculator, contactablePlaneBodiesAndBases.keySet(), controlDT,
            processedOutputs, groundReactionWrenchDistributor, updatables, momentumRateOfChangeControlModule, rootJointAccelerationControlModule,
            dynamicGraphicObjectsListRegistry);
      this.desiredCoMPositionPort = desiredCoMPositionPort;
      this.desiredPelvisOrientationPort = desiredPelvisOrientationPort;

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(fullRobotModel.getElevator());
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

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodiesAndBases.keySet())
      {
         RigidBody base = contactablePlaneBodiesAndBases.get(contactablePlaneBody);
         GeometricJacobian jacobian = new GeometricJacobian(base, contactablePlaneBody.getRigidBody(), contactablePlaneBody.getBodyFrame());
         jacobians.put(contactablePlaneBody, jacobian);

         String bodyName = contactablePlaneBody.getRigidBody().getName();
         FixedPositionTrajectoryGenerator swingPositionTrajectoryGenerator = new FixedPositionTrajectoryGenerator(bodyName + "DesiredPosition", worldFrame,
                                                                                registry);
         swingPositionTrajectoryGenerators.put(contactablePlaneBody, swingPositionTrajectoryGenerator);

         FixedOrientationTrajectoryGenerator swingOrientationTrajectoryGenerator = new FixedOrientationTrajectoryGenerator(bodyName + "DesiredOrientation",
                                                                                      worldFrame, registry);
         swingOrientationTrajectoryGenerators.put(contactablePlaneBody, swingOrientationTrajectoryGenerator);

         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(contactablePlaneBody, jacobian, swingPositionTrajectoryGenerator,
                                                                swingOrientationTrajectoryGenerator, null, yoTime, twistCalculator, registry);
         endEffectorControlModules.put(contactablePlaneBody, endEffectorControlModule);

         positionControlJoints.removeAll(Arrays.asList(jacobian.getJointsInOrder()));
      }
   }

   @Override
   public void initialize()
   {
      super.initialize();

      for (OneDoFJoint oneDoFJoint : positionControlJoints)
      {
         desiredJointPositions.put(oneDoFJoint, oneDoFJoint.getQ());
      }

      FramePoint currentCoM = new FramePoint(centerOfMassFrame);
      currentCoM.changeFrame(desiredCoMPosition.getReferenceFrame());
      desiredCoMPosition.set(currentCoM);

      FrameOrientation currentPelvisOrientaton = new FrameOrientation(pelvisFrame);
      currentPelvisOrientaton.changeFrame(desiredPelvisOrientation.getReferenceFrame());
      desiredPelvisOrientation.set(currentPelvisOrientaton);

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         ReferenceFrame endEffectorFrame = endEffectorControlModules.get(contactablePlaneBody).getEndEffectorFrame();
         swingPositionTrajectoryGenerators.get(contactablePlaneBody).setPosition(new FramePoint(endEffectorFrame));
         swingOrientationTrajectoryGenerators.get(contactablePlaneBody).setOrientation(new FrameOrientation(endEffectorFrame));
      }
   }

   @Override
   public void doMotionControl()
   {
      doChestcontrol();
      doEndEffectorControl();
      doCoMControl();
      doPelvisControl();
      doJointPositionControl();
   }


   private void doChestcontrol()
   {
      chestOrientationControlModule.compute();
      solver.setDesiredSpatialAcceleration(spineJacobian, chestOrientationControlModule.getTaskspaceConstraintData());
   }

   private void doEndEffectorControl()
   {
      for (ContactablePlaneBody contactablePlaneBody : endEffectorControlModules.keySet())
      {

         EndEffectorControlModule endEffectorControlModule = endEffectorControlModules.get(contactablePlaneBody);
         endEffectorControlModule.setContactPoints(contactStates.get(contactablePlaneBody).getContactPoints2d());
         endEffectorControlModule.setCenterOfPressure(centersOfPressure2d.get(contactablePlaneBody).getFramePoint2dCopy());
         endEffectorControlModule.startComputation();
         endEffectorControlModule.waitUntilComputationIsDone();
         TaskspaceConstraintData taskspaceConstraintData = endEffectorControlModule.getTaskSpaceConstraintOutputPort().getData();
         GeometricJacobian jacobian = endEffectorControlModule.getJacobian();
         solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
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
         doPDControl(oneDoFJoint, 100.0, 20.0, desiredJointPositions.get(oneDoFJoint), 0.0);
      }
   }

   public void setContactablePlaneBodiesInContact(ContactablePlaneBody contactablePlaneBody, boolean inContact, double coefficientOfFriction)
   {
      YoPlaneContactState contactState = contactStates.get(contactablePlaneBody);
      if (inContact)
      {
         contactState.set(contactablePlaneBody.getContactPoints2d(), coefficientOfFriction);
      }
      else
         contactState.set(new ArrayList<FramePoint2d>(), coefficientOfFriction);
   }

   protected void doAdditionalTorqueControl()
   {
      // TODO Auto-generated method stub
      
   }
}
