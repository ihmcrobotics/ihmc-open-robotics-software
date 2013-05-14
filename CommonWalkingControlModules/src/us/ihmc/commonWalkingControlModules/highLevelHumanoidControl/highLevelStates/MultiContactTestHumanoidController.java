package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.head.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.TorusPoseProvider;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.trajectories.FixedOrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.FixedPositionTrajectoryGenerator;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;

import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class MultiContactTestHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   public final static HighLevelState controllerState = HighLevelState.MULTI_CONTACT;

   protected final ControlFlowInputPort<FramePoint> desiredCoMPositionPort;
   protected final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoM", worldFrame, registry);

   public MultiContactTestHumanoidController(SideDependentList<? extends ContactablePlaneBody> feet, SideDependentList<? extends ContactablePlaneBody> hands,
           ControlFlowInputPort<FramePoint> desiredCoMPositionPort, ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationPort,
           DesiredHeadOrientationProvider desiredHeadOrientationProvider, MomentumBasedController momentumBasedController,
           WalkingControllerParameters walkingControllerParameters, DesiredHandPoseProvider handPoseProvider, TorusPoseProvider torusPoseProvider,
           SideDependentList<HandControllerInterface> handControllers, LidarControllerInterface lidarControllerInterface,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(feet, desiredPelvisOrientationPort, desiredHeadOrientationProvider, momentumBasedController, walkingControllerParameters, handPoseProvider,
            torusPoseProvider, handControllers, lidarControllerInterface, dynamicGraphicObjectsListRegistry, controllerState);

      this.desiredCoMPositionPort = desiredCoMPositionPort;
   }

   protected void setupFootControlModules()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = bipedFeet.get(robotSide);
         GeometricJacobian jacobian = legJacobians.get(robotSide);

         String bodyName = foot.getRigidBody().getName();
         FixedPositionTrajectoryGenerator swingPositionTrajectoryGenerator = new FixedPositionTrajectoryGenerator(bodyName + "DesiredPosition", worldFrame,
                                                                                registry);
         swingPositionTrajectoryGenerators.put(foot, swingPositionTrajectoryGenerator);

         FixedOrientationTrajectoryGenerator swingOrientationTrajectoryGenerator = new FixedOrientationTrajectoryGenerator(bodyName + "DesiredOrientation",
                                                                                      worldFrame, registry);
         swingOrientationTrajectoryGenerators.put(foot, swingOrientationTrajectoryGenerator);

         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(foot, jacobian, swingPositionTrajectoryGenerator, null,
                                                                swingOrientationTrajectoryGenerator, null, yoTime, twistCalculator, registry);
         footEndEffectorControlModules.put(foot, endEffectorControlModule);

      }
   }

   public void initialize()
   {
      super.initialize();

      FramePoint currentCoM = new FramePoint(momentumBasedController.getCenterOfMassFrame());
      currentCoM.changeFrame(desiredCoMPosition.getReferenceFrame());
      desiredCoMPosition.set(currentCoM);

      FrameOrientation currentPelvisOrientaton = new FrameOrientation(referenceFrames.getPelvisFrame());
      currentPelvisOrientaton.changeFrame(desiredPelvisOrientation.getReferenceFrame());
      desiredPelvisOrientation.set(currentPelvisOrientaton);

      // keep desired pelvis orientation as it is
      desiredPelvisAngularVelocity.set(0.0, 0.0, 0.0);
      desiredPelvisAngularAcceleration.set(0.0, 0.0, 0.0);

      for (ContactablePlaneBody contactablePlaneBody : footEndEffectorControlModules.keySet())
      {
         ReferenceFrame endEffectorFrame = footEndEffectorControlModules.get(contactablePlaneBody).getEndEffectorFrame();
         swingPositionTrajectoryGenerators.get(contactablePlaneBody).setPosition(new FramePoint(endEffectorFrame));
         swingOrientationTrajectoryGenerators.get(contactablePlaneBody).setOrientation(new FrameOrientation(endEffectorFrame));
      }

   }

   protected void doCoMControl()
   {
      desiredCoMPositionPort.setData(desiredCoMPosition.getFramePointCopy());
   }

   protected void doFootControl()
   {
      for (ContactablePlaneBody contactablePlaneBody : footEndEffectorControlModules.keySet())
      {
         EndEffectorControlModule endEffectorControlModule = footEndEffectorControlModules.get(contactablePlaneBody);
         List<FramePoint2d> contactPoints = momentumBasedController.getContactStates().get(contactablePlaneBody).getContactPoints2d();
         ConstraintType constraintType = EndEffectorControlModule.getUnconstrainedForZeroAndFullyConstrainedForFourContactPoints(contactPoints.size());
         endEffectorControlModule.setContactPoints(contactPoints, constraintType);
         endEffectorControlModule.setCenterOfPressure(momentumBasedController.getCoP(contactablePlaneBody));
      }

      super.doFootControl();
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
}
