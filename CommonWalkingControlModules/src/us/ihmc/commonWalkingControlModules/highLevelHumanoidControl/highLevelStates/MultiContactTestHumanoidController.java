package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableRollingBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAngularAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.trajectories.ChangeableConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;

public class MultiContactTestHumanoidController extends AbstractHighLevelHumanoidControlPattern implements VariableChangedListener
{
   public final static HighLevelState controllerState = HighLevelState.MULTI_CONTACT;

   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoM", worldFrame, registry);

   private final DesiredFootPoseProvider footPoseProvider;

   private final BooleanYoVariable l_footDoHeelOff = new BooleanYoVariable("l_footDoHeelOff", registry);
   private final BooleanYoVariable r_footDoHeelOff = new BooleanYoVariable("r_footDoHeelOff", registry);
   private final SideDependentList<BooleanYoVariable> doHeelOff = new SideDependentList<BooleanYoVariable>(l_footDoHeelOff, r_footDoHeelOff);

   private final LinkedHashMap<ContactablePlaneBody, ChangeableConfigurationProvider> desiredConfigurationProviders = new LinkedHashMap<ContactablePlaneBody,
                                                                                                                         ChangeableConfigurationProvider>();
   private final LinkedHashMap<ContactablePlaneBody, StraightLinePositionTrajectoryGenerator> swingPositionTrajectoryGenerators =
      new LinkedHashMap<ContactablePlaneBody, StraightLinePositionTrajectoryGenerator>();
   private final LinkedHashMap<ContactablePlaneBody, OrientationInterpolationTrajectoryGenerator> swingOrientationTrajectoryGenerators =
      new LinkedHashMap<ContactablePlaneBody, OrientationInterpolationTrajectoryGenerator>();


   private final ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(1.0);
   private final CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule;

   public MultiContactTestHumanoidController(VariousWalkingProviders variousWalkingProviders,
           CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule,
           RootJointAngularAccelerationControlModule rootJointAccelerationControlModule, MomentumBasedController momentumBasedController,
           WalkingControllerParameters walkingControllerParameters, 
           SideDependentList<HandControllerInterface> handControllers, LidarControllerInterface lidarControllerInterface,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(variousWalkingProviders, rootJointAccelerationControlModule, momentumBasedController, walkingControllerParameters, handControllers,
            lidarControllerInterface, dynamicGraphicObjectsListRegistry, controllerState);

      this.footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      this.momentumRateOfChangeControlModule = momentumRateOfChangeControlModule;

      setupFootControlModules();

      for (final RobotSide robotSide : RobotSide.values)
      {
         doHeelOff.get(robotSide).addVariableChangedListener(this);
      }
   }

   protected void setupFootControlModules()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);
         GeometricJacobian jacobian = legJacobians.get(robotSide);

         String bodyName = foot.getRigidBody().getName();


         final ConstantConfigurationProvider currentConfigurationProvider = new ConstantConfigurationProvider(new FramePose(foot.getBodyFrame()));
         final ChangeableConfigurationProvider desiredConfigurationProvider =
            new ChangeableConfigurationProvider(footPoseProvider.getDesiredFootPose(robotSide));

         StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(bodyName, worldFrame,
                                                                                  trajectoryTimeProvider.getValue(), currentConfigurationProvider,
                                                                                  desiredConfigurationProvider, registry, false);

         OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(bodyName, worldFrame,
                                                                                         trajectoryTimeProvider, currentConfigurationProvider,
                                                                                         desiredConfigurationProvider, registry, false);

         desiredConfigurationProviders.put(foot, desiredConfigurationProvider);
         swingPositionTrajectoryGenerators.put(foot, positionTrajectoryGenerator);
         swingOrientationTrajectoryGenerators.put(foot, orientationTrajectoryGenerator);

         DoubleTrajectoryGenerator onToesFixedTrajectory = createDummyDoubleTrajectoryGenerator();

         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(foot, jacobian, positionTrajectoryGenerator, null,
                                                                orientationTrajectoryGenerator, onToesFixedTrajectory, momentumBasedController, registry);
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
   }

   protected void doCoMControl()
   {
      momentumRateOfChangeControlModule.getDesiredCoMPositionInputPort().setData(desiredCoMPosition.getFramePointCopy());

      momentumRateOfChangeControlModule.startComputation();
      momentumRateOfChangeControlModule.waitUntilComputationIsDone();
      MomentumRateOfChangeData momentumRateOfChangeData = momentumRateOfChangeControlModule.getMomentumRateOfChangeOutputPort().getData();
      momentumBasedController.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   protected void doFootControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);

         if (footPoseProvider.checkForNewPose(robotSide))
         {
            FramePose newFootPose = footPoseProvider.getDesiredFootPose(robotSide);
            desiredConfigurationProviders.get(foot).set(newFootPose);
            footEndEffectorControlModules.get(foot).resetCurrentState();
         }

         EndEffectorControlModule endEffectorControlModule = footEndEffectorControlModules.get(foot);
         FramePoint2d cop = momentumBasedController.getCoP(foot);
         endEffectorControlModule.setCenterOfPressure(cop);
      }

      super.doFootControl();
   }

   public void setFootInContact(RobotSide robotSide, boolean inContact)
   {
      if (feet == null)
         return;

      if (inContact)
      {
         setFlatFootContactState(feet.get(robotSide));
      }
      else
      {
         setContactStateForSwing(feet.get(robotSide));
      }
   }

   public void setHandInContact(RobotSide robotSide, boolean inContact)
   {
      ContactablePlaneBody handPalm = handPalms.get(robotSide);

      if (inContact)
      {
         // TODO: If we know the surface normal here, use it.
         FrameVector normalContactVector = null;
         momentumBasedController.setPlaneContactState(handPalm, handPalm.getContactPoints2d(), coefficientOfFriction.getDoubleValue(), normalContactVector);
      }
      else
      {
         FrameVector normalContactVector = null;
         momentumBasedController.setPlaneContactState(handPalm, new ArrayList<FramePoint2d>(), coefficientOfFriction.getDoubleValue(), normalContactVector);
      }
   }

   private void setOnToesContactState(ContactablePlaneBody contactableBody)
   {
      FrameVector normalContactVector = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
      List<FramePoint> contactPoints = getContactPointsAccordingToFootConstraint(contactableBody, ConstraintType.TOES);
      List<FramePoint2d> contactPoints2d = getContactPoints2d(contactableBody, contactPoints);
      setContactState(contactableBody, contactPoints2d, ConstraintType.TOES, normalContactVector);
   }

   private void setFlatFootContactState(ContactablePlaneBody contactableBody)
   {
      FrameVector normalContactVector = new FrameVector(contactableBody.getPlaneFrame(), 0.0, 0.0, 1.0);
      setContactState(contactableBody, contactableBody.getContactPoints2d(), ConstraintType.FULL, normalContactVector);
   }

   private void setContactStateForSwing(ContactablePlaneBody contactableBody)
   {
      // Initialize desired foot pose to the actual, so no surprising behavior
      ReferenceFrame footFrame = footEndEffectorControlModules.get(contactableBody).getEndEffectorFrame();
      desiredConfigurationProviders.get(contactableBody).set(new FramePose(footFrame));

      FrameVector normalContactVector = new FrameVector(contactableBody.getPlaneFrame(), 0.0, 0.0, 1.0);
      setContactState(contactableBody, new ArrayList<FramePoint2d>(), ConstraintType.UNCONSTRAINED, normalContactVector);
   }

   private List<FramePoint> getContactPointsAccordingToFootConstraint(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector direction = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);
      if (constraintType == ConstraintType.HEEL)
         direction.scale(-1.0);

      return DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPoints(), direction, 2);
   }

   private List<FramePoint2d> getContactPoints2d(ContactablePlaneBody contactableBody, List<FramePoint> contactPoints)
   {
      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getPlaneFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      return contactPoints2d;
   }

   private void setContactState(ContactablePlaneBody contactableBody, List<FramePoint2d> contactPoints, ConstraintType constraintType,
                                FrameVector normalContactVector)
   {
      if (contactPoints.size() == 0)
      {
         footEndEffectorControlModules.get(contactableBody).doSingularityEscapeBeforeTransitionToNextState();
      }

      momentumBasedController.setPlaneContactState(contactableBody, contactPoints, coefficientOfFriction.getDoubleValue(), normalContactVector);

      updateEndEffectorControlModule(contactableBody, contactPoints, constraintType);
   }

   private void updateEndEffectorControlModule(ContactablePlaneBody contactablePlaneBody, List<FramePoint2d> contactPoints, ConstraintType constraintType)
   {
      footEndEffectorControlModules.get(contactablePlaneBody).setContactPoints(contactPoints, constraintType);
   }

   public void variableChanged(YoVariable v)
   {
      if (!(v instanceof BooleanYoVariable))
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (v.equals(doHeelOff.get(robotSide)))
         {
            if (doHeelOff.get(robotSide).getBooleanValue())
            {
               setOnToesContactState(feet.get(robotSide));
            }
            else
            {
               setFlatFootContactState(feet.get(robotSide));
            }
         }
      }
   }

   private DoubleTrajectoryGenerator createDummyDoubleTrajectoryGenerator()
   {
      DoubleTrajectoryGenerator onToesFixedTrajectory = new DoubleTrajectoryGenerator()
      {
         public double getValue()
         {
            return 0;
         }

         public boolean isDone()
         {
            return true;
         }

         public void initialize()
         {
         }

         public void compute(double time)
         {
         }

         public double getVelocity()
         {
            return 0;
         }

         public double getAcceleration()
         {
            return 0;
         }
      };

      return onToesFixedTrajectory;
   }
}
