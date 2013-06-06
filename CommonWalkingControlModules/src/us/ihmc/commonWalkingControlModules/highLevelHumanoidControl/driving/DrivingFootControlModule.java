package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.EuclideanPositionController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.*;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import javax.media.j3d.Transform3D;
import java.util.List;

/**
 * @author twan
 *         Date: 6/6/13
 */
public class DrivingFootControlModule
{
   private final double AVERAGE_VELOCITY = 0.3;

   private final YoVariableRegistry registry;
   private final GeometricJacobian footJacobian;
   private final FramePoint toePoint;
   private final EuclideanPositionController toePointPositionController;
   private final MomentumBasedController momentumBasedController;

   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredVelocity = new FrameVector();
   private final FrameVector feedForward = new FrameVector();
   private final FrameVector currentVelocity = new FrameVector();

   private final PositionTrajectoryGenerator positionTrajectoryGenerator;
   private final YoFramePoint targetPosition;

   private final DoubleYoVariable trajectoryInitializationTime;
   private final DoubleYoVariable time;

   private final TwistCalculator twistCalculator;
   private final Twist currentTwist = new Twist();
   private final FramePoint toePointInBase = new FramePoint();
   private final ReferenceFrame toePointFrame;

   public DrivingFootControlModule(RigidBody elevator, ContactablePlaneBody contactablePlaneFoot, MomentumBasedController momentumBasedController,
                                   ReferenceFrame vehicleFrame, DoubleYoVariable yoTime, TwistCalculator twistCalculator, YoVariableRegistry parentRegistry)
   {
      RigidBody foot = contactablePlaneFoot.getRigidBody();
      registry = new YoVariableRegistry(foot.getName() + getClass().getSimpleName());
      footJacobian = new GeometricJacobian(elevator, foot, elevator.getBodyFixedFrame());
      toePoint = getCenterToePoint(contactablePlaneFoot);
      String toePointName = foot.getName() + "ToePoint";
      Transform3D transform = new Transform3D();
      transform.set(toePoint.getVectorCopy());
      toePointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(toePointName, toePoint.getReferenceFrame(), transform);
      toePointPositionController = new EuclideanPositionController(toePointName, toePointFrame, registry);
      this.momentumBasedController = momentumBasedController;
      this.time = yoTime;
      trajectoryInitializationTime = new DoubleYoVariable(toePointName + "InitializationTime", registry);

      targetPosition = new YoFramePoint(toePointName + "Target", vehicleFrame, registry);

      PositionProvider initialPositionProvider = new ConstantPositionProvider(toePoint);
      PositionProvider finalPositionProvider = new YoPositionProvider(targetPosition);
      DoubleProvider trajectoryTimeProvider = new AverageVelocityTrajectoryTimeProvider(initialPositionProvider, finalPositionProvider, AVERAGE_VELOCITY, 1e-3);
      this.positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(toePointName + "Trajectory", vehicleFrame, trajectoryTimeProvider,
              initialPositionProvider, finalPositionProvider, registry);

      double kP = 100.0;
      double dampingRatio = 1.0;
      double kD = GainCalculator.computeDerivativeGain(kP, dampingRatio);
      toePointPositionController.setProportionalGains(kP, kP, kP);
      toePointPositionController.setDerivativeGains(kD, kD, kD);

      this.twistCalculator = twistCalculator;

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      targetPosition.set(toePoint.changeFrameCopy(targetPosition.getReferenceFrame()));
      initializeTrajectory();
   }

   public void doControl()
   {
      positionTrajectoryGenerator.compute(time.getDoubleValue() - trajectoryInitializationTime.getDoubleValue());
      positionTrajectoryGenerator.get(desiredPosition);
      positionTrajectoryGenerator.packVelocity(desiredVelocity);
      positionTrajectoryGenerator.packAcceleration(feedForward);

      updateCurrentVelocity();

      FrameVector output = new FrameVector(toePointFrame);
      toePointPositionController.compute(output, desiredPosition, desiredVelocity, currentVelocity, feedForward);
      footJacobian.compute();
      momentumBasedController.setDesiredPointAcceleration(footJacobian, toePoint, output);
   }

   private void updateCurrentVelocity()
   {
      twistCalculator.packRelativeTwist(currentTwist, footJacobian.getBase(), footJacobian.getEndEffector());
      currentTwist.changeFrame(footJacobian.getBaseFrame());
      toePointInBase.setAndChangeFrame(toePoint);
      toePointInBase.changeFrame(footJacobian.getBaseFrame());
      currentTwist.packVelocityOfPointFixedInBodyFrame(currentVelocity, toePointInBase);
   }

   private void initializeTrajectory()
   {
      positionTrajectoryGenerator.initialize();
      trajectoryInitializationTime.set(time.getDoubleValue());
   }

   private static FramePoint getCenterToePoint(ContactablePlaneBody foot)
   {
      FrameVector forward = new FrameVector(foot.getPlaneFrame(), 1.0, 0.0, 0.0);
      int nToePoints = 2;
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(foot.getContactPoints(), forward, nToePoints);
      FramePoint centerToePoint = FramePoint.average(toePoints);

      return centerToePoint;
   }
}
