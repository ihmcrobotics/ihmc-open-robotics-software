package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.yoUtilities.controllers.PositionController;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;


/**
 * @author twan
 *         Date: 5/9/13
 */
public class PointPositionHandControlState extends State<HandControlState>
{
   private final YoVariableRegistry registry;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final MomentumBasedController momentumBasedController;
   private final TwistCalculator twistCalculator;
   private final YoFramePoint yoDesiredPosition;


   private PositionTrajectoryGenerator positionTrajectoryGenerator;
   private PositionController positionController;

   private FramePoint pointInBody = new FramePoint();
   
   private int jacobianId;
   private RigidBody base;
   private RigidBody endEffector;

   // temp stuff:
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);
   private final FrameVector currentVelocity = new FrameVector(worldFrame);

   private final FrameVector pointAcceleration = new FrameVector(worldFrame);
   private Twist currentTwist = new Twist();

   private final FramePoint point = new FramePoint(worldFrame);

   public PointPositionHandControlState(MomentumBasedController momentumBasedController, RobotSide robotSide,
                                        YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(null); //HandControlState.POINT_POSITION);
      this.momentumBasedController = momentumBasedController;
      this.twistCalculator = momentumBasedController.getTwistCalculator();

      String stateName = FormattingTools.underscoredToCamelCase(this.getStateEnum().toString(), true) + "State";
      String name = robotSide.getCamelCaseNameForStartOfExpression() + stateName;
      registry = new YoVariableRegistry(name);
      String desiredHandPositionName = robotSide.getCamelCaseNameForStartOfExpression() + "HandDesPointPosition";
      yoDesiredPosition = new YoFramePoint(desiredHandPositionName, worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition desiredPositionViz = new YoGraphicPosition(desiredHandPositionName, yoDesiredPosition, 0.01, YoAppearance.FireBrick());
         yoGraphicsListRegistry.registerYoGraphic(stateName, desiredPositionViz);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void doAction()
   {
      point.setIncludingFrame(pointInBody);
      point.changeFrame(base.getBodyFixedFrame());

      updateCurrentVelocity(point);

      positionTrajectoryGenerator.compute(getTimeInCurrentState());

      positionTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
      pointAcceleration.setToZero(positionController.getBodyFrame());

      positionController.compute(pointAcceleration, desiredPosition, desiredVelocity, currentVelocity, desiredAcceleration);

      desiredPosition.changeFrame(worldFrame);
      yoDesiredPosition.set(desiredPosition);

      pointAcceleration.changeFrame(base.getBodyFixedFrame());

      momentumBasedController.setDesiredPointAcceleration(jacobianId, point, pointAcceleration);
   }


   private void updateCurrentVelocity(FramePoint point)
   {
      twistCalculator.packRelativeTwist(currentTwist, base, endEffector);
      currentTwist.changeFrame(base.getBodyFixedFrame());
      currentTwist.packLinearVelocityOfPointFixedInBodyFrame(currentVelocity, point);
   }

   @Override
   public void doTransitionIntoAction()
   {
      positionTrajectoryGenerator.initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public boolean isDone()
   {
      return positionTrajectoryGenerator.isDone();
   }

   public void setTrajectory(PositionTrajectoryGenerator positionTrajectoryGenerator,
                             PositionController positionController, FramePoint pointInBody, int jacobianId)
   {
      this.positionTrajectoryGenerator = positionTrajectoryGenerator;
      this.positionController = positionController;
      this.pointInBody.setIncludingFrame(pointInBody);
      this.jacobianId = jacobianId;
      this.base = momentumBasedController.getJacobian(jacobianId).getBase();
      this.endEffector = momentumBasedController.getJacobian(jacobianId).getEndEffector();
   }

   public FramePoint getDesiredPosition()
   {
      positionTrajectoryGenerator.compute(getTimeInCurrentState());
      positionTrajectoryGenerator.get(desiredPosition);
      return desiredPosition;
   }

   public ReferenceFrame getFrameToControlPoseOf()
   {
      return positionController.getBodyFrame();
   }
}
