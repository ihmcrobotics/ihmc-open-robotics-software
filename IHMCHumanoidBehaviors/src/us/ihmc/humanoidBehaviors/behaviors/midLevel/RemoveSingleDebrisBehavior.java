package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.DropDebrisTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspPieceOfDebrisTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RemoveSingleDebrisBehavior extends BehaviorInterface
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TaskExecutor taskExecutor = new TaskExecutor();

   private final GraspPieceOfDebrisBehavior graspPieceOfDebris;
   private final DropDebrisBehavior dropPieceOfDebris;
   private final WalkToLocationBehavior walkCloseToObjectBehavior;

   private final BooleanYoVariable haveInputsBeenSet;
   private final BooleanYoVariable isObjectTooFar;

   private final WalkingControllerParameters walkingControllerParameters;

   private final FramePose2d targetPose2dInWorld;
   private RobotSide robotSide;

   private final SDFFullRobotModel fullRobotModel;

   private final ReferenceFrame midZupFrame;

   private static final double OPTIMAL_DISTANCE_TO_GRAB_OBJECT = 0.85; //0.85

   private static final Boolean useWholeBodyIK = true;

   public RemoveSingleDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, DoubleYoVariable yoTime, WholeBodyControllerParameters wholeBodyControllerParameters,
         WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.walkingControllerParameters = walkingControllerParameters;

      graspPieceOfDebris = new GraspPieceOfDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, wholeBodyControllerParameters, yoTime, useWholeBodyIK);
      dropPieceOfDebris = new DropDebrisBehavior(outgoingCommunicationBridge, referenceFrames,yoTime);
      walkCloseToObjectBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);

      haveInputsBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);
      isObjectTooFar = new BooleanYoVariable("isObjectTooFar", registry);

      midZupFrame = referenceFrames.getMidFeetZUpFrame();
      targetPose2dInWorld = new FramePose2d(worldFrame);
   }

   @Override
   public void doControl()
   {
      taskExecutor.doControl();
   }

   public void setInputs(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector)
   {
      calculateLocation(graspPosition);
      if (isObjectTooFar.getBooleanValue())
         taskExecutor.submit(new WalkToLocationTask(targetPose2dInWorld, walkCloseToObjectBehavior, 0.0, walkingControllerParameters.getMaxStepLength()));

      robotSide = determineSideToUse(graspPosition);

      taskExecutor.submit(new GraspPieceOfDebrisTask(graspPieceOfDebris, debrisTransform, graspPosition, graspVector, robotSide));
      taskExecutor.submit(new DropDebrisTask(dropPieceOfDebris, robotSide));

      haveInputsBeenSet.set(true);
   }

   private RobotSide determineSideToUse(Point3d position)
   {
      FramePose pose = new FramePose(worldFrame);
      pose.setPose(position, new Quat4d(0, 0, 0, 1));
      pose.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
      if (pose.getY() <= 0.0)
         return RobotSide.RIGHT;
      else
         return RobotSide.LEFT;
   }

   private void calculateLocation(Point3d graspPosition)
   {
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();

      //
      FramePoint graspPositionInChestFame = new FramePoint(worldFrame);
      graspPositionInChestFame.set(graspPosition);
      graspPositionInChestFame.changeFrame(chestFrame);
      //      

      if (!(graspPositionInChestFame.getX() >= OPTIMAL_DISTANCE_TO_GRAB_OBJECT))
         return;

      isObjectTooFar.set(true);

      targetPose2dInWorld.changeFrame(midZupFrame);
      targetPose2dInWorld.setX(graspPositionInChestFame.getX() - OPTIMAL_DISTANCE_TO_GRAB_OBJECT);
      targetPose2dInWorld.setY(0.0);
      targetPose2dInWorld.changeFrame(worldFrame);
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      graspPieceOfDebris.consumeObjectFromNetworkProcessor(object);
      dropPieceOfDebris.consumeObjectFromNetworkProcessor(object);
      walkCloseToObjectBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      graspPieceOfDebris.consumeObjectFromController(object);
      dropPieceOfDebris.consumeObjectFromController(object);
      walkCloseToObjectBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      graspPieceOfDebris.stop();
      dropPieceOfDebris.stop();
      walkCloseToObjectBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      graspPieceOfDebris.enableActions();
      dropPieceOfDebris.enableActions();
      walkCloseToObjectBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      graspPieceOfDebris.pause();
      dropPieceOfDebris.pause();
      walkCloseToObjectBehavior.pause();
   }

   @Override
   public void resume()
   {
      graspPieceOfDebris.resume();
      dropPieceOfDebris.resume();
      walkCloseToObjectBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return (taskExecutor.isDone() && hasInputBeenSet());
   }

   @Override
   public void finalize()
   {
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      isObjectTooFar.set(false);

      haveInputsBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

}
