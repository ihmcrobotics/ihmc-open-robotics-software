package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class RemoveSingleDebrisBehavior extends BehaviorInterface
{
   private final ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();

   private final GraspObjectBehavior graspObject;
   private final DropDebrisBehavior dropMic;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private BehaviorInterface currentBehavior;

   private final WalkToLocationBehavior walkCloseToObject;

   private final Point3d targetLocation;
   private final YoFrameOrientation targetOrientation;

   private final FullRobotModel fullRobotModel;

   private static final double OPTIMAL_DISTANCE_TO_GRAB_OBJECT = 0.5;



   
   public RemoveSingleDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrame, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;

      graspObject = new GraspObjectBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      dropMic = new DropDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      walkCloseToObject = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrame);

      isDone = new BooleanYoVariable("isDone", registry);
      haveInputsBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);

      targetLocation = new Point3d();
      targetOrientation = new YoFrameOrientation(getName() + "TargetOrientation", ReferenceFrame.getWorldFrame(), registry);

   }



   @Override
   public void doControl()
   {
      if (!isDone.getBooleanValue())
      {
         if (currentBehavior != null)
         {
            checkTransitionCondition();
            currentBehavior.doControl();
         }
      }
   }

   private void checkTransitionCondition()
   {
      if (currentBehavior.isDone())
      {
         currentBehavior.finalize();
         if (!behaviors.isEmpty())
         {
            currentBehavior = behaviors.remove(0);
            currentBehavior.initialize();

            if (currentBehavior instanceof DropDebrisBehavior)
               dropMic.setInputs(graspObject.getSideToUse());
         }
         else
         {
            isDone.set(true);
         }
      }
   }

   public void setInputs(RigidBodyTransform graspTransform, Point3d graspPosition, Vector3d graspVector)
   {
      double rotationAngleAboutNormal = 0.0;
      graspObject.setGraspPose(graspPosition, graspVector, rotationAngleAboutNormal);
//      checkStandHeight(graspPosition);
      if (!isObjectCloseEnough(graspPosition))
      {
         calculateLocation(graspPosition);
         walkCloseToObject.setTarget(targetLocation, targetOrientation);
      }
      else
         currentBehavior = behaviors.remove(0);
      haveInputsBeenSet.set(true);
   }

   private void calculateLocation(Point3d graspPosition)
   {
      FramePoint target = new FramePoint(fullRobotModel.getChest().getBodyFixedFrame());

      //
      FramePoint graspPositionInChestFame = new FramePoint(ReferenceFrame.getWorldFrame());
      graspPositionInChestFame.set(graspPosition);
      graspPositionInChestFame.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
      //      

      target.setToZero();
      target.setX(graspPositionInChestFame.getX() - OPTIMAL_DISTANCE_TO_GRAB_OBJECT);
      target.changeFrame(ReferenceFrame.getWorldFrame());
      targetLocation.set(target.getPoint());
      FrameOrientation frame = new FrameOrientation();
      frame.setToZero(fullRobotModel.getChest().getBodyFixedFrame());
      frame.changeFrame(ReferenceFrame.getWorldFrame());
      targetOrientation.set(frame);
   }

   private boolean isObjectCloseEnough(Point3d graspPosition)
   {
      //
      FramePoint graspPositionInChestFame = new FramePoint(ReferenceFrame.getWorldFrame());
      graspPositionInChestFame.set(graspPosition);
      graspPositionInChestFame.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
      //      
      if (graspPositionInChestFame.getX() >= OPTIMAL_DISTANCE_TO_GRAB_OBJECT)
         return false;
      return true;
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      if (currentBehavior != null)
         currentBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      if (currentBehavior != null)
         currentBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      currentBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      currentBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      currentBehavior.pause();
   }

   @Override
   public void resume()
   {
      currentBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void finalize()
   {
      behaviors.clear();
      isDone.set(false);
      currentBehavior = null;
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      behaviors.clear();
      isDone.set(false);

      walkCloseToObject.initialize();
      graspObject.initialize();
      dropMic.initialize();
      behaviors.add(walkCloseToObject);
      behaviors.add(graspObject);
      behaviors.add(dropMic);
      currentBehavior = behaviors.remove(0);
      haveInputsBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

}
