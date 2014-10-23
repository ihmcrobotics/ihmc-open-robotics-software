package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RemovePieceOfDebrisBehavior extends BehaviorInterface
{
   private final ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();

   private final GraspObjectBehavior graspObject;
   private final DropDebrisBehavior dropMic;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private BehaviorInterface currentBehavior;

   public RemovePieceOfDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrame, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      graspObject = new GraspObjectBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      dropMic = new DropDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);

      isDone = new BooleanYoVariable("isDone", registry);
      haveInputsBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);

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
      graspObject.setGraspPose(graspPosition, graspVector);
      haveInputsBeenSet.set(true);
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
      graspObject.initialize();
      dropMic.initialize();
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
