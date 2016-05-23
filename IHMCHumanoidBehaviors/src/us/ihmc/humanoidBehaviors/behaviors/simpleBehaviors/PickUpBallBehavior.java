package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.humanoidBehaviors.behaviors.*;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class PickUpBallBehavior extends BehaviorInterface
{
   private static final boolean CREATE_COACTIVE_ELEMENT = true;
   private final PickUpBallsBehaviorCoactiveElementBehaviorSide coactiveElement;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   public PickUpBallBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         SDFFullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);

      if (CREATE_COACTIVE_ELEMENT)
      {
         coactiveElement = new PickUpBallsBehaviorCoactiveElementBehaviorSide(this);
         registry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
         registry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());
      }
      else
      {
         coactiveElement = null;
      }
   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   @Override protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {

   }

   @Override protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {

   }

   @Override public void stop()
   {

   }

   @Override public void enableActions()
   {

   }

   @Override public void pause()
   {

   }

   @Override public void resume()
   {

   }

   @Override public boolean isDone()
   {
      return false;
   }

   @Override public void doPostBehaviorCleanup()
   {

   }

   @Override public void initialize()
   {

   }

   @Override public boolean hasInputBeenSet()
   {
      return false;
   }

   @Override
   public CoactiveElement getCoactiveElement()
   {
      return coactiveElement;
   }

}
