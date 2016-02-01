package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIKPacketCreator;

public class WholeBodyPacketBehavior extends BehaviorInterface
{
   private final BooleanYoVariable hasInputBeenSet;
   private final BooleanYoVariable hasPacketBeenSent;
   private final BooleanYoVariable isDone;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeElapsed;

   private final WholeBodyIKPacketCreator wholeBodyNetworkModule;
   private final ArrayList<Packet> packetsToSend = new ArrayList<Packet>();

   private SDFFullRobotModel desiredFullRobotModel;

   public WholeBodyPacketBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime,
         WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new DoubleYoVariable(behaviorNameFirstLowerCase + "trajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);
      hasInputBeenSet = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      isDone = new BooleanYoVariable(behaviorNameFirstLowerCase + "IsDone", registry);

      wholeBodyNetworkModule = new WholeBodyIKPacketCreator(wholeBodyControllerParameters);
   }

   public void setInputs(SDFFullRobotModel desiredFullRobotModel, double trajectoryDuration)
   {
      this.desiredFullRobotModel = desiredFullRobotModel;
      this.trajectoryTime.set(trajectoryDuration);
      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet() && !isDone())
      {
         if (!hasPacketBeenSent.getBooleanValue())
         {
            createAndSendPacketsToController();
         }
         else
         {
            trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue());
            if (!isDone.getBooleanValue() && hasInputBeenSet() && !isPaused.getBooleanValue() && !isStopped.getBooleanValue()
                  && trajectoryTimeElapsed.getDoubleValue() > trajectoryTime.getDoubleValue())
            {
               isDone.set(true);
            }
         }
      }
   }

   public void createAndSendPacketsToController()
   {
      packetsToSend.clear();
      startTime.set(yoTime.getDoubleValue());
      wholeBodyNetworkModule.createPackets(desiredFullRobotModel, trajectoryTime.getDoubleValue(), packetsToSend);
      for (int i = 0; i < packetsToSend.size(); i++)
      {
         sendPacketToController(packetsToSend.get(i));
      }
      hasPacketBeenSent.set(true);
   }

   @Override
   public void initialize()
   {
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      isDone.set(false);
      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
      packetsToSend.clear();
      defaultInitialize();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      isDone.set(false);
      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
      packetsToSend.clear();
      defaultPostBehaviorCleanup();
   }

   @Override
   public void stop()
   {
      defaultStop();
   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public void resume()
   {
      defaultResume();
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
