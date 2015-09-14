package us.ihmc.humanoidBehaviors.behaviors.primitives;

import javax.vecmath.Point3d;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.LookAtStatus;
import us.ihmc.communication.packets.sensing.LookAtPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.FormattingTools;
import us.ihmc.utilities.io.printing.PrintTools;

public class LookAtBehavior extends BehaviorInterface
{
   private final boolean DEBUG = false;
   
   private LookAtPacket packetToSend;
   private int lookAtPacketIndex = 0;
   
   private final double trajectoryTime;
   
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTimeElapsed;
   
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable inputsHaveBeenSupplied;
   private final ConcurrentListeningQueue<LookAtPacket> inputListeningQueue = new ConcurrentListeningQueue<LookAtPacket>();
   private final ConcurrentListeningQueue<LookAtStatus> statusListeningQueue = new ConcurrentListeningQueue<LookAtStatus>();

   public LookAtBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, WalkingControllerParameters walkingControllerParameters, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = walkingControllerParameters.getTrajectoryTimeHeadOrientation();
      trajectoryTimeElapsed = new DoubleYoVariable(behaviorNameFirstLowerCase + "trajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);
      isDone = new BooleanYoVariable(behaviorName + "_isDone", registry);
      inputsHaveBeenSupplied = new BooleanYoVariable(behaviorName + "_inputsHaveBeenSupplied", registry);
   }

   @Override
   public void doControl()
   {
      trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue());
      
      if (inputListeningQueue.isNewPacketAvailable())
      {
         packetToSend = inputListeningQueue.getNewestPacket();
         lookAtPacketIndex = packetToSend.getIndex();
         inputsHaveBeenSupplied.set(true);
      }

      if (packetToSend != null)
      {
         sendPacketToController(packetToSend);
         packetToSend = null;
         startTime.set(yoTime.getDoubleValue());
      }

      if (statusListeningQueue.isNewPacketAvailable())
      {
         LookAtStatus lookAtStatus = statusListeningQueue.getNewestPacket();
         if (lookAtStatus.getIndex() == lookAtPacketIndex)
         {
            isDone.set(lookAtStatus.isFinished());
         }
      }
      
      if (!isDone.getBooleanValue() && hasInputBeenSet() && !isPaused.getBooleanValue() && !isStopped.getBooleanValue()
            && trajectoryTimeElapsed.getDoubleValue() > trajectoryTime)
      {
         if (DEBUG)
            PrintTools.debug(this, "setting isDone = true");
         isDone.set(true);
      }
   }

   public void setLookAtLocation(Point3d pointToLookAt)
   {
      lookAtPacketIndex++;
      packetToSend = new LookAtPacket(pointToLookAt, lookAtPacketIndex);
      inputsHaveBeenSupplied.set(true);
   }

   public boolean isLooking()
   {
      return hasInputBeenSet() && !isDone();
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
   public void stop()
   {

   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void pause()
   {

   }

   @Override
   public void resume()
   {

   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      initialize();
   }

   @Override
   public void initialize()
   {
      inputsHaveBeenSupplied.set(false);
      isDone.set(false);
      inputListeningQueue.clear();
      statusListeningQueue.clear();

      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return inputsHaveBeenSupplied.getBooleanValue();
   }
}
