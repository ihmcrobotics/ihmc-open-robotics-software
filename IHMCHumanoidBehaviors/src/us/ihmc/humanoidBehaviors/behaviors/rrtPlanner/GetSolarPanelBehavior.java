package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.time.YoStopwatch;

public class GetSolarPanelBehavior extends AbstractBehavior
{  
   private final String prefix = getClass().getSimpleName();
   private final DoubleYoVariable yoTime;
   
   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(30);
   
   
   private final BooleanYoVariable receivedPlanarRegionsList;
   private final YoStopwatch requestNewPlanarRegionsTimer;
   private final DoubleYoVariable planarRegionsResponseTimeout;
   
   private PlanarRegionsList planarRegionsList;
   
   
   private boolean isDone = false;
   
   public GetSolarPanelBehavior(CommunicationBridgeInterface communicationBridge, DoubleYoVariable yoTime)
   {
      super(communicationBridge);      
      
      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
      this.yoTime = yoTime;
      
      receivedPlanarRegionsList = new BooleanYoVariable(prefix + "ReceivedPlanarRegionsList", registry);
      requestNewPlanarRegionsTimer = new YoStopwatch(yoTime);
      planarRegionsResponseTimeout = new DoubleYoVariable(prefix + "PlanarRegionsResponseTimeout", registry);
      
      planarRegionsResponseTimeout.set(3.0);
      
      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);

      
      PrintTools.info("Finished initiate");
   }
   
   private void requestPlanarRegions()
   {
//      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.REINITIALIZE);
//      message.setDestination(PacketDestination.REA_MODULE);      
//      sendPacket(message);
      
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(
            RequestPlanarRegionsListMessage.RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
      sendPacket(requestPlanarRegionsListMessage);
   }
   
   @Override
   public void doControl()
   {
      
      if (planarRegionsListQueue.isNewPacketAvailable())
      {
         PrintTools.info("Available ");
         
         PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
         planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         //footstepPlanner.setPlanarRegions(planarRegionsList);
         receivedPlanarRegionsList.set(true);
         //havePlanarRegionsBeenSet.set(true);
      }
      //else if (requestNewPlanarRegionsTimer.totalElapsed() > planarRegionsResponseTimeout.getDoubleValue())
      else
      {
         PrintTools.info("Not yet ");
         
         requestPlanarRegions();
         requestNewPlanarRegionsTimer.reset();
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      PrintTools.info("Entered GetSolarPanelBehavior ");
      receivedPlanarRegionsList.set(false);
      requestNewPlanarRegionsTimer.reset();
      requestPlanarRegions();
      
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorExited()
   {
      PrintTools.info("Exit GetSolarPanelBehavior ");
      
   }

   @Override
   public boolean isDone()
   {
      return receivedPlanarRegionsList.getBooleanValue();
   }

}
