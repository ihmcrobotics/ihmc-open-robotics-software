package us.ihmc.atlas.ros;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import multisense_ros.StampedPps;

import org.zeromq.ZMQ;

import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSRequestType;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosTimestampSubscriber;

public class AtlasPPSTimestampOffsetProvider implements PPSTimestampOffsetProvider
{
   private final int ppsPort;
   private final String ppsTopic;
   private RosTimestampSubscriber ppsSubscriber;
   private final AtomicLong currentTimeStampOffset = new AtomicLong(0);
   private final AtomicLong lastTimeStampOffset = new AtomicLong(0);
   private ZMQ.Socket requester;

   private final byte[] requestPayload = { PPSRequestType.GET_NEW_PPS_TIMESTAMP };
   private final ByteBuffer responseBuffer = ByteBuffer.allocate(8);

   private final AtomicBoolean offsetIsDetermined = new AtomicBoolean(false);
   private boolean isRosMainNodeAttached=false;
   private static AtlasPPSTimestampOffsetProvider instance = null;

   private AtlasPPSTimestampOffsetProvider(AtlasSensorInformation sensorInformation)
   {
      ppsPort = sensorInformation.getPPSProviderPort();
      ppsTopic = sensorInformation.getPPSRosTopic();

      setupZMQSocket();

      setupPPSSubscriber();
   }

   private void setupPPSSubscriber()
   {
      ppsSubscriber = new RosTimestampSubscriber()
      {
         @Override
         public void onNewMessage(StampedPps message)
         {
            long robotPPSTimestamp=requestNewestRobotTimestamp();
            long multisensePPSTimestamp=message.getHostTime().totalNsecs();
            currentTimeStampOffset.set( robotPPSTimestamp-multisensePPSTimestamp);
            if(offsetIsDetermined.get() && Math.abs(currentTimeStampOffset.get()-lastTimeStampOffset.get())>1e7)
            {
               System.out.println(getClass().getSimpleName() + " UnstablePPSOffset"+
               String.format("robot %d ns, multisense %d ns,diff, %.4f s",
                     robotPPSTimestamp, multisensePPSTimestamp,
                     TimeTools.nanoSecondstoSeconds(currentTimeStampOffset.get())));
            }
            lastTimeStampOffset.set(currentTimeStampOffset.get());
            offsetIsDetermined.set(true);
         }
      };
   }

   private void setupZMQSocket()
   {
      ZMQ.Context context = ZMQ.context(1);
      requester = context.socket(ZMQ.REQ);
      requester.connect("tcp://" + NetworkParameters.getHost(NetworkParameterKeys.robotController) + ":" + ppsPort);
   }

   @Override
   public synchronized void attachToRosMainNode(RosMainNode rosMainNode)
   {
      if(!isRosMainNodeAttached)
      {
         rosMainNode.attachSubscriber(ppsTopic, ppsSubscriber);
         isRosMainNodeAttached=true;
      }
   }

   @Override
   public long getCurrentTimestampOffset()
   {
      return currentTimeStampOffset.get();
   }

   @Override
   public long adjustTimeStampToRobotClock(long timeStamp)
   {
      return timeStamp + currentTimeStampOffset.get();
   }

   @Override
   public long adjustRobotTimeStampToRosClock(long timeStamp)
   {
      return timeStamp - currentTimeStampOffset.get();
   }

   @Override
   public long requestNewestRobotTimestamp()
   {
      requester.send(requestPayload, 0);
      responseBuffer.rewind();
      responseBuffer.put(requester.recv());
      return responseBuffer.getLong(0);
   }

   @Override
   public boolean offsetIsDetermined()
   {
      return offsetIsDetermined.get();
   }

   public synchronized static PPSTimestampOffsetProvider getInstance(AtlasSensorInformation sensorInformation)
   {
      if(instance == null)
            instance=new AtlasPPSTimestampOffsetProvider(sensorInformation);
      return instance;
   }

}