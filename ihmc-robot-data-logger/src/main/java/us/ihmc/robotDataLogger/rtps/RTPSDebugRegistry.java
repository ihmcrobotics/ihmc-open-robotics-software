package us.ihmc.robotDataLogger.rtps;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class RTPSDebugRegistry
{
   private final YoInteger skippedPackets;
   private final YoInteger skippedIncomingPackets;
   private final YoInteger nonIncreasingTimestamps;
   private final YoInteger packetsOutOfOrder;
   private final YoInteger mergedPackets;
   private final YoInteger totalPackets;
   private final YoInteger skippedPacketDueToFullBuffer;
   private final YoInteger firstSegmentsMissing;

   private final YoVariableRegistry loggerDebugRegistry = new YoVariableRegistry("loggerStatus");
   
   public RTPSDebugRegistry()
   {
      
      this.skippedPackets = new YoInteger("skippedPackets", loggerDebugRegistry);
      this.skippedIncomingPackets = new YoInteger("skippedIncomingPackets", loggerDebugRegistry);
      this.nonIncreasingTimestamps = new YoInteger("nonIncreasingTimestamps", loggerDebugRegistry);
      this.packetsOutOfOrder = new YoInteger("packetsOutOfOrder", loggerDebugRegistry);
      this.mergedPackets = new YoInteger("mergedPackets", loggerDebugRegistry);
      this.totalPackets = new YoInteger("totalPackets", loggerDebugRegistry);
      this.skippedPacketDueToFullBuffer = new YoInteger("skippedPacketDueToFullBuffer", loggerDebugRegistry);
      this.firstSegmentsMissing = new YoInteger("firstSegmentsMissing", loggerDebugRegistry);
   }

   public YoInteger getSkippedPackets()
   {
      return skippedPackets;
   }

   public YoInteger getSkippedIncomingPackets()
   {
      return skippedIncomingPackets;
   }

   public YoInteger getNonIncreasingTimestamps()
   {
      return nonIncreasingTimestamps;
   }

   public YoInteger getPacketsOutOfOrder()
   {
      return packetsOutOfOrder;
   }

   public YoInteger getMergedPackets()
   {
      return mergedPackets;
   }

   public YoInteger getTotalPackets()
   {
      return totalPackets;
   }

   public YoInteger getSkippedPacketDueToFullBuffer()
   {
      return skippedPacketDueToFullBuffer;
   }

   public YoInteger getFirstSegmentsMissing()
   {
      return firstSegmentsMissing;
   }
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return loggerDebugRegistry;
   }
}
