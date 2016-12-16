package us.ihmc.communication.packets;

import java.util.Random;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ValveDetectorResultPacket extends Packet<ValveDetectorResultPacket>
{
   public HeatMapPacket heatMap;
   public BoundingBoxesPacket boundingBoxes;

   public ValveDetectorResultPacket()
   {

   }

   public ValveDetectorResultPacket(HeatMapPacket heatMap, BoundingBoxesPacket boundingBoxes)
   {
      this.heatMap = heatMap;
      this.boundingBoxes = boundingBoxes;
   }

   public ValveDetectorResultPacket(Random random)
   {
      this.heatMap = new HeatMapPacket(random);
      this.boundingBoxes = new BoundingBoxesPacket(random);
   }

   @Override
   public boolean epsilonEquals(ValveDetectorResultPacket other, double epsilon)
   {
      return this.heatMap.epsilonEquals(other.heatMap, epsilon) && this.boundingBoxes.epsilonEquals(other.boundingBoxes, epsilon);
   }
}
