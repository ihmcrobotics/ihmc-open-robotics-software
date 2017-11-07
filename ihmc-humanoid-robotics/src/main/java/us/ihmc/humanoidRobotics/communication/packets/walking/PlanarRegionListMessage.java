package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

import java.util.ArrayList;
import java.util.Iterator;

public class PlanarRegionListMessage extends Packet<PlanarRegionListMessage> implements Iterable<PlanarRegionListMessage>, VisualizablePacket
{
   public ArrayList<PlanarRegionMessage> planarRegionList = new ArrayList<>();

   public PlanarRegionListMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public PlanarRegionListMessage(ArrayList<PlanarRegionMessage> planarRegionList)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      if (planarRegionList != null)
      {
         this.planarRegionList = planarRegionList;
      }
   }

   public ArrayList<PlanarRegionMessage> getDataList()
   {
      return planarRegionList;
   }

   public void add(PlanarRegionMessage planarRegionMessage)
   {
      this.planarRegionList.add(planarRegionMessage);
   }

   public void clear()
   {
      this.planarRegionList.clear();
   }

   public PlanarRegionMessage get(int i)
   {
      return this.planarRegionList.get(i);
   }

   public int size()
   {
      return this.planarRegionList.size();
   }

   @Override
   public boolean epsilonEquals(PlanarRegionListMessage otherList, double epsilon)
   {
      for (int i = 0; i < size(); i++)
      {
         if (!otherList.get(i).epsilonEquals(get(i), epsilon))
         {
            return false;
         }
      }

      return true;
   }

   @Override
   public String toString()
   {
      return "TODO";
   }

   @Override
   public Iterator<PlanarRegionMessage> iterator()
   {
      return planarRegionList.iterator();
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePlanarRegionListMessage(this);
   }

}
