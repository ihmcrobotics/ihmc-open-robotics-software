package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class HumanoidBehaviorButtonPacket extends Packet<HumanoidBehaviorButtonPacket>
{
   public ArrayList<ButtonData> buttonDataList;
   
   public HumanoidBehaviorButtonPacket(Random random)
   {
      buttonDataList = new ArrayList<ButtonData>();
      int size = Math.abs(random.nextInt(1000));

      for (int i = 0; i < size; i++)
      {
         ButtonData buttonData = new ButtonData(random);
         buttonDataList.add(buttonData);
      }
   }
   
   public HumanoidBehaviorButtonPacket()
   {
   }
   
   public HumanoidBehaviorButtonPacket(ArrayList<ButtonData> buttonData)
   {
      this.buttonDataList = buttonData;
   }
   
   public ArrayList<ButtonData> getButtonDataList()
   {
      return buttonDataList;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorButtonPacket other, double epsilon) 
   {
      boolean ret = buttonDataList.size() == other.getButtonDataList().size();
      
      for (int i = 0; i < buttonDataList.size(); i++)
      {
         ret &= this.getButtonDataList().get(i).epsilonEquals(other.getButtonDataList().get(i), epsilon);
      }
      
      return ret;
   }

}
