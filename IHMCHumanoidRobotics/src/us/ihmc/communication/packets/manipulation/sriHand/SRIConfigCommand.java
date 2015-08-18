package us.ihmc.communication.packets.manipulation.sriHand;

import us.ihmc.communication.packets.Packet;

public class SRIConfigCommand extends Packet<SRIConfigCommand>
{
   public enum Action
   {
      CONFIG_ACT_NONE(0), CONFIG_ACT_READ(1), CONFIG_ACT_WRITE(2), CONFIG_ACT_WRITE_FLASH(3);

      private byte command;

      private Action(int command)
      {
         this.command = (byte) command;
      }

      public byte getCommand()
      {
         return command;
      }
   }

   public Action action;
   public String key;
   public double value;

   public SRIConfigCommand()
   {
      // Empty constructor for serialization
   }

   /**
    * 
    * @param action CONFIG_ACT_NONE, CONFIG_ACT_READ, CONFIG_ACT_WRITE, CONFIG_ACT_WRITE_FLASH
    * @param key Key as defined in resources/SRI/dictionary.yaml
    * @param value
    */
   public SRIConfigCommand(Action action, String key, double value)
   {
      this.action = action;
      this.key = key;
      this.value = value;
   }

   public Action getAction()
   {
      return action;
   }

   public String getKey()
   {
      return key;
   }

   public double getValue()
   {
      return value;
   }

   @Override
   public boolean epsilonEquals(SRIConfigCommand other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public String toString()
   {
      return "SRIConfigCommand \n\taction=" + action + "\n\tkey=" + key + "\n\tvalue=" + value;
   }

}
