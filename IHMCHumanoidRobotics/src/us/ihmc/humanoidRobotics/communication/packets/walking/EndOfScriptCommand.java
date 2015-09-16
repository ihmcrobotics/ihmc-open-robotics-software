package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/16/13
 * Time: 3:43 PM
 * To change this template use File | Settings | File Templates.
 */
public class EndOfScriptCommand extends Packet<EndOfScriptCommand>
{
   public EndOfScriptCommand()
   {

   }
   
   public String toString()
   {
	   return this.getClass().getSimpleName();
   }

   @Override
   public boolean epsilonEquals(EndOfScriptCommand other, double epsilon)
   {
      return true;
   }
}
