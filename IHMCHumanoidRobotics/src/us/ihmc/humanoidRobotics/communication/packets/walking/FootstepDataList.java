package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;

//To be deleted after the controller rewrite is done. 
//Keeping for right now in order to transform old script files.
public class FootstepDataList extends IHMCRosApiMessage<FootstepDataListMessage>
{
   public ArrayList<FootstepData> footstepDataList = new ArrayList<FootstepData>();

   @FieldDocumentation("swingTime is the time spent in single-support when stepping")
   public double swingTime = 0.0;
   @FieldDocumentation("transferTime is the time spent in double-support between steps")
   public double transferTime = 0.0;
   
   @Override
   public boolean epsilonEquals(FootstepDataListMessage other, double epsilon)
   {
      return false;
   }

}
