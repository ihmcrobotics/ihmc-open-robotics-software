package us.ihmc.valkyrie.treadmill;

import java.io.IOException;
import java.net.Socket;

import us.ihmc.utilities.treadmill.TreadmillInterface;

public class ValkyrieTreadmill implements TreadmillInterface
{
   public static MAVLink mav = new MAVLink();
   public static java.io.InputStream in = null;

   public static volatile double duty = 0;

   public ValkyrieTreadmill()
   {
      
   }
   
   @Override
   public void getSpeed()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setSpeed()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void getIncline()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setIncline()
   {
      // TODO Auto-generated method stub
      
   }
}
