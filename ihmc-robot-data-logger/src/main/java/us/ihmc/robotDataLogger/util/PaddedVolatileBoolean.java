package us.ihmc.robotDataLogger.util;

import org.openjdk.jol.info.ClassLayout;

/**
 * 
 * Padded boolean that is guaranteed to be on its own 64 bit cache line
 * 
 * @author Jesper Smith
 *
 */
public class PaddedVolatileBoolean extends PaddedVolatileLong
{
   public PaddedVolatileBoolean()
   {
      
   }
   
   public PaddedVolatileBoolean(boolean initialValue)
   {
      set(initialValue);
   }
   
   public boolean getBoolean()
   {
      return getLong() == 1;
   }
   
   public void set(boolean value)
   {
      set(value ? 1 : 0);
   }
   
   public static void main(String[] args)
   {
      System.out.println(ClassLayout.parseClass(PaddedVolatileLong.class).toPrintable());
   }
}
