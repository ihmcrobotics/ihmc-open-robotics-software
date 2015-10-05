package us.ihmc.controlFlow;


public class DataTypeFive
{
   private String string;
   
   public String getString()
   {
      return string;
   }
   
   public void setString(String string)
   {
      this.string = string;
   }
   
   public void set(DataTypeFive dataTypeFive)
   {
      this.string = dataTypeFive.string;
      
   }

}