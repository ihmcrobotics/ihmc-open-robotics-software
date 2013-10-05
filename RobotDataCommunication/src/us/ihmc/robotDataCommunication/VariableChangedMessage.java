package us.ihmc.robotDataCommunication;

public class VariableChangedMessage
{
   private int id = -1;
   private double val = -1;
   
   public VariableChangedMessage()
   {
   }
   
   public void setId(int id)
   {
      this.id = id;
   }
   
   public void setVal(double val)
   {
      this.val = val;
   }

   public int getId()
   {
      return id;
   }

   public double getVal()
   {
      return val;
   }
   
   public static class Builder implements us.ihmc.concurrent.Builder<VariableChangedMessage>
   {

      public VariableChangedMessage newInstance()
      {
         return new VariableChangedMessage();
      }
      
   }
   
   
}