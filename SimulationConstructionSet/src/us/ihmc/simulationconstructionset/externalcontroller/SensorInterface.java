package us.ihmc.simulationconstructionset.externalcontroller;

interface SensorInterface
{
   public String getYoVariableOrder();

   public int getNumberOfVariables();

   public double[] getMessageValues();

   public void setTau(double tau);



}
