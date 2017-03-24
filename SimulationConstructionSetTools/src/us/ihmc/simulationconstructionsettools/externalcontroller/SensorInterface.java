package us.ihmc.simulationconstructionsettools.externalcontroller;

interface SensorInterface
{
   public String getYoVariableOrder();

   public int getNumberOfVariables();

   public double[] getMessageValues();

   public void setTau(double tau);



}
