package us.ihmc.simulationconstructionset.externalcontroller;

interface ExternalControllerInterface
{
   public void update(double[] dataToBeUpdated);

   public void doControl();

   public double[] getTorques();

}
