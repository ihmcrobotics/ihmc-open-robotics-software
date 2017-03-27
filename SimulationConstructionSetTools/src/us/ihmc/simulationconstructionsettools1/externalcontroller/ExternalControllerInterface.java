package us.ihmc.simulationconstructionsettools1.externalcontroller;

interface ExternalControllerInterface
{
   public void update(double[] dataToBeUpdated);

   public void doControl();

   public double[] getTorques();

}
