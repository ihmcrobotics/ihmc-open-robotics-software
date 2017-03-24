package us.ihmc.simulationconstructionsettools.externalcontroller;

interface ExternalControllerInterface
{
   public void update(double[] dataToBeUpdated);

   public void doControl();

   public double[] getTorques();

}
