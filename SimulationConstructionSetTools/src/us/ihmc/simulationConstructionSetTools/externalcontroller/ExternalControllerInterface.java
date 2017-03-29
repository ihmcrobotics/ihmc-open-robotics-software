package us.ihmc.simulationConstructionSetTools.externalcontroller;

interface ExternalControllerInterface
{
   public void update(double[] dataToBeUpdated);

   public void doControl();

   public double[] getTorques();

}
