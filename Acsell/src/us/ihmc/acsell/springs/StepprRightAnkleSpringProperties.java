package us.ihmc.acsell.springs;

public class StepprRightAnkleSpringProperties implements HystereticSpringProperties {

   public StepprRightAnkleSpringProperties() {

   }
   
   @Override
   public double getLoadingSpringConstant() {
      return 0.0;
   }
   
   @Override
   public double getLinearSpringConstant() {
      return 300;
   }
   
   @Override
   public double getUnloadingSpringConstant() {
      return 0.0;
   }
      
   @Override
   public double getLoadingRestLength() {
      return 0.0;
   }
   
   @Override
   public double getLinearSpringRestLength() {
      return -0.18;
   }
   
   @Override
   public double getUnloadingRestLength() {
      return 0.0;
   }
   
   @Override
   public boolean isCompression() {
      return true;
   }

   @Override
   public boolean isExtension() {
      return false;
   }
   
   @Override
   public double getDirectionallity() {
      return 1.0; //Ankle Compresses at Negative Angles
   }
}
