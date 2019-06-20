package us.ihmc.atlas.parameters;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class AtlasSteppingParameters implements SteppingParameters
{
   private final AtlasJointMap jointMap;

   public AtlasSteppingParameters(AtlasJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public double getMinSwingHeightFromStanceFoot()
   {
      return 0.10 * jointMap.getModelScale();
   }
   
   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.30 * jointMap.getModelScale();
   }

   @Override
   public double getFootForwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootForwardForControl();
   }

   @Override
   public double getFootBackwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootBackForControl();
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25 * jointMap.getModelScale();
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.5 * jointMap.getModelScale(); // 0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.6 * jointMap.getModelScale(); // 0.5; //0.35;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15 * jointMap.getModelScale();
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6 * jointMap.getModelScale(); // 0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.6 * jointMap.getModelScale();
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.25 * jointMap.getModelScale();
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.2 * jointMap.getModelScale();
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return Math.PI / 4.0;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      return 0;
   }

   @Override
   public double getTurningStepWidth()
   {
      return 0.25;
   }

   @Override
   public double getMinAreaPercentForValidFootstep()
   {
      return 0.5;
   }

   @Override
   public double getDangerAreaPercentForValidFootstep()
   {
      return 0.75;
   }

   @Override
   public double getFootWidth()
   {
      return jointMap.getPhysicalProperties().getFootWidthForControl();
   }

   @Override
   public double getToeWidth()
   {
      return jointMap.getPhysicalProperties().getToeWidthForControl();
   }

   @Override
   public double getFootLength()
   {
      return jointMap.getPhysicalProperties().getFootLengthForControl();
   }

   @Override
   public double getActualFootWidth()
   {
      return jointMap.getPhysicalProperties().getActualFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return jointMap.getPhysicalProperties().getActualFootLength();
   }
}
