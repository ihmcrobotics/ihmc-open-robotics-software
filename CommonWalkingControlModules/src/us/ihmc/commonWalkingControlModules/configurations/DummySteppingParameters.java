package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Created by agrabertilton on 2/20/15.
 */
public class DummySteppingParameters implements SteppingParameters
{
   public static final double footWidthForControl = 0.11; //0.12; // 0.08;   //0.124887;
   public static final double toeWidthForControl = 0.085; //0.095; // 0.07;   //0.05;   //
   public static final double footLengthForControl = 0.22; //0.255;
   public static final double footBackForControl = 0.09; // 0.06;   //0.082;    // 0.07;

   public static final double actualFootWidth = 0.138;
   public static final double actualFootLength = 0.26;
   public static final double footForward = footLengthForControl - footBackForControl;

   public DummySteppingParameters()
   {
   }

   @Override
   public double getFootForwardOffset()
   {
      return footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return footBackForControl;
   }

   public double getFootWidth()
   {
      return footWidthForControl;
   }

   @Override
   public double getToeWidth()
   {
      return toeWidthForControl;
   }

   @Override
   public double getFootLength()
   {
      return footLengthForControl;
   }

   @Override
   public double getActualFootWidth()
   {
      return actualFootWidth;
   }

   @Override
   public double getActualFootLength()
   {
      return actualFootLength;
   }

   @Override
   public double getFootstepArea()
   {
      return (getToeWidth() + getFootWidth()) * getFootLength() / 2.0;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.5; //0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.6; //0.5; //0.35;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.6;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.25;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.2;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.3;
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
   public double getMinStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6; //0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getMinSwingHeightFromStanceFoot()
   {
      return 0.1;
   }
}
