package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

public class SolarPanelLinearPath
{
   private SolarPanelCleaningPose startPose;
   private SolarPanelCleaningPose endPose;
   private double motionStartTime;
   private double motionEndTime;
   
   public SolarPanelLinearPath(SolarPanelCleaningPose startPose, SolarPanelCleaningPose endPose, double motionStartTime, double motionEndTime)
   {
      this.startPose = startPose;
      this.endPose = endPose;
      this.motionStartTime = motionStartTime;
      this.motionEndTime = motionEndTime;
   }
   
   public SolarPanelCleaningPose getInterpolatedCleaningPose(double time)
   {
      SolarPanelCleaningPose cleaningPose = new SolarPanelCleaningPose(startPose);
      
      double scale;
      
      if(time < motionStartTime)
      {
         scale = 0;
      }
      else if(time >= motionStartTime && time < motionEndTime)
      {
         scale = (time - motionStartTime)/(motionEndTime - motionStartTime);
      }      
      else
      {
         scale = 1;
      }      
      
      double u = scale*(endPose.getU() - startPose.getU()) + startPose.getU();
      double v = scale*(endPose.getV() - startPose.getV()) + startPose.getV();
      double w = scale*(endPose.getW() - startPose.getW()) + startPose.getW();
      
      double zRotation = scale*(endPose.getZRotation() - startPose.getZRotation()) + startPose.getZRotation();
      
      cleaningPose.setUVWCoordinate(u, v, w);
      cleaningPose.setZRotation(zRotation);
      
      return cleaningPose;
   }
   
   public SolarPanelCleaningPose getStartPose()
   {
      return startPose;
   }
   
   public SolarPanelCleaningPose getEndPose()
   {
      return endPose;
   }
   
   public double getMotionStartTime()
   {
      return motionStartTime;
   }
   
   public double getMotionEndTime()
   {
      return motionEndTime;
   }
}
