package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import us.ihmc.robotics.lidar.LidarScanParameters;

public class LidarTestParameters
{
   private double minRange;
   private double maxRange;
   private boolean rotateWall;
   private double rotationSpeed;
   private double lidarSweepStartAngle;
   private double lidarSweepEndAngle;
   private double lidarPitchMinAngle;
   private double lidarPitchMaxAngle;
   private int scansPerSweep;
   private int scanHeight;
   private double wallDistance;
   private double wallRotation;
   private double wallThickness;
   private double experimentHeight;
   private boolean placeLidar;
   private boolean placeWall;
   private boolean placeIhmcSphere;
   private boolean placeJmeSphere;
   private boolean showGpuPoints;
   private boolean showTracePoints;
   private boolean showScanRays;
   private boolean showWindow;
   private double lidarRotation;
   private int viewWidth;
   private int viewHeight;
   private boolean showSky;
   private double gpuVsTraceTolerance;
   private double lidarTestRotationAmount;
   private boolean printDebug;
   private boolean rotationLimitEnabled;
   private double currentRotation;
   private boolean isStopped;

   public LidarTestParameters()
   {
      this.minRange = 0.1;
      this.maxRange = 30.0;
      this.rotateWall = true;
      this.rotationSpeed = 2 * Math.PI / 100.0;
      this.lidarSweepStartAngle = -Math.PI / 4;
      this.lidarSweepEndAngle = Math.PI / 4;
      this.scansPerSweep = 720;
      this.lidarPitchMinAngle = 0.0;
      this.lidarPitchMaxAngle = 0.0;
      this.scanHeight = 1;
      this.wallDistance = 3.0;
      this.wallRotation = Math.PI / 6;
      this.wallThickness = 0.001;
      this.experimentHeight = 100.0;
      this.placeLidar = false;
      this.placeWall = true;
      this.placeIhmcSphere = false;
      this.placeJmeSphere = false;
      this.showWindow = true;
      this.lidarRotation = 0.0;
      this.viewWidth = 800;
      this.viewHeight = 600;
      this.showSky = true;
      this.gpuVsTraceTolerance = 1.0;
      this.lidarTestRotationAmount = 15 * (2 * Math.PI);
      this.printDebug = false;
      this.rotationLimitEnabled = false;
      this.currentRotation = 0.0;
      this.isStopped = false;
   }

   public void rotate(double timePerFrame)
   {
      currentRotation += rotationSpeed * 0.1;    // Ignores actual CPU time so test is independent of computer speed.
   }

   public boolean testIsOver()
   {
      if (rotationLimitEnabled && (currentRotation >= lidarTestRotationAmount))
      {
         isStopped = true;

         return true;
      }
      else
      {
         return false;
      }
   }
   
   public LidarScanParameters getLidarScanParameters()
   {
      return new LidarScanParameters(scansPerSweep, scanHeight, (float) lidarSweepStartAngle, (float) lidarSweepEndAngle, (float) lidarPitchMinAngle, (float) lidarPitchMaxAngle, 0, (float) minRange, (float) maxRange, 0, 0l);
   }

   public double getCurrentRotation()
   {
      return currentRotation;
   }

   public boolean isStopped()
   {
      return isStopped;
   }

   public double getMinRange()
   {
      return minRange;
   }

   public void setMinRange(double minRange)
   {
      this.minRange = minRange;
   }

   public double getMaxRange()
   {
      return maxRange;
   }

   public void setMaxRange(double maxRange)
   {
      this.maxRange = maxRange;
   }

   public boolean getRotateWall()
   {
      return rotateWall;
   }

   public void setRotateWall(boolean rotateWall)
   {
      this.rotateWall = rotateWall;
   }

   public double getRotationSpeed()
   {
      return rotationSpeed;
   }

   public void setRotationSpeed(double rotationSpeed)
   {
      this.rotationSpeed = rotationSpeed;
   }

   public double getLidarSweepStartAngle()
   {
      return lidarSweepStartAngle;
   }

   public void setLidarSweepStartAngle(double lidarSweepStartAngle)
   {
      this.lidarSweepStartAngle = lidarSweepStartAngle;
   }

   public double getLidarSweepEndAngle()
   {
      return lidarSweepEndAngle;
   }

   public void setLidarSweepEndAngle(double lidarSweepEndAngle)
   {
      this.lidarSweepEndAngle = lidarSweepEndAngle;
   }

   public int getScansPerSweep()
   {
      return scansPerSweep;
   }

   public void setScansPerSweep(int scansPerSweep)
   {
      this.scansPerSweep = scansPerSweep;
   }

   public double getWallDistance()
   {
      return wallDistance;
   }

   public void setWallDistance(double wallDistance)
   {
      this.wallDistance = wallDistance;
   }

   public double getWallRotation()
   {
      return wallRotation;
   }

   public void setWallRotation(double wallRotation)
   {
      this.wallRotation = wallRotation;
   }

   public double getWallThickness()
   {
      return wallThickness;
   }

   public void setWallThickness(double wallThickness)
   {
      this.wallThickness = wallThickness;
   }

   public double getExperimentHeight()
   {
      return experimentHeight;
   }

   public void setExperimentHeight(double experimentHeight)
   {
      this.experimentHeight = experimentHeight;
   }

   public boolean getPlaceWall()
   {
      return placeWall;
   }

   public void setPlaceWall(boolean placeWall)
   {
      this.placeWall = placeWall;
   }

   public boolean getPlaceIhmcSphere()
   {
      return placeIhmcSphere;
   }

   public void setPlaceIhmcSphere(boolean placeIhmcSphere)
   {
      this.placeIhmcSphere = placeIhmcSphere;
   }

   public boolean getPlaceJmeSphere()
   {
      return placeJmeSphere;
   }

   public void setPlaceJmeSphere(boolean placeJmeSphere)
   {
      this.placeJmeSphere = placeJmeSphere;
   }

   public boolean getShowGpuPoints()
   {
      return showGpuPoints;
   }

   public void setShowGpuPoints(boolean showGpuPoints)
   {
      this.showGpuPoints = showGpuPoints;
   }

   public boolean getShowTracePoints()
   {
      return showTracePoints;
   }

   public void setShowTracePoints(boolean showTracePoints)
   {
      this.showTracePoints = showTracePoints;
   }

   public boolean getShowScanRays()
   {
      return showScanRays;
   }

   public void setShowScanRays(boolean showScanRays)
   {
      this.showScanRays = showScanRays;
   }

   public boolean getPlaceLidar()
   {
      return placeLidar;
   }

   public void setPlaceLidar(boolean placeLidar)
   {
      this.placeLidar = placeLidar;
   }

   public boolean getShowWindow()
   {
      return showWindow;
   }

   public void setShowWindow(boolean showWindow)
   {
      this.showWindow = showWindow;
   }

   public double getLidarRotation()
   {
      return lidarRotation;
   }

   public void setLidarRotation(double lidarRotation)
   {
      this.lidarRotation = lidarRotation;
   }

   public int getViewWidth()
   {
      return viewWidth;
   }

   public void setViewWidth(int viewWidth)
   {
      this.viewWidth = viewWidth;
   }

   public int getViewHeight()
   {
      return viewHeight;
   }

   public void setViewHeight(int viewHeight)
   {
      this.viewHeight = viewHeight;
   }

   public boolean getShowSky()
   {
      return showSky;
   }

   public void setShowSky(boolean showSky)
   {
      this.showSky = showSky;
   }

   public double getGpuVsTraceTolerance()
   {
      return gpuVsTraceTolerance;
   }

   public void setGpuVsTraceTolerance(double gpuVsTraceTolerance)
   {
      this.gpuVsTraceTolerance = gpuVsTraceTolerance;
   }

   public double getLidarTestRotationAmount()
   {
      return lidarTestRotationAmount;
   }

   public void setLidarTestRotationAmount(double lidarTestRotationAmount)
   {
      this.lidarTestRotationAmount = lidarTestRotationAmount;
   }

   public boolean getPrintDebug()
   {
      return printDebug;
   }

   public void setPrintDebug(boolean printDebug)
   {
      this.printDebug = printDebug;
   }

   public boolean isRotationLimitEnabled()
   {
      return rotationLimitEnabled;
   }

   public void setRotationLimitEnabled(boolean rotationLimitEnabled)
   {
      this.rotationLimitEnabled = rotationLimitEnabled;
   }

   public double getLidarPitchMinAngle()
   {
      return lidarPitchMinAngle;
   }

   public void setLidarPitchMinAngle(double lidarPitchMinAngle)
   {
      this.lidarPitchMinAngle = lidarPitchMinAngle;
   }

   public double getLidarPitchMaxAngle()
   {
      return lidarPitchMaxAngle;
   }

   public void setLidarPitchMaxAngle(double lidarPitchMaxAngle)
   {
      this.lidarPitchMaxAngle = lidarPitchMaxAngle;
   }

   public int getScanHeight()
   {
      return scanHeight;
   }

   public void setScanHeight(int scanHeight)
   {
      this.scanHeight = scanHeight;
   }
}
