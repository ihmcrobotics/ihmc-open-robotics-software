package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class NewInstantaneousCapturePointPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private BooleanYoVariable VISUALIZE = new BooleanYoVariable("icpPlannerVISUALIZE", registry);
   double ICP_CORNER_POINT_SIZE = 0.004;

   private final BooleanYoVariable atAStop = new BooleanYoVariable("icpPlannerAtAStop", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable("icpPlannerComeToStop", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable("icpPlannerIsInitialTransfer", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("icpPlannerDoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("icpPlannerSingleSupportTime", registry);
   private final DoubleYoVariable initialTransferDuration = new DoubleYoVariable("icpPlannerInitialTransferTime", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("icpPlannerInitialTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable("", registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint("DesiredCapturePointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint doubleSupportStartCapturePointPosition = new YoFramePoint("icpPlannerDoubleSupportStartICPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint doubleSupportStartCapturePointVelocity = new YoFramePoint("icpPlannerDoubleSupportStartICPVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector("DesiredCapturePointVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector("DesiredCapturePointAcceleration", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);
   private final ArrayList<YoFramePoint> constantCentersOfPressure = new ArrayList<YoFramePoint>(); 
   private final ArrayList<YoFramePoint> capturePointCornerPoints = new ArrayList<YoFramePoint>(); 

//   public NewInstantaneousCapturePointPlanner(int maxNumberFootstepsToConsider, WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry, 
//         YoGraphicsListRegistry yoGraphicsListRegistry)
   public NewInstantaneousCapturePointPlanner(int maxNumberFootstepsToConsider, double initialTransferTime, YoVariableRegistry parentRegistry, 
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if(yoGraphicsListRegistry == null)
      {
         VISUALIZE.set(false);
      }
      
//      this.initialTransferDuration.set(walkingControllerParameters.getDoubleSupportInitialTransferTime());
      this.initialTransferDuration.set(initialTransferTime);
      
      this.numberFootstepsToConsider.set(maxNumberFootstepsToConsider);
      this.atAStop.set(true);
      
      parentRegistry.addChild(this.registry);
      
      YoGraphicsList yoGraphicsList = new YoGraphicsList("ICPComputer");
      ArtifactList artifactList = new ArtifactList("ICPPlanner");
      
      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         String constantCoPName = "constantCoP" + i;
         YoFramePoint yoFramePoint = new YoFramePoint(constantCoPName, ReferenceFrame.getWorldFrame(), registry);
         constantCentersOfPressure.add(yoFramePoint);
         capturePointCornerPoints.add(yoFramePoint);

         if (VISUALIZE.getBooleanValue())
         {
            YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition(constantCoPName, yoFramePoint, ICP_CORNER_POINT_SIZE,
                  YoAppearance.Green(), GraphicType.SOLID_BALL);
            yoGraphicsList.add(constantFootCenterCentersOfPressureViz);
            artifactList.add(constantFootCenterCentersOfPressureViz.createArtifact());
         }
      }

      if (VISUALIZE.getBooleanValue())
      {
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }
   
   public void initializeDoubleSupport(double doubleSupportDuration, double singleSupportDuration, YoFramePoint currentDesiredCapturePointPosition, 
         YoFrameVector currentDesiredCapturePointVelocity, double omega0, double initialTime, ArrayList<YoFramePoint> footstepList)
   {
      this.omega0.set(omega0);
      this.singleSupportDuration.set(singleSupportDuration);
      this.initialTime.set(initialTime);
      
      if(atAStop.getBooleanValue())
      {
         //Investigate why this is needed in the current ICP planner.
         this.desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
         this.desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);
         
         this.doubleSupportDuration.set(this.initialTransferDuration.getDoubleValue());
         
         doubleSupportStartCapturePointPosition.set(currentDesiredCapturePointPosition);
         doubleSupportStartCapturePointVelocity.set(0.0, 0.0, 0.0);
      }
      else
      {
         this.doubleSupportDuration.set(doubleSupportDuration);
      }
      
      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints();
      
      if(atAStop.getBooleanValue())
      {
         atAStop.set(false);
      }
   }
   
   public void initializeSingleSupport(double doubleSupportDuration, double singleSupportDuration,
          double omega0, double initialTime, ArrayList<YoFramePoint> footstepList)
   {
      this.omega0.set(omega0);
      this.initialTime.set(initialTime);
      this.isInitialTransfer.set(false);
      comeToStop.set(footstepList.size() <= 2);
      atAStop.set(false);
      
      computeConstantCentersOfPressure(footstepList);
      computeCapturePointCornerPoints();
   }
   
   protected void computeConstantCentersOfPressure(ArrayList<YoFramePoint> footstepList)
   {
      comeToStop.set(footstepList.size() <= 2);
      
      if(atAStop.getBooleanValue())
      {
         this.doubleSupportDuration.set(initialTransferDuration.getDoubleValue());

         CapturePointTools.computeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet(constantCentersOfPressure, footstepList, numberFootstepsToConsider.getIntegerValue());
      }
      else
      {
         CapturePointTools.computeConstantCentersOfPressuresOnFeetWithEndBetweenFeet(constantCentersOfPressure, footstepList, numberFootstepsToConsider.getIntegerValue());   
      }
   }
   
   protected void computeCapturePointCornerPoints()
   {
      double steppingDuration = singleSupportDuration.getDoubleValue() + doubleSupportDuration.getDoubleValue();
      CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentersOfPressure, capturePointCornerPoints, 
            steppingDuration, omega0.getDoubleValue());
   }
   
   public void computeDesiredCapturePointPosition(double time)
   {
      YoFramePoint initialCapturePoint = capturePointCornerPoints.get(0);
      YoFramePoint initialCenterOfPressure = constantCentersOfPressure.get(0);
      
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), time, initialCapturePoint, 
            initialCenterOfPressure, desiredCapturePointPosition);
   }
   
   public void computeDesiredCapturePointVelocity(double time)
   {
      YoFramePoint initialCapturePoint = capturePointCornerPoints.get(0);
      YoFramePoint initialCenterOfPressure = constantCentersOfPressure.get(0);
      
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), time, initialCapturePoint, 
            initialCenterOfPressure, desiredCapturePointVelocity);
   }
   
   public void computeDesiredCapturePointAcceleration(double time)
   {
      YoFramePoint initialCapturePoint = capturePointCornerPoints.get(0);
      YoFramePoint initialCenterOfPressure = constantCentersOfPressure.get(0);
      
      CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), time, initialCapturePoint, initialCenterOfPressure, 
            desiredCapturePointAcceleration);
   }
   
   public void computeDesiredCornerPoints(ArrayList<YoFramePoint> constantCentersOfPressure, double stepTime, double omega0)
   {
      CapturePointTools.computeDesiredEndOfStepCapturePointLocations(constantCentersOfPressure, capturePointCornerPoints, 
            stepTime, omega0);
   }
   
   public YoFramePoint getDesiredCapturePointPosition()
   {
      return desiredCapturePointPosition;
   }
   
   public YoFramePoint getDoubleSupportStartCapturePointPosition()
   {
      return doubleSupportStartCapturePointPosition;
   }
   
   public ArrayList<YoFramePoint> getConstantCentersOfPressure()
   {
      return constantCentersOfPressure;
   }
   
   public ArrayList<YoFramePoint> getCapturePointCornerPoints()
   {
      return capturePointCornerPoints;
   }
   
   public YoFramePoint getDoubleSupportStartCapturePointVelocity()
   {
      return doubleSupportStartCapturePointVelocity;
   }
   
   public void packDesiredCapturePointIntoPoint3d(Point3d pointToPack)
   {
      desiredCapturePointPosition.get(pointToPack);
   }
   
   public YoFrameVector getDesiredCapturePointVelocity()
   {
      return desiredCapturePointVelocity;
   }
   
   public void packDesiredCapturePointVelocityIntoVector3d(Vector3d vectorToPack)
   {
      desiredCapturePointVelocity.get(vectorToPack);
   }
   
   public YoFrameVector getDesiredCapturePointAcceleration()
   {
      return desiredCapturePointAcceleration;
   }
   
   public void packDesiredCapturePointAccelerationIntoVector3d(Vector3d vectorToPack)
   {
      desiredCapturePointAcceleration.get(vectorToPack);
   }

   public void setDoubleSupportTime(double time)
   {
      this.doubleSupportDuration.set(time);
   }

   public void setSingleSupportTime(double time)
   {
      this.singleSupportDuration.set(time);
   }
   
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }
   
   public boolean isDone(double time)
   {
      return false;
   }
}
