package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;


public class DoubleSupportFootCenterToToeICPComputer
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = false;

   private final double isDoneThreshold = -1e-4;

   private final int maxNumberOfConsideredFootsteps;
   private final double doubleSupportFirstStepFraction;
   private final double singleSupportToePercentage;
   private int numberOfCornerPoints;

   private final ArrayList<YoFramePoint> constantFootCenterCentersOfPressure = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> constantToeCentersOfPressure = new ArrayList<YoFramePoint>();

   private final ArrayList<YoFramePoint> footCenterICPCornerFramePoints = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> toeICPCornerFramePoints = new ArrayList<YoFramePoint>();

   private final DoubleYoVariable icpForwardFromCenter = new DoubleYoVariable("icpForwardFromCenter", registry);
   private final DoubleYoVariable icpInFromCenter = new DoubleYoVariable("icpInFromCenter", registry);

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("icpPlannerHasBeenInitialized", registry);

   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable("icpPlannerIsDoubleSupport", registry);
   private final DoubleYoVariable timeInState = new DoubleYoVariable("icpPlannerTimeInState", registry);
   protected final DoubleYoVariable estimatedTimeRemainingForState = new DoubleYoVariable("icpPlannerEstiTimeRemaining", registry);

   private final DoubleYoVariable initialTime = new DoubleYoVariable("icpPlannerInitialTime", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable("icpPlannerComeToStop", registry);
   private final BooleanYoVariable atAStop = new BooleanYoVariable("icpPlannerAtAStop", registry);
   private final BooleanYoVariable footCenterToToeICPComputerIsDone = new BooleanYoVariable("footCenterToToeICPComputerIsDone", registry);

   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable("icpPlannerIsInitialTransfer", registry);

   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("icpPlannerSingleSupportDuration", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("icpPlannerDoubleSupportDuration", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable("icpPlannerDoubleSupportInitialTransferDuration", registry);

   private final Point3d desiredICPPosition = new Point3d();
   private final Vector3d desiredICPVelocity = new Vector3d();
   private final Vector3d desiredICPAcceleration = new Vector3d();
   private final Point3d desiredECMP = new Point3d();

   private final Point3d smoothDesiredICPPosition = new Point3d();
   private final Vector3d smoothDesiredICPVelocity = new Vector3d();
   private final Vector3d smoothDesiredICPAcceleration = new Vector3d();
   private final Point3d smoothDesiredECMP = new Point3d();

   private final YoFramePoint desiredICPPositionFramePoint = new YoFramePoint("desiredICPPositionC", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredICPVelocityFrameVector = new YoFrameVector("desiredICPVelocityC", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredICPAccelerationFrameVector = new YoFrameVector("desiredICPAccelerationC", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredECMPFramePoint = new YoFramePoint("desiredECMPPositionC", ReferenceFrame.getWorldFrame(), registry);

   private final Point3d comPositionVector = new Point3d();
   private final Vector3d comVelocityVector = new Vector3d();

   private final YoFramePoint desiredComPositionFramePoint = new YoFramePoint("desiredComPositionC", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredComVelocityFrameVector = new YoFrameVector("desiredComVelocityC", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable desiredICPvelAbsolute = new DoubleYoVariable("desiredICPvelAbsolute", registry);
   private final DoubleYoVariable desiredCOMvelAbsolute = new DoubleYoVariable("desiredCOMvelAbsolute", registry);

   private final DoubleYoVariable omega0 = new DoubleYoVariable("icpPlannerOmega0", registry);
   private final double dt;

   private final double maxFrontalToeOffset = 0.1;
   private final BooleanYoVariable doHeelToToeTransfer = new BooleanYoVariable("doHeelToToeTransfer", registry);

   // TODO: Finish YoVariablizing these to make rewindable and visualizable.

   private Point3d constantCenterOfPressure;
   private Point3d upcomingCornerPoint = new Point3d();

   private Point3d singleSupportStartICP = new Point3d();
   private Vector3d singleSupportStartICPVelocity = new Vector3d();
   private Point3d singleSupportEndICP = new Point3d();

   private Vector3d singleSupportEndICPVelocity = new Vector3d();

   private final Point3d doubleSupportEndICP = new Point3d();
   private final Vector3d doubleSupportEndICPVelocity = new Vector3d();

   private final VelocityConstrainedPositionTrajectoryGenerator doubleSupportPolynomialTrajectory;
   
   public DoubleSupportFootCenterToToeICPComputer(double dt, double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      
      this.dt = dt;

      int numberOfCoefficientsForDoubleSupport = 5; // 5 will set acceleration at end to 0.0
      doubleSupportPolynomialTrajectory = new VelocityConstrainedPositionTrajectoryGenerator("icpDouble", numberOfCoefficientsForDoubleSupport,
            ReferenceFrame.getWorldFrame(), registry);

      //Don't set setDoHeelToToeTransfer to true unless you make the VRC Task 2 work with it on first, especially the mud!      
      this.doubleSupportInitialTransferDuration.set(1.0);

      this.setDoHeelToToeTransfer(false);
      //      doHeelToToeTransfer.set(true);

      this.singleSupportToePercentage = 0.5;
      comPositionVector.set(0.0, 0.0, 0.0);
      comVelocityVector.set(0.0, 0.0, 0.0);

      hasBeenInitialized.set(false);
      isInitialTransfer.set(true);

      parentRegistry.addChild(registry);

      this.atAStop.set(true);

      this.doubleSupportFirstStepFraction = doubleSupportFirstStepFraction; 
      
      this.maxNumberOfConsideredFootsteps = maxNumberOfConsideredFootsteps;

      this.numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;

      YoGraphicsList yoGraphicsList = new YoGraphicsList("ICPComputer");
      ArtifactList artifactList = new ArtifactList("ICPComputer");
      double icpCornerPointSize = 0.004;

      for (int i = 0; i < maxNumberOfConsideredFootsteps - 1; i++)
      {
         String icpCornerPointsName = "icpCornerPoints" + i;
         YoFramePoint yoFramePoint = new YoFramePoint(icpCornerPointsName, ReferenceFrame.getWorldFrame(), registry);
         footCenterICPCornerFramePoints.add(yoFramePoint);

         if (VISUALIZE)
         {
            YoGraphicPosition icpCornerPointViz = new YoGraphicPosition(icpCornerPointsName, yoFramePoint, icpCornerPointSize, YoAppearance.Black(),
                  GraphicType.SOLID_BALL);
            yoGraphicsList.add(icpCornerPointViz);
            artifactList.add(icpCornerPointViz.createArtifact());
         }

         if (doHeelToToeTransfer.getBooleanValue())
         {
            String toeICPCornerPointsName = "toeICPCornerPointsName" + i;
            YoFramePoint yoToeFramePoint = new YoFramePoint(toeICPCornerPointsName, ReferenceFrame.getWorldFrame(), registry);
            toeICPCornerFramePoints.add(yoToeFramePoint);

            if (VISUALIZE)
            {
               YoGraphicPosition toeICPCornerPointViz = new YoGraphicPosition(toeICPCornerPointsName, yoToeFramePoint, icpCornerPointSize,
                     YoAppearance.Chocolate(), GraphicType.SOLID_BALL);
               yoGraphicsList.add(toeICPCornerPointViz);
               artifactList.add(toeICPCornerPointViz.createArtifact());
            }
         }
      }

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         String constantCoPName = "constantCoP" + i;
         YoFramePoint yoFramePoint = new YoFramePoint(constantCoPName, ReferenceFrame.getWorldFrame(), registry);
         constantFootCenterCentersOfPressure.add(yoFramePoint);

         if (VISUALIZE)
         {
            YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition(constantCoPName, yoFramePoint, icpCornerPointSize,
                  YoAppearance.Green(), GraphicType.SOLID_BALL);
            yoGraphicsList.add(constantFootCenterCentersOfPressureViz);
            artifactList.add(constantFootCenterCentersOfPressureViz.createArtifact());
         }

         if (doHeelToToeTransfer.getBooleanValue())
         {
            String constantToeCoPName = "constantToeCoP" + i;
            YoFramePoint toeFramePoint = new YoFramePoint(constantToeCoPName, ReferenceFrame.getWorldFrame(), registry);
            constantToeCentersOfPressure.add(toeFramePoint);

            if (VISUALIZE)
            {
               YoGraphicPosition constantToeCentersOfPressureViz = new YoGraphicPosition(constantToeCoPName, toeFramePoint, icpCornerPointSize,
                     YoAppearance.Blue(), GraphicType.SOLID_BALL);
               yoGraphicsList.add(constantToeCentersOfPressureViz);
               artifactList.add(constantToeCentersOfPressureViz.createArtifact());
            }
         }
      }

      if (VISUALIZE)
      {
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

   }

   public void initializeSingleSupport(ArrayList<FramePoint> footLocationList, ArrayList<ReferenceFrame> soleFrameList, double singleSupportDuration,
         double doubleSupportDuration, double omega0, double initialTime)
   {
      this.omega0.set(omega0);

      isInitialTransfer.set(false);
      comeToStop.set(footLocationList.size() <= 2);
      atAStop.set(false);

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);

      this.isDoubleSupport.set(false);

      this.initialTime.set(initialTime);

      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      double durationForCornerPoints = getDurationForCornerPoints(singleSupportDuration, steppingDuration);
      
      if (doHeelToToeTransfer.getBooleanValue())
      {
         double toeToFootCenterShiftDuration = steppingDuration * singleSupportToePercentage;
         double footCenterToToeShiftDuration = steppingDuration * (1 - singleSupportToePercentage);
         double toeShiftToDoubleSupportDuration = toeToFootCenterShiftDuration - doubleSupportDuration * doubleSupportFirstStepFraction;
         double secondDoubleSupportFractionDuration = doubleSupportDuration * (1 - doubleSupportFirstStepFraction);

         NewDoubleSupportICPComputer.computeConstantCentersOfPressureAndCornerPointsForFootCenterAndToe(constantFootCenterCentersOfPressure,
               constantToeCentersOfPressure, footCenterICPCornerFramePoints, toeICPCornerFramePoints, maxFrontalToeOffset, footLocationList, soleFrameList,
               maxNumberOfConsideredFootsteps, isInitialTransfer.getBooleanValue(), this.omega0.getDoubleValue(), toeToFootCenterShiftDuration,
               footCenterToToeShiftDuration);

         upcomingCornerPoint = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();

         JojosICPutilities.extrapolateDCMposAndVel(singleSupportStartICP, singleSupportStartICPVelocity, constantFootCenterCentersOfPressure.get(0)
               .getPoint3dCopy(), secondDoubleSupportFractionDuration, omega0, footCenterICPCornerFramePoints.get(0).getPoint3dCopy());

         JojosICPutilities.extrapolateDCMposAndVel(singleSupportEndICP, singleSupportEndICPVelocity, constantToeCentersOfPressure.get(0).getPoint3dCopy(),
               toeShiftToDoubleSupportDuration, omega0, toeICPCornerFramePoints.get(0).getPoint3dCopy());

         doubleSupportPolynomialTrajectory.setTrajectoryParameters(singleSupportDuration, singleSupportStartICP, singleSupportStartICPVelocity, singleSupportEndICP, singleSupportEndICPVelocity);
         doubleSupportPolynomialTrajectory.initialize();
      }
      else
      {
         NewDoubleSupportICPComputer.computeConstantCentersOfPressure(constantFootCenterCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps,
               isInitialTransfer.getBooleanValue());
         ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantFootCenterCentersOfPressure);

         Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds,
               durationForCornerPoints, this.omega0.getDoubleValue());

         for (int i = 0; i < icpCornerPoints.length; i++)
         {
            footCenterICPCornerFramePoints.get(i).set(icpCornerPoints[i]);
         }

         Point3d cornerPoint0 = icpCornerPoints[0];
         Point3d supportFoot = constantFootCenterCentersOfPressure.get(0).getPoint3dCopy();

         singleSupportStartICP = NewDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
               doubleSupportFirstStepFraction, this.omega0.getDoubleValue());
         
         upcomingCornerPoint = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();
      }
   }

   public void initializeDoubleSupportInitialTransfer(ArrayList<FramePoint> footLocationList, ArrayList<ReferenceFrame> soleFrameList,
         Point3d initialICPPosition, double singleSupportDuration, double doubleSupportDuration, double doubleSupportInitialTransferDuration, double omega0,
         double initialTime)
   {
      this.omega0.set(omega0);

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.doubleSupportInitialTransferDuration.set(doubleSupportInitialTransferDuration);

      this.isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      double toeToFootCenterShiftDuration = steppingDuration * singleSupportToePercentage;
      double footCenterToToeShiftDuration = steppingDuration * (1 - singleSupportToePercentage);
      double durationForCornerPoints = getDurationForCornerPoints(singleSupportDuration, steppingDuration);
      
      if (doHeelToToeTransfer.getBooleanValue())
      {
         NewDoubleSupportICPComputer.computeConstantCentersOfPressureAndCornerPointsForFootCenterAndToe(constantFootCenterCentersOfPressure,
               constantToeCentersOfPressure, footCenterICPCornerFramePoints, toeICPCornerFramePoints, maxFrontalToeOffset, footLocationList, soleFrameList,
               maxNumberOfConsideredFootsteps, isInitialTransfer.getBooleanValue(), this.omega0.getDoubleValue(), toeToFootCenterShiftDuration,
               footCenterToToeShiftDuration);
      }
      else
      {
         NewDoubleSupportICPComputer.computeConstantCentersOfPressure(constantFootCenterCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps,
               isInitialTransfer.getBooleanValue() || comeToStop.getBooleanValue());

         int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;

         ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantFootCenterCentersOfPressure);

         Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds,
               durationForCornerPoints, this.omega0.getDoubleValue());

         for (int i = 0; i < icpCornerPoints.length; i++)
         {
            footCenterICPCornerFramePoints.get(i).set(icpCornerPoints[i]);
         }
      }

      upcomingCornerPoint = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();

      singleSupportEndICP.set(initialICPPosition);
      singleSupportEndICPVelocity.set(0.0, 0.0, 0.0);

      if (doHeelToToeTransfer.getBooleanValue())
      {
         double secondDoubleSupportFractionDuration = doubleSupportDuration * (1 - doubleSupportFirstStepFraction);

         JojosICPutilities.extrapolateDCMposAndVel(doubleSupportEndICP, doubleSupportEndICPVelocity, constantFootCenterCentersOfPressure.get(1)
               .getPoint3dCopy(), secondDoubleSupportFractionDuration, omega0, footCenterICPCornerFramePoints.get(1).getPoint3dCopy());

      }
      else
      {
         if (comeToStop.getBooleanValue())
         {
            // will be midpoint of feet from computeConstantCentersOfPressure
            constantFootCenterCentersOfPressure.get(0).get(doubleSupportEndICP);
            doubleSupportEndICPVelocity.set(0, 0, 0);
         }
         else
         {
            Point3d cornerPoint1 = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();
            Point3d transferToFoot = constantFootCenterCentersOfPressure.get(1).getPoint3dCopy();

            NewDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
                  doubleSupportDuration, doubleSupportFirstStepFraction, this.omega0.getDoubleValue());
         }
      }

      if (isInitialTransfer.getBooleanValue())
      {
         doubleSupportPolynomialTrajectory.setTrajectoryParameters(doubleSupportInitialTransferDuration, singleSupportEndICP, singleSupportEndICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);
      }
      else
      {
         doubleSupportPolynomialTrajectory.setTrajectoryParameters(doubleSupportDuration, singleSupportEndICP, singleSupportEndICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);
      }
      
      doubleSupportPolynomialTrajectory.initialize();
      atAStop.set(comeToStop.getBooleanValue());
   }

   private double getDurationForCornerPoints(double singleSupportDuration, double steppingDuration)
   {
      return steppingDuration;
   }

   public void initializeDoubleSupport(ArrayList<FramePoint> footLocationList, ArrayList<ReferenceFrame> soleFrameList, double singleSupportDuration,
         double doubleSupportDuration, double omega0, Point3d currentDesiredICP, Vector3d currentDesiredICPVelocity, double initialTime)
   {
      comeToStop.set(footLocationList.size() <= 2);

      if (atAStop.getBooleanValue() || comeToStop.getBooleanValue())
      {
         double doubleSupportInitialTransferDuration = this.doubleSupportInitialTransferDuration.getDoubleValue();

         // ALesman: this no longer happens because we now always want to initialize from the existing desired icp
         if (currentDesiredICP == null || currentDesiredICPVelocity == null)
         {
            this.computeICPPositionVelocityAcceleration(desiredICPPosition, desiredICPVelocity, desiredICPAcceleration, desiredECMP, initialTime);
         }
         else
         {
            desiredICPPosition.set(currentDesiredICP);
            desiredICPVelocity.set(currentDesiredICPVelocity);
            hasBeenInitialized.set(true);
         }

         initializeDoubleSupportInitialTransfer(footLocationList, soleFrameList, desiredICPPosition, singleSupportDuration, doubleSupportDuration,
               doubleSupportInitialTransferDuration, this.omega0.getDoubleValue(), initialTime);
      }
      else
      {
         initializeDoubleSupportLocal(footLocationList, singleSupportDuration, doubleSupportDuration, this.omega0.getDoubleValue(), initialTime);
      }
   }

   private void initializeDoubleSupportLocal(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
         double initialTime)
   {
      this.omega0.set(omega0);

      isInitialTransfer.set(false);

      NewDoubleSupportICPComputer.computeConstantCentersOfPressure(constantFootCenterCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps,
            isInitialTransfer.getBooleanValue());

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);

      this.isDoubleSupport.set(true);

      this.initialTime.set(initialTime);

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      double durationForCornerPoints = getDurationForCornerPoints(singleSupportDuration, steppingDuration);

      if (doHeelToToeTransfer.getBooleanValue())
      {
         double toeToFootCenterShiftDuration = steppingDuration * singleSupportToePercentage;
         double toeShiftToDoubleSupportDuration = toeToFootCenterShiftDuration - doubleSupportDuration * doubleSupportFirstStepFraction;
         double secondDoubleSupportFractionDuration = doubleSupportDuration * (1 - doubleSupportFirstStepFraction);

         JojosICPutilities.extrapolateDCMposAndVel(singleSupportEndICP, singleSupportEndICPVelocity, constantToeCentersOfPressure.get(0).getPoint3dCopy(),
               toeShiftToDoubleSupportDuration, omega0, toeICPCornerFramePoints.get(0).getPoint3dCopy());

         JojosICPutilities.extrapolateDCMposAndVel(doubleSupportEndICP, doubleSupportEndICPVelocity, constantFootCenterCentersOfPressure.get(1)
               .getPoint3dCopy(), secondDoubleSupportFractionDuration, omega0, footCenterICPCornerFramePoints.get(1).getPoint3dCopy());

      }
      else
      {
         ArrayList<Point3d> constantCentersOfPressurePoint3ds = convertToListOfPoint3ds(constantFootCenterCentersOfPressure);

         Point3d[] icpCornerPoints = NewDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressurePoint3ds,
               durationForCornerPoints, this.omega0.getDoubleValue());

         Point3d cornerPoint0 = icpCornerPoints[0];
         Point3d cornerPoint1 = icpCornerPoints[1];
         Point3d transferFromFoot = constantFootCenterCentersOfPressure.get(0).getPoint3dCopy();
         Point3d transferToFoot = constantFootCenterCentersOfPressure.get(1).getPoint3dCopy();

         NewDoubleSupportICPComputer.computeSingleSupportEndICPAndVelocity(singleSupportEndICP, singleSupportEndICPVelocity, transferFromFoot, cornerPoint0,
               doubleSupportDuration, doubleSupportFirstStepFraction, singleSupportDuration, this.omega0.getDoubleValue());
         NewDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
               doubleSupportDuration, doubleSupportFirstStepFraction, this.omega0.getDoubleValue());
      }

      upcomingCornerPoint = footCenterICPCornerFramePoints.get(1).getPoint3dCopy();

      doubleSupportPolynomialTrajectory.setTrajectoryParameters(doubleSupportDuration, singleSupportEndICP, singleSupportEndICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);
      doubleSupportPolynomialTrajectory.initialize();
   }

   public Point3d getDoubleSupportStartICP()
   {
      return singleSupportEndICP;
   }

   public Point3d getDoubleSupportEndICP()
   {
      return doubleSupportEndICP;
   }

   public Vector3d getDoubleSupportStartICPVelocity()
   {
      return singleSupportEndICPVelocity;
   }

   public Vector3d getDoubleSupportEndICPVelocity()
   {
      return doubleSupportEndICPVelocity;
   }

   public void computeICPPositionVelocityAcceleration(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Vector3d icpAccelToPack, Point3d ecmpToPack,
         double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      if (isDoubleSupport.getBooleanValue())
         getDesiredICPPosVelAccDoubleSupport(icpPositionToPack, icpVelocityToPack, icpAccelToPack, ecmpToPack, timeInState.getDoubleValue());
      else
         getDesiredICPPosVelAccSingleSupport(icpPositionToPack, icpVelocityToPack, icpAccelToPack, ecmpToPack, timeInState.getDoubleValue());

      desiredICPPosition.set(icpPositionToPack);
      desiredICPVelocity.set(icpVelocityToPack);
      desiredICPAcceleration.set(icpAccelToPack);
      desiredECMP.set(ecmpToPack);

      desiredICPPositionFramePoint.set(icpPositionToPack);
      desiredICPVelocityFrameVector.set(icpVelocityToPack);
      desiredICPAccelerationFrameVector.set(icpAccelToPack);
      desiredECMPFramePoint.set(ecmpToPack);

      computeDesiredICPAndCOMVelocityAbsolutes(icpVelocityToPack, dt); // NOTE: if simulation-tick is other than 5ms, change that value!
   }

   public void computeDesiredICPAndCOMVelocityAbsolutes(Vector3d icpVelocityToPack, double simDT)
   {
      desiredICPvelAbsolute
            .set(Math.sqrt(Math.pow(icpVelocityToPack.getX(), 2) + Math.pow(icpVelocityToPack.getY(), 2) + Math.pow(icpVelocityToPack.getZ(), 2)));

      JojosICPutilities.discreteIntegrateCoMAndGetCoMVelocity(simDT, omega0.getDoubleValue(), desiredICPPosition, comPositionVector, comVelocityVector);

      desiredComPositionFramePoint.set(comPositionVector);
      desiredComVelocityFrameVector.set(comVelocityVector);

      desiredCOMvelAbsolute.set(Math.sqrt(Math.pow(desiredComVelocityFrameVector.getX(), 2) + Math.pow(desiredComVelocityFrameVector.getY(), 2)
            + Math.pow(desiredComVelocityFrameVector.getZ(), 2)));

   }

   private void getDesiredICPPosVelAccSingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Vector3d icpAccelerationToPack, Point3d ecmpToPack,
         double timeInState)
   {
      if (doHeelToToeTransfer.getBooleanValue())
      {
         computePolynomialDesiredICPPosVelAccAndECMP(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack, ecmpToPack, timeInState); //, singleSupportParameterMatrix);
      }
      else
      {
         constantCenterOfPressure = constantFootCenterCentersOfPressure.get(0).getPoint3dCopy();

         JojosICPutilities.extrapolateDCMPosVelAcc(icpPositionToPack, icpVelocityToPack, icpAccelerationToPack, constantCenterOfPressure, timeInState,
               omega0.getDoubleValue(), singleSupportStartICP);

         ecmpToPack.set(constantCenterOfPressure);
      }
   }

   public double getTimeInState(double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);

      return timeInState.getDoubleValue();
   }

   private void getDesiredICPPosVelAccDoubleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Vector3d icpAccelToPack, Point3d ecmpToPack,
         double timeInState)
   {
      computePolynomialDesiredICPPosVelAccAndECMP(icpPositionToPack, icpVelocityToPack, icpAccelToPack, ecmpToPack, timeInState); //, doubleSupportParameterMatrix);
   }

   private final Vector3d tempVector = new Vector3d();

   private void computePolynomialDesiredICPPosVelAccAndECMP(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Vector3d icpAccelToPack, Point3d ecmpToPack,
         double timeInState) //, DenseMatrix64F parameterMatrix)
   {
      doubleSupportPolynomialTrajectory.compute(timeInState);
      doubleSupportPolynomialTrajectory.get(icpPositionToPack);
      doubleSupportPolynomialTrajectory.packVelocity(icpVelocityToPack);
      doubleSupportPolynomialTrajectory.packAcceleration(icpAccelToPack);

      tempVector.set(icpVelocityToPack);
      tempVector.scale(-1.0 / omega0.getDoubleValue());
      ecmpToPack.set(icpPositionToPack);
      ecmpToPack.add(tempVector);
   }

   private final ArrayList<FramePoint> footLocationList = new ArrayList<FramePoint>();
   private final ArrayList<ReferenceFrame> soleFrameList = new ArrayList<ReferenceFrame>();

   public void initializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      footLocationList.clear();
      soleFrameList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, soleFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      //      double omega0 = transferToAndNextFootstepsData.getW0();
      this.omega0.set(transferToAndNextFootstepsData.getW0());

      initializeSingleSupport(footLocationList, soleFrameList, singleSupportDuration, doubleSupportDuration, omega0.getDoubleValue(), initialTime);
   }

   public void reInitializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double currentTime)
   {
      footLocationList.clear();
      soleFrameList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, soleFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      //      double omega0 = transferToAndNextFootstepsData.getW0();
      this.omega0.set(transferToAndNextFootstepsData.getW0());

      initializeSingleSupport(footLocationList, soleFrameList, singleSupportDuration, doubleSupportDuration, omega0.getDoubleValue(),
            initialTime.getDoubleValue());
   }

   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point3d initialICPPosition,
         double initialTime)
   {
      footLocationList.clear();
      soleFrameList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, soleFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      double doubleSupportInitialTransferDuration = transferToAndNextFootstepsData.getDoubleSupportInitialTransferDuration();
      //      double omega0 = transferToAndNextFootstepsData.getW0();
      this.omega0.set(transferToAndNextFootstepsData.getW0());

      initializeDoubleSupportInitialTransfer(footLocationList, soleFrameList, initialICPPosition, singleSupportDuration, doubleSupportDuration,
            doubleSupportInitialTransferDuration, omega0.getDoubleValue(), initialTime);
   }

   public void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime)
   {
      footLocationList.clear();
      soleFrameList.clear();

      transferToAndNextFootstepsData.getFootLocationList(footLocationList, soleFrameList, icpForwardFromCenter.getDoubleValue(), icpInFromCenter.getDoubleValue());
      double singleSupportDuration = transferToAndNextFootstepsData.getSingleSupportDuration();
      double doubleSupportDuration = transferToAndNextFootstepsData.getDoubleSupportDuration();
      this.omega0.set(transferToAndNextFootstepsData.getW0());

      initializeDoubleSupport(footLocationList, soleFrameList, singleSupportDuration, doubleSupportDuration, omega0.getDoubleValue(),
            transferToAndNextFootstepsData.getCurrentDesiredICP(), transferToAndNextFootstepsData.getCurrentDesiredICPVelocity(), initialTime);
   }

   private void computeTimeInStateAndEstimatedTimeRemaining(double time)
   {
      timeInState.set(time - initialTime.getDoubleValue());

      if (isDoubleSupport.getBooleanValue())
      {
         if (isInitialTransfer.getBooleanValue())
         {
            estimatedTimeRemainingForState.set(doubleSupportInitialTransferDuration.getDoubleValue() - timeInState.getDoubleValue());
         }

         else
         {
            estimatedTimeRemainingForState.set(doubleSupportDuration.getDoubleValue() - timeInState.getDoubleValue());
         }
      }
      else
         estimatedTimeRemainingForState.set(singleSupportDuration.getDoubleValue() - timeInState.getDoubleValue());
   }

   public boolean isDone(double time)
   {
      computeTimeInStateAndEstimatedTimeRemaining(time);
      footCenterToToeICPComputerIsDone.set(estimatedTimeRemainingForState.getDoubleValue() <= isDoneThreshold);

      return (footCenterToToeICPComputerIsDone.getBooleanValue());
   }

   public double getEstimatedTimeRemainingForState(double time)
   {
      return estimatedTimeRemainingForState.getDoubleValue();
   }

   public boolean isPerformingICPDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   public void reset(double time)
   {
      ///////////////////// Deleting all but the first two HumanoidReferenceFrames in soleFrameList
      ArrayList<ReferenceFrame> tempFrameList = new ArrayList<ReferenceFrame>();

      for (int i = 0; i < 2; i++)
      {
         tempFrameList.add(soleFrameList.get(i));
      }

      soleFrameList.clear();

      for (int i = 0; i < 2; i++)
      {
         soleFrameList.add(tempFrameList.get(i));
      }
      /////////////////////////////

      //footLocationList.clear();
      //footLocationList.add(constantFootCenterCentersOfPressure.get(0).getFramePointCopy());
      //footLocationList.add(constantFootCenterCentersOfPressure.get(1).getFramePointCopy());

      Point3d initialICPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();
      Vector3d initialICPAcceleration = new Vector3d();
      Point3d InitialECMPPosition = new Point3d();

      computeICPPositionVelocityAcceleration(initialICPPosition, initialICPVelocity, initialICPAcceleration, InitialECMPPosition, time);

      comeToStop.set(true);
      initializeDoubleSupportInitialTransfer(footLocationList, soleFrameList, initialICPPosition, singleSupportDuration.getDoubleValue(),
            doubleSupportDuration.getDoubleValue(), doubleSupportInitialTransferDuration.getDoubleValue(), omega0.getDoubleValue(), time);
   }

   public static ArrayList<Point3d> convertToListOfPoint3ds(ArrayList<YoFramePoint> yoFramePoints)
   {
      ArrayList<Point3d> ret = new ArrayList<Point3d>(yoFramePoints.size());

      for (YoFramePoint yoFramePoint : yoFramePoints)
      {
         ret.add(yoFramePoint.getPoint3dCopy());
      }

      return ret;
   }

   public List<YoFramePoint> getConstantCentersOfPressure()
   {
      return constantFootCenterCentersOfPressure;
   }
   
   public FramePoint2d getSingleSupportStartICP()
   {
      FramePoint2d tmpPoint = new FramePoint2d();
      tmpPoint.set(singleSupportStartICP.getX(), singleSupportStartICP.getY());
      return tmpPoint;
   }

   public List<YoFramePoint> getFootCenterICPCornerPoints()
   {
      return footCenterICPCornerFramePoints;
   }

   public List<YoFramePoint> getToeICPCornerPoints()
   {
      return toeICPCornerFramePoints;
   }

   public Point3d getUpcomingCornerPoint()
   {
      return upcomingCornerPoint;
   }

   public boolean getDoHeelToToeTransfer()
   {
      return doHeelToToeTransfer.getBooleanValue();
   }

   public void setDoHeelToToeTransfer(boolean doHeelToToeTransfer)
   {
      this.doHeelToToeTransfer.set(doHeelToToeTransfer);
   }
   
   public void setICPInFromCenter(double icpInFromCenter)
   {
      this.icpInFromCenter.set(icpInFromCenter);
   }
}
