package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;


import java.awt.Color;
import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class SCSDoubleSupportICPTesterController5 implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);


   int numberOfConsideredFootstepLocations = 4;

   int numberOfStepsInStepList = 5;
   private final ArrayList<YoFramePoint> footStepLocationsFramePoints = new ArrayList<YoFramePoint>();



   private final ArrayList<YoFramePoint> consideredFootStepLocationsFramePoints = new ArrayList<YoFramePoint>();

   private final ArrayList<YoFramePoint> equivalentConstantCoPsFramePoints = new ArrayList<YoFramePoint>();
   private final ArrayList<DenseMatrix64F> equivalentConstantCoPsVectors = new ArrayList<DenseMatrix64F>();

   private final ArrayList<YoFramePoint> initialICPsFramePoints = new ArrayList<YoFramePoint>();
   private final ArrayList<DenseMatrix64F> initialICPsVectors = new ArrayList<DenseMatrix64F>();

   private final YoFramePoint initialDoubleSupportICPposFramePoint = new YoFramePoint("initialDoubleSupportICPposFramePoint", "",
                                                                        ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint finalDoubleSupportICPposFramePoint = new YoFramePoint("finalDoubleSupportICPposFramePoint", "", ReferenceFrame.getWorldFrame(),
                                                                      registry);

   private final YoFramePoint desiredDCMposOfTimeFramePoint = new YoFramePoint("desiredDCMposOfTime", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredECMPofTimeFramePoint = new YoFramePoint("desiredECMPofTime", "", ReferenceFrame.getWorldFrame(), registry);



   private ArrayList<YoFrameLineSegment2d> listOfICPLineSegments = new ArrayList<YoFrameLineSegment2d>();

   private final YoFrameLineSegment2d icpVelocityLine = new YoFrameLineSegment2d("icpVelocityLine", "", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameLineSegment2d comVelocityLine = new YoFrameLineSegment2d("comVelocityLine", "", ReferenceFrame.getWorldFrame(), registry);



   private final YoFrameVector desiredDCMvelOfTimeFrameVector = new YoFrameVector("desiredDCMvelOfTime", "", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable desiredDCMvelAbsolute = new DoubleYoVariable("desiredDCMvelAbsolute", registry);

   private PointAndLinePlotter equivalentCoPAndICPPlotter = new PointAndLinePlotter();


   YoFramePoint tempFramePointI = new YoFramePoint("pointTemp1", "", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint tempFramePointIminus1 = new YoFramePoint("pointTemp2", "", ReferenceFrame.getWorldFrame(), registry);


   private final YoFramePoint comPositionFramePoint = new YoFramePoint("comPositionFramePoint", "", ReferenceFrame.getWorldFrame(), registry);
   private final DenseMatrix64F comPositionVector = new DenseMatrix64F(3, 1, true, 0, 0, 0);

   private final YoFrameVector comVelocityFrameVector = new YoFrameVector("comVelocityFrameVector", "", ReferenceFrame.getWorldFrame(), registry);
   private final DenseMatrix64F comVelocityVector = new DenseMatrix64F(3, 1, true, 0, 0, 0);




   private final BooleanYoVariable isFirstStep = new BooleanYoVariable("isFirstStep", registry);
   private final BooleanYoVariable isSingleSupport = new BooleanYoVariable("isSingleSupport", registry);
   private final SmoothSupportState supportState = new SmoothSupportState();
   private final DoubleYoVariable moveTime = new DoubleYoVariable("moveTime", registry);
   private final DoubleYoVariable currentTime = new DoubleYoVariable("currentTime", registry);

   private final DoubleYoVariable simDT = new DoubleYoVariable("simDT", registry);

   private final double singleSupportTime;
   private final double doubleSupportTime;
   private final double initialTransferSupportTime;




   private DoubleSupportICPComputer dsICPcomputer = new DoubleSupportICPComputer(registry);
   private Robot testRobot = new Robot("testRobot");


   double leftRightFlip = 1;
   double stepLength = 0.3;
   double halfStepWidth = 0.1;




   public SCSDoubleSupportICPTesterController5(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, DoubleYoVariable yoTime, double simDText,
           double singleSupportTimeExt, double doubleSupportTimeExt, double initialTransferSupportTimeExt, Robot testRobot)
   {
      this.testRobot = testRobot;

      this.simDT.set(simDText);
      this.singleSupportTime = singleSupportTimeExt;
      this.doubleSupportTime = doubleSupportTimeExt;
      this.initialTransferSupportTime = initialTransferSupportTimeExt;




      YoFramePoint tempstepListElement1 = new YoFramePoint("stepListElement" + 100, "", ReferenceFrame.getWorldFrame(), registry);
      tempstepListElement1.set(0, -leftRightFlip * halfStepWidth, 0.5);
      footStepLocationsFramePoints.add(tempstepListElement1);


      for (int i = 0; i < numberOfStepsInStepList; i++)
      {
         YoFramePoint tempstepListElement = new YoFramePoint("stepListElement" + i, "", ReferenceFrame.getWorldFrame(), registry);
         tempstepListElement.set(i * stepLength, leftRightFlip * halfStepWidth, 0.5);

         footStepLocationsFramePoints.add(tempstepListElement);
         leftRightFlip = -leftRightFlip;
      }



      for (int i = 0; i < numberOfConsideredFootstepLocations; i++)
      {
         consideredFootStepLocationsFramePoints.add(new YoFramePoint("tempFramePoint" + i, "", ReferenceFrame.getWorldFrame(), registry));
      }

      for (int i = 0; i < numberOfConsideredFootstepLocations; i++)
      {
         DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1, true, 0, 0, 0);
         equivalentConstantCoPsVectors.add(tempMatrix);

         YoFramePoint tempFramePoint = new YoFramePoint("equivalentConstantCoPframepoint" + i, "", ReferenceFrame.getWorldFrame(), registry);
         equivalentConstantCoPsFramePoints.add(tempFramePoint);
      }


      for (int i = 0; i < numberOfConsideredFootstepLocations - 1; i++)
      {
         YoFramePoint tempInitialICPFramePoint = new YoFramePoint("tempInitialICPFramePoint" + i, "", ReferenceFrame.getWorldFrame(), registry);
         initialICPsFramePoints.add(tempInitialICPFramePoint);

         YoFrameLineSegment2d tempYoFrameLine2d = new YoFrameLineSegment2d("line" + i, "", ReferenceFrame.getWorldFrame(), registry);
         listOfICPLineSegments.add(tempYoFrameLine2d);


         DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1, true, 0, 0, 0);
         initialICPsVectors.add(tempMatrix);

      }




      equivalentCoPAndICPPlotter.addPointListToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "currentFoot",
              consideredFootStepLocationsFramePoints, YoAppearance.Black(), 0.01);

      equivalentCoPAndICPPlotter.addPointListToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "equivalentConstCoP",
              equivalentConstantCoPsFramePoints, YoAppearance.Red(), 0.005);

      equivalentCoPAndICPPlotter.addPointListToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "initialICP", initialICPsFramePoints,
              YoAppearance.Green(), 0.01);


      equivalentCoPAndICPPlotter.addSinglePointToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "initialDoubleSupportICPpos",
              initialDoubleSupportICPposFramePoint, YoAppearance.Cyan(), 0.01);

      equivalentCoPAndICPPlotter.addSinglePointToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "finalDoubleSupportICPpos",
              finalDoubleSupportICPposFramePoint, YoAppearance.Cyan(), 0.01);

      equivalentCoPAndICPPlotter.addSinglePointToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "desiredDCMposOfTime",
              desiredDCMposOfTimeFramePoint, YoAppearance.OrangeRed(), 0.01);



      equivalentCoPAndICPPlotter.addSinglePointToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "comPosition", comPositionFramePoint,
              YoAppearance.Blue(), 0.015);

      equivalentCoPAndICPPlotter.addSinglePointToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "desiredECMPofTime",
              desiredECMPofTimeFramePoint, YoAppearance.Magenta(), 0.011);

      equivalentCoPAndICPPlotter.addLineListToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "icpLines", listOfICPLineSegments,
              Color.orange);

      equivalentCoPAndICPPlotter.addSingleLineToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "icpVelocityLine", icpVelocityLine,
              Color.gray);

      equivalentCoPAndICPPlotter.addSingleLineToDynamicGraphicsObjectsListRegistry(dynamicGraphicObjectsListRegistry, "comVelocityLine", comVelocityLine,
              Color.blue);


   }


   public void doControl()
   {
      supportState.propagateStateAndStateTime(this.simDT.getDoubleValue());
      moveTime.set(supportState.getMoveTime());
      currentTime.set(supportState.getCurrentTime());
      isSingleSupport.set(supportState.getIsSingleSupport());
      isFirstStep.set(supportState.getIsFirstStep());

//    supportState

      double steppingTime = supportState.getSteppingTime();
      double eCMPheight = 1.0;
      double gravConst = 9.81;
      double doubleSupportFirstStepFraction = 0.5;
      double dcmConst = Math.sqrt(eCMPheight / gravConst);

      if (supportState.getStepListUpdateRequestFlag() == true)
      {
         dsICPcomputer.updateSubFootListForSmoothICPTrajectory(equivalentConstantCoPsVectors, 
               footStepLocationsFramePoints, 
               equivalentConstantCoPsFramePoints,
               consideredFootStepLocationsFramePoints, 
                 numberOfConsideredFootstepLocations, 
                 equivalentConstantCoPsVectors);

         supportState.setStepListUpdateRequestFlag(false);
      }


      if (supportState.getIsSingleSupport())
      {
         dsICPcomputer.computeDoubleSupportPolynomialParams(equivalentConstantCoPsVectors, dcmConst, steppingTime, doubleSupportFirstStepFraction,
                 initialICPsVectors, supportState.getIsFirstStep(), supportState.getInitialTransferSupportTime(), supportState.getDoubleSupportTime());
      }


      dsICPcomputer.calcDCMandECMPofTime(equivalentConstantCoPsVectors, doubleSupportFirstStepFraction, dcmConst, initialICPsVectors,
                                         supportState.getIsFirstStep(), supportState.getInitialTransferSupportTime(), supportState.getDoubleSupportTime(),
                                         supportState.getIsSingleSupport(), supportState.getCurrentTime(), supportState.getSteppingTime());




      for (int i = 0; i < numberOfConsideredFootstepLocations; i++)
      {
         equivalentConstantCoPsFramePoints.get(i).set(equivalentConstantCoPsVectors.get(i).get(0), equivalentConstantCoPsVectors.get(i).get(1),
                 equivalentConstantCoPsVectors.get(i).get(2));
      }


      for (int i = 0; i < initialICPsVectors.size(); i++)
      {
         initialICPsFramePoints.get(i).set(initialICPsVectors.get(i).get(0), initialICPsVectors.get(i).get(1), initialICPsVectors.get(i).get(2));
      }


      initialDoubleSupportICPposFramePoint.set(dsICPcomputer.getInitialDoubleSupportICPpos().get(0), dsICPcomputer.getInitialDoubleSupportICPpos().get(1),
              dsICPcomputer.getInitialDoubleSupportICPpos().get(2));

      finalDoubleSupportICPposFramePoint.set(dsICPcomputer.getFinalDoubleSupportICPpos().get(0), dsICPcomputer.getFinalDoubleSupportICPpos().get(1),
              dsICPcomputer.getFinalDoubleSupportICPpos().get(2));


      desiredDCMposOfTimeFramePoint.set(dsICPcomputer.getDesiredDCMposOfTime().get(0), dsICPcomputer.getDesiredDCMposOfTime().get(1),
                                        dsICPcomputer.getDesiredDCMposOfTime().get(2));
      desiredDCMvelOfTimeFrameVector.set(dsICPcomputer.getDesiredDCMvelOfTime().get(0), dsICPcomputer.getDesiredDCMvelOfTime().get(1),
                                         dsICPcomputer.getDesiredDCMvelOfTime().get(2));
      desiredECMPofTimeFramePoint.set(dsICPcomputer.getDesiredECMPofTime().get(0), dsICPcomputer.getDesiredECMPofTime().get(1),
                                      dsICPcomputer.getDesiredECMPofTime().get(2));

      desiredDCMvelAbsolute.set(Math.sqrt(Math.pow(desiredDCMvelOfTimeFrameVector.getX(), 2) + Math.pow(desiredDCMvelOfTimeFrameVector.getY(), 2)
              + Math.pow(desiredDCMvelOfTimeFrameVector.getZ(), 2)));



      JojosICPutilities.discreteIntegrateCoMAndGetCoMVelocity(simDT.getDoubleValue(), dcmConst, dsICPcomputer.getDesiredDCMposOfTime(), comPositionVector,
              comVelocityVector);

      comPositionFramePoint.set(comPositionVector.get(0), comPositionVector.get(1), comPositionVector.get(2));
      comVelocityFrameVector.set(comVelocityVector.get(0), comVelocityVector.get(1), comVelocityVector.get(2));




      FramePoint2d ls1p1 = new FramePoint2d(ReferenceFrame.getWorldFrame(), equivalentConstantCoPsFramePoints.get(0).getX(),
                              equivalentConstantCoPsFramePoints.get(0).getY());
      FramePoint2d ls1p2 = new FramePoint2d(ReferenceFrame.getWorldFrame(), initialICPsFramePoints.get(1).getX(), initialICPsFramePoints.get(1).getY());

      if (ls1p2.distanceSquared(ls1p1) < 1E-6)
         ls1p2.setX(ls1p2.getX() + 1E-6);


      FrameLineSegment2d lineSegment1 = new FrameLineSegment2d(ls1p1, ls1p2);
      listOfICPLineSegments.get(0).setFrameLineSegment2d(lineSegment1);

      FramePoint2d ls2p1 = new FramePoint2d(ReferenceFrame.getWorldFrame(), equivalentConstantCoPsFramePoints.get(1).getX(),
                              equivalentConstantCoPsFramePoints.get(1).getY());
      FramePoint2d ls2p2 = new FramePoint2d(ReferenceFrame.getWorldFrame(), initialICPsFramePoints.get(2).getX(), initialICPsFramePoints.get(2).getY());

      if (ls2p2.distanceSquared(ls2p1) < 1E-6)
         ls2p2.setX(ls2p2.getX() + 1E-6);

      FrameLineSegment2d lineSegment2 = new FrameLineSegment2d(ls2p1, ls2p2);
      listOfICPLineSegments.get(1).setFrameLineSegment2d(lineSegment2);

      FramePoint2d ls3p1 = new FramePoint2d(ReferenceFrame.getWorldFrame(), equivalentConstantCoPsFramePoints.get(2).getX(),
                              equivalentConstantCoPsFramePoints.get(2).getY());
      FramePoint2d ls3p2 = new FramePoint2d(ReferenceFrame.getWorldFrame(), equivalentConstantCoPsFramePoints.get(3).getX(),
                              equivalentConstantCoPsFramePoints.get(3).getY());


      if (ls3p2.distanceSquared(ls3p1) < 1E-6)
         ls3p2.setX(ls3p2.getX() + 1E-6);

      FrameLineSegment2d lineSegment3 = new FrameLineSegment2d(ls3p1, ls3p2);
      listOfICPLineSegments.get(2).setFrameLineSegment2d(lineSegment3);




      FramePoint2d velPointerStart = new FramePoint2d(ReferenceFrame.getWorldFrame(), desiredDCMposOfTimeFramePoint.getX(),
                                        desiredDCMposOfTimeFramePoint.getY());
      FramePoint2d velPointerEnd = new FramePoint2d(ReferenceFrame.getWorldFrame(),
                                      desiredDCMposOfTimeFramePoint.getX() + desiredDCMvelOfTimeFrameVector.getX(),
                                      desiredDCMposOfTimeFramePoint.getY() + desiredDCMvelOfTimeFrameVector.getY());

      if (velPointerStart.distanceSquared(velPointerEnd) < 1E-6)
         velPointerStart.setX(velPointerStart.getX() + 1E-6);

      FrameLineSegment2d lineSegment4 = new FrameLineSegment2d(velPointerStart, velPointerEnd);
      icpVelocityLine.setFrameLineSegment2d(lineSegment4);




      FramePoint2d comVelPointerStart = new FramePoint2d(ReferenceFrame.getWorldFrame(), comPositionFramePoint.getX(), comPositionFramePoint.getY());
      FramePoint2d comVelPointerEnd = new FramePoint2d(ReferenceFrame.getWorldFrame(), comPositionFramePoint.getX() + comVelocityFrameVector.getX(),
                                         comPositionFramePoint.getY() + comVelocityFrameVector.getY());

      if (comVelPointerStart.distanceSquared(comVelPointerEnd) < 1E-6)
         comVelPointerStart.setX(comVelPointerStart.getX() + 1E-6);

      FrameLineSegment2d lineSegmentComVel = new FrameLineSegment2d(comVelPointerStart, comVelPointerEnd);
      comVelocityLine.setFrameLineSegment2d(lineSegmentComVel);
   }

   private ArrayList<FramePoint> getFramePoints(ArrayList<YoFramePoint> yoFramePoints)
   {
      ArrayList<FramePoint> ret = new ArrayList<FramePoint>();
      
      for (YoFramePoint yoFramePoint : yoFramePoints)
      {
         ret.add(yoFramePoint.getFramePointCopy());
      }
      
      return ret;
   }
   
   public void initialize()
   {
      this.isSingleSupport.set(true);

      boolean initialIsSingleSupport = false;
      this.supportState.initializeSupportState(initialIsSingleSupport, singleSupportTime, doubleSupportTime, initialTransferSupportTime);
   }


   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

}
