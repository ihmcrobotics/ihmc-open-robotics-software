package us.ihmc.avatar.logProcessor;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class UpdatableHighLevelHumanoidControllerToolbox extends HighLevelHumanoidControllerToolbox
{
   private static final boolean UPDATE_CAPTURE_POINT_FROM_SCS = false;

   private final SideDependentList<YoFramePoint2d> desiredCoPsUpdatedFromSCS = new SideDependentList<>();

   private final YoFramePoint capturePointUpdatedFromSCS;

   public UpdatableHighLevelHumanoidControllerToolbox(SimulationConstructionSet scs, FullHumanoidRobotModel fullRobotModel,
         GeometricJacobianHolder robotJacobianHolder, CommonHumanoidReferenceFrames referenceFrames, SideDependentList<FootSwitchInterface> footSwitches,
         CenterOfMassDataHolderReadOnly centerOfMassDataHolder, SideDependentList<ForceSensorDataReadOnly> wristForceSensors, DoubleYoVariable yoTime,
         double gravityZ, double omega0, TwistCalculator twistCalculator, SideDependentList<ContactableFoot> feet, double controlDT,
         ArrayList<Updatable> updatables, List<ContactablePlaneBody> contactableBodies, YoGraphicsListRegistry yoGraphicsListRegistry,
         InverseDynamicsJoint... jointsToIgnore)
   {
      super(fullRobotModel, robotJacobianHolder, referenceFrames, footSwitches, centerOfMassDataHolder, wristForceSensors, yoTime, gravityZ, omega0,
            twistCalculator, feet, controlDT, updatables, contactableBodies, yoGraphicsListRegistry, jointsToIgnore);

      if (UPDATE_CAPTURE_POINT_FROM_SCS)
      {
         String capturePointNameSpace = HighLevelHumanoidControllerToolbox.class.getSimpleName();
         DoubleYoVariable capturePointX = (DoubleYoVariable) scs.getVariable(capturePointNameSpace, "capturePointX");
         DoubleYoVariable capturePointY = (DoubleYoVariable) scs.getVariable(capturePointNameSpace, "capturePointY");
         DoubleYoVariable capturePointZ = (DoubleYoVariable) scs.getVariable(capturePointNameSpace, "capturePointZ");
         capturePointUpdatedFromSCS = new YoFramePoint(capturePointX, capturePointY, capturePointZ, worldFrame);
      }
      else
      {
         capturePointUpdatedFromSCS = null;
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String desiredCoPNameSpace = PlaneContactWrenchProcessor.class.getSimpleName();
         String desiredCoPName = side + "SoleCoP2d";
         DoubleYoVariable desiredCoPx = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "X");
         DoubleYoVariable desiredCoPy = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "Y");
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         YoFramePoint2d desiredCoP = new YoFramePoint2d(desiredCoPx, desiredCoPy, soleFrame);
         desiredCoPsUpdatedFromSCS.put(robotSide, desiredCoP);
      }
   }

   @Override
   public void update()
   {
      if (UPDATE_CAPTURE_POINT_FROM_SCS)
      {
         // update the yoCapturePoint
         yoCapturePoint.set(capturePointUpdatedFromSCS);
      }

      // update the bipedSupportPolygons
      updateBipedSupportPolygons();

      // update the footDesiredCenterOfPressures
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactableFoot contactableFoot = feet.get(robotSide);
         FramePoint2d desiredCop = desiredCoPsUpdatedFromSCS.get(robotSide).getFrameTuple2d();
         setDesiredCenterOfPressure(contactableFoot, desiredCop);
      }
   }
}
