package us.ihmc.avatar.logProcessor;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class UpdatableHighLevelHumanoidControllerToolbox extends HighLevelHumanoidControllerToolbox
{
   private static final boolean UPDATE_CAPTURE_POINT_FROM_SCS = false;

   private final SideDependentList<YoFramePoint2D> desiredCoPsUpdatedFromSCS = new SideDependentList<>();

   private final YoFramePoint3D capturePointUpdatedFromSCS;

   public UpdatableHighLevelHumanoidControllerToolbox(SimulationConstructionSet scs, FullHumanoidRobotModel fullRobotModel,
                                                      CommonHumanoidReferenceFrames referenceFrames, SideDependentList<FootSwitchInterface> footSwitches,
                                                      SideDependentList<ForceSensorDataReadOnly> wristForceSensors, YoDouble yoTime, double gravityZ,
                                                      double omega0, SideDependentList<ContactableFoot> feet, double controlDT, ArrayList<Updatable> updatables,
                                                      List<ContactablePlaneBody> contactableBodies, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                      JointBasics... jointsToIgnore)
   {
      super(fullRobotModel, referenceFrames, footSwitches, wristForceSensors, yoTime, gravityZ, omega0, feet, controlDT, updatables, contactableBodies,
            yoGraphicsListRegistry, jointsToIgnore);

      if (UPDATE_CAPTURE_POINT_FROM_SCS)
      {
         String capturePointNameSpace = HighLevelHumanoidControllerToolbox.class.getSimpleName();
         YoDouble capturePointX = (YoDouble) scs.getVariable(capturePointNameSpace, "capturePointX");
         YoDouble capturePointY = (YoDouble) scs.getVariable(capturePointNameSpace, "capturePointY");
         YoDouble capturePointZ = (YoDouble) scs.getVariable(capturePointNameSpace, "capturePointZ");
         capturePointUpdatedFromSCS = new YoFramePoint3D(capturePointX, capturePointY, capturePointZ, worldFrame);
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
         YoDouble desiredCoPx = (YoDouble) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "X");
         YoDouble desiredCoPy = (YoDouble) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "Y");
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         YoFramePoint2D desiredCoP = new YoFramePoint2D(desiredCoPx, desiredCoPy, soleFrame);
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
         setDesiredCenterOfPressure(contactableFoot, desiredCoPsUpdatedFromSCS.get(robotSide));
      }
   }
}
