package us.ihmc.quadrupedUI.uiControllers;

import com.sun.javafx.collections.ImmutableObservableList;
import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedUI.QuadrupedUIMessagerAPI;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.zip.CRC32;

public class ManualStepTabController
{
   private static final double defaultStepHeight = 0.05;
   private static final double defaultStepDuration = 0.4;
   private static final double defaultDwellTime = 0.2;

   private final AtomicBoolean useTrotOverCrawl = new AtomicBoolean(false);

   private JavaFXMessager messager;
   private AtomicReference<QuadrupedXGaitSettingsReadOnly> xGaitSettingsReference;

   private FullQuadrupedRobotModel fullRobotModel;
   private OneDoFJointBasics[] allJoints;
   private int jointNameHash;
   private QuadrupedReferenceFrames referenceFrames;


   @FXML
   private Spinner<Double> swingHeight;

   @FXML
   private Spinner<Double> stepDuration;

   @FXML
   private Spinner<Double> stepHeight;

   @FXML
   private Spinner<Double> stepLength;

   @FXML
   private Spinner<Double> stepWidth;

   @FXML ComboBox<RobotQuadrant> firstFoot;

   @FXML
   private Spinner<Integer> numberOfSteps;

   @FXML
   private Spinner<Double> dwellTime;

   @FXML
   private Button stepButton;

   @FXML
   private CheckBox useTrot;

   public void setFullRobotModelFactory(FullQuadrupedRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      allJoints = fullRobotModel.getOneDoFJoints();
      jointNameHash = calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel);
   }

   private static int calculateJointNameHash(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointBasics joint : joints)
      {
         crc.update(joint.getName().getBytes());
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         crc.update(forceSensorDefinition.getSensorName().getBytes());
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         crc.update(imuDefinition.getName().getBytes());
      }

      return (int) crc.getValue();
   }


   public void attachMessager(JavaFXMessager messager, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings)
   {
      this.messager = messager;
      xGaitSettingsReference = messager.createInput(QuadrupedUIMessagerAPI.XGaitSettingsTopic);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.RobotConfigurationDataTopic, this::handleRobotConfigurationData);

      xGaitSettingsReference.set(defaultXGaitSettings);
   }

   public void handleRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (referenceFrames == null || fullRobotModel == null)
         return;

      if (robotConfigurationData.getJointNameHash() != jointNameHash )
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");

      RigidBodyTransform newRootJointPose = new RigidBodyTransform(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());
      fullRobotModel.getRootJoint().setJointConfiguration(newRootJointPose);

      float[] newJointConfiguration = robotConfigurationData.getJointAngles().toArray();
      for (int i = 0; i < allJoints.length; i++)
         allJoints[i].setQ(newJointConfiguration[i]);

      fullRobotModel.getElevator().updateFramesRecursively();
      referenceFrames.updateFrames();
   }



   public void bindControls()
   {
      swingHeight.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.01, 0.2, defaultStepHeight, 0.01));
      stepHeight.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 0.25, 0.0, 0.01));
      stepLength.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-0.2, 0.45, 0.0, 0.02));
      stepWidth.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-0.1, 0.1, 0.0, 0.02));
      stepDuration.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.05, 3.0, defaultStepDuration, 0.05));
      dwellTime.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 3.0, defaultDwellTime, 0.05));
      numberOfSteps.setValueFactory(new SpinnerValueFactory.IntegerSpinnerValueFactory(1, 20, 1));
      firstFoot.setItems(new ImmutableObservableList<>(RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_LEFT, RobotQuadrant.HIND_RIGHT));
      useTrot.setSelected(false);

      firstFoot.getSelectionModel().select(RobotQuadrant.FRONT_RIGHT);
   }

   public void sendSteps()
   {
      if (firstFoot.getSelectionModel().isEmpty())
      {
         LogTools.info("Select a quadrant");
         return;
      }

      if (useTrot.isSelected())
         handleTrotRequest();
      else
         handleCrawlRequest();
   }

   public void setUseTrot()
   {
      useTrotOverCrawl.set(useTrot.isSelected());
   }


   private void handleTrotRequest()
   {
      RobotQuadrant firstFootQuadrant = firstFoot.getValue();
      double swingHeight = this.swingHeight.getValue();
      double stepHeight = this.stepHeight.getValue();
      double stepLength = this.stepLength.getValue();
      double stepWidth = this.stepWidth.getValue();
      double stepDuration = this.stepDuration.getValue();
      double dwellTime = this.dwellTime.getValue();
      int numberOfSteps = this.numberOfSteps.getValue();

      QuadrupedXGaitSettingsReadOnly xGaitSettings = xGaitSettingsReference.get();

      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();

      stepListMessage.setIsExpressedInAbsoluteTime(false);
      stepListMessage.getQuadrupedStepList().clear();

      double stanceLength = xGaitSettings.getStanceLength();
      double stanceWidth = xGaitSettings.getStanceWidth();

      ReferenceFrame supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D centerPoint = new FramePoint3D(supportFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D solePosition = new FramePoint3D(referenceFrames.getSoleFrame(robotQuadrant));
         solePosition.changeFrame(supportFrame);
         centerPoint.add(solePosition);
      }
      centerPoint.scale(0.25);

      QuadrantDependentList<FramePoint3D> nominalPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D nominalPosition = new FramePoint3D(centerPoint);
         nominalPosition.addX(0.5 * robotQuadrant.getEnd().negateIfHindEnd(stanceLength));
         nominalPosition.addY(0.5 * robotQuadrant.getSide().negateIfRightSide(stanceWidth));
         nominalPositions.put(robotQuadrant, nominalPosition);
      }

      double timeDelay = 0.0;
      double lengthOffset = 0.0;
      double widthOffset = 0.0;


      for (int i = 0; i < numberOfSteps; i++)
      {
         lengthOffset += stepLength;
         widthOffset += stepWidth;

         RobotQuadrant frontQuadrant = firstFootQuadrant;
         RobotQuadrant rearQuadrant = firstFootQuadrant.getDiagonalOppositeQuadrant();

         QuadrupedTimedStepMessage frontFootMessage = stepListMessage.getQuadrupedStepList().add();
         QuadrupedTimedStepMessage hindFootMessage = stepListMessage.getQuadrupedStepList().add();

         nominalPositions.get(frontQuadrant).changeFrame(supportFrame);
         nominalPositions.get(rearQuadrant).changeFrame(supportFrame);

         FramePoint3D frontPosition = new FramePoint3D(nominalPositions.get(frontQuadrant));
         FramePoint3D rearPosition = new FramePoint3D(nominalPositions.get(rearQuadrant));
         frontPosition.addX(lengthOffset);
         frontPosition.addY(widthOffset);
         rearPosition.addX(lengthOffset);
         rearPosition.addY(widthOffset);

         frontPosition.changeFrame(ReferenceFrame.getWorldFrame());
         rearPosition.changeFrame(ReferenceFrame.getWorldFrame());
         frontPosition.addZ(stepHeight);
         rearPosition.addZ(stepHeight);

         frontFootMessage.getTimeInterval().setStartTime(timeDelay);
         frontFootMessage.getTimeInterval().setEndTime(timeDelay + stepDuration);
         frontFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
         frontFootMessage.getQuadrupedStepMessage().setRobotQuadrant(frontQuadrant.toByte());
         frontFootMessage.getQuadrupedStepMessage().getGoalPosition().set(frontPosition);

         hindFootMessage.getTimeInterval().setStartTime(timeDelay);
         hindFootMessage.getTimeInterval().setEndTime(timeDelay + stepDuration);
         hindFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
         hindFootMessage.getQuadrupedStepMessage().setRobotQuadrant(rearQuadrant.toByte());
         hindFootMessage.getQuadrupedStepMessage().getGoalPosition().set(rearPosition);


         firstFootQuadrant = firstFootQuadrant.getAcrossBodyQuadrant();
         timeDelay += stepDuration + dwellTime;
      }

      messager.submitMessage(QuadrupedUIMessagerAPI.ManualStepsListMessageTopic, stepListMessage);
   }

   private void handleCrawlRequest()
   {
      RobotQuadrant firstFootQuadrant = firstFoot.getValue();
      double swingHeight = this.swingHeight.getValue();
      double stepHeight = this.stepHeight.getValue();
      double stepLength = this.stepLength.getValue();
      double stepWidth = this.stepWidth.getValue();
      double stepDuration = this.stepDuration.getValue();
      double dwellTime = this.dwellTime.getValue();
      int numberOfSteps = this.numberOfSteps.getValue();

      QuadrupedXGaitSettingsReadOnly xGaitSettings = xGaitSettingsReference.get();

      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();

      stepListMessage.setIsExpressedInAbsoluteTime(false);
      stepListMessage.getQuadrupedStepList().clear();

      double currentTime = 0.0;

      double stanceLength = xGaitSettings.getStanceLength();
      double stanceWidth = xGaitSettings.getStanceWidth();

      ReferenceFrame supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D centerPoint = new FramePoint3D(supportFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D solePosition = new FramePoint3D(referenceFrames.getSoleFrame(robotQuadrant));
         solePosition.changeFrame(supportFrame);
         centerPoint.add(solePosition);
      }
      centerPoint.scale(0.25);

      QuadrantDependentList<FramePoint3D> nominalPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D nominalPosition = new FramePoint3D(centerPoint);
         nominalPosition.addX(0.5 * robotQuadrant.getEnd().negateIfHindEnd(stanceLength));
         nominalPosition.addY(0.5 * robotQuadrant.getSide().negateIfRightSide(stanceWidth));
         nominalPositions.put(robotQuadrant, nominalPosition);
      }


      double lengthOffset = 0.0;
      double widthOffset = 0.0;

      for (int i = 0; i < Math.ceil(numberOfSteps / 2.0); i++)
      {
         lengthOffset += 0.5 * stepLength;
         widthOffset += 0.5 * stepWidth;

         QuadrupedTimedStepMessage firstFootMessage = stepListMessage.getQuadrupedStepList().add();

         nominalPositions.get(firstFootQuadrant).changeFrame(supportFrame);

         FramePoint3D firstPosition = new FramePoint3D(nominalPositions.get(firstFootQuadrant));
         firstPosition.addX(lengthOffset);
         firstPosition.addY(widthOffset);

         firstPosition.changeFrame(ReferenceFrame.getWorldFrame());
         firstPosition.addZ(stepHeight);

         firstFootMessage.getTimeInterval().setStartTime(currentTime);
         firstFootMessage.getTimeInterval().setEndTime(currentTime + stepDuration);
         firstFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
         firstFootMessage.getQuadrupedStepMessage().setRobotQuadrant(firstFootQuadrant.toByte());
         firstFootMessage.getQuadrupedStepMessage().getGoalPosition().set(firstPosition);


         if (2 * i + 1 < numberOfSteps)
         {
            lengthOffset += 0.5 * stepLength;
            widthOffset += 0.5 * stepWidth;

            QuadrupedTimedStepMessage oppositeFootMessage = stepListMessage.getQuadrupedStepList().add();

            currentTime += 0.5 * stepDuration + dwellTime;
            RobotQuadrant oppositeQuadrant = firstFootQuadrant.getDiagonalOppositeQuadrant();

            nominalPositions.get(oppositeQuadrant).changeFrame(supportFrame);

            FramePoint3D secondPosition = new FramePoint3D(nominalPositions.get(oppositeQuadrant));
            secondPosition.addX(lengthOffset);
            secondPosition.addY(widthOffset);

            secondPosition.changeFrame(ReferenceFrame.getWorldFrame());
            secondPosition.addZ(stepHeight);

            oppositeFootMessage.getTimeInterval().setStartTime(currentTime);
            oppositeFootMessage.getTimeInterval().setEndTime(currentTime + stepDuration);
            oppositeFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
            oppositeFootMessage.getQuadrupedStepMessage().setRobotQuadrant(oppositeQuadrant.toByte());
            oppositeFootMessage.getQuadrupedStepMessage().getGoalPosition().set(secondPosition);
         }
         else
         {
            break;
         }

         firstFootQuadrant = firstFootQuadrant.getAcrossBodyQuadrant();

         currentTime += 0.5 * stepDuration + dwellTime;
      }

      messager.submitMessage(QuadrupedUIMessagerAPI.ManualStepsListMessageTopic, stepListMessage);
   }
}
