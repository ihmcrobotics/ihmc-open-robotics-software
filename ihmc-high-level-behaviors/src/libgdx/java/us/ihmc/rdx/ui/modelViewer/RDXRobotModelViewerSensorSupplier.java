package us.ihmc.rdx.ui.modelViewer;

import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableSensor;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public interface RDXRobotModelViewerSensorSupplier
{
   RDXInteractableSensor build(RDXBaseUI baseUI, HumanoidReferenceFrames humanoidReferenceFrames, HumanoidRobotSensorInformation sensorInformation);
}
