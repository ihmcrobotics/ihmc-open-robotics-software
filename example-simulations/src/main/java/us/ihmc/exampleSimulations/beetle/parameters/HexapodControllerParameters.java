package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.wholeBodyControlCore.pidGains.PIDSE3GainsBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface HexapodControllerParameters
{
   double getTransferTime();

   double getSwingTime();

   double getSwingXYProportionalGain();

   double getSwingZProportionalGain();

   PIDSE3GainsBasics getBodySpatialGains();

   void getBodySpatialLinearQPWeight(Vector3D linearWeight);

   void getBodySpatialAngularQPWeight(Vector3D angularWeight);

   SelectionMatrix6D getBodySpatialSelectionMatrix();

   PIDSE3GainsBasics getFootGains();

   WholeBodyControllerCoreMode getControlMode();
}