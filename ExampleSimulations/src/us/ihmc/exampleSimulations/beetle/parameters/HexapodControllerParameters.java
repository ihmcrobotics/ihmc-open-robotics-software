package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface HexapodControllerParameters
{
   double getTransferTime();

   double getSwingTime();

   double getSwingXYProportionalGain();

   double getSwingZProportionalGain();

   PIDSE3Gains getBodySpatialGains();

   void getBodySpatialLinearQPWeight(Vector3D linearWeight);

   void getBodySpatialAngularQPWeight(Vector3D angularWeight);

   SelectionMatrix6D getBodySpatialSelectionMatrix();

   YoPIDSE3Gains getFootGains();
}