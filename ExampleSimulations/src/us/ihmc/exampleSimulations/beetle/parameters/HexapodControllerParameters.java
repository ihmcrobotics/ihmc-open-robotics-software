package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface HexapodControllerParameters
{
   double getTransferTime();

   double getSwingTime();

   double getSwingXYProportionalGain();

   double getSwingZProportionalGain();

   SE3PIDGainsInterface getBodySpatialGains();

   void getBodySpatialLinearQPWeight(Vector3D linearWeight);

   void getBodySpatialAngularQPWeight(Vector3D angularWeight);

   SelectionMatrix6D getBodySpatialSelectionMatrix();

   YoSE3PIDGainsInterface getFootGains();
}