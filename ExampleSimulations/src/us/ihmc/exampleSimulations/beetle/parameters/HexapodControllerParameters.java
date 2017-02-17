package us.ihmc.exampleSimulations.beetle.parameters;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;

public interface HexapodControllerParameters
{
   double getTransferTime();

   double getSwingTime();

   double getSwingXYProportionalGain();

   double getSwingZProportionalGain();

   SE3PIDGainsInterface getBodySpatialGains();

   void getBodySpatialLinearQPWeight(Vector3D linearWeight);

   void getBodySpatialAngularQPWeight(Vector3D angularWeight);

   DenseMatrix64F getBodySpatialSelectionMatrix();

   YoSE3PIDGainsInterface getFootGains();
}