package us.ihmc.exampleSimulations.beetle.parameters;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.controllers.PDGainsInterface;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface HexapodControllerParameters
{
   double getSwingTime();

   double getSwingXYProportionalGain();

   double getSwingZProportionalGain();

   SE3PIDGainsInterface getBodySpatialGains();

   void getBodySpatialLinearQPWeight(Vector3d linearWeight);

   void getBodySpatialAngularQPWeight(Vector3d angularWeight);

   DenseMatrix64F getBodySpatialSelectionMatrix();

   YoSE3PIDGainsInterface getFootGains();

   PDGainsInterface getJointGains(OneDoFJoint joint);

}