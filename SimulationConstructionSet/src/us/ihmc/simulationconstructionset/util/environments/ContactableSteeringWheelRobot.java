package us.ihmc.simulationconstructionset.util.environments;

import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameCylinder3d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class ContactableSteeringWheelRobot extends ContactableValveRobot
{
   private double spinnerHandleThickness;

   
   public ContactableSteeringWheelRobot(String name, double valveRadius, double valveOffsetFromWall, double valveThickness, int numberOfSpokes,
         double spokesThickness, FramePose valvePoseInWorld, double valveNumberOfPossibleTurns, double valveMass)
   {
      super(name, valveRadius, valveOffsetFromWall, valveThickness, numberOfSpokes, spokesThickness, valvePoseInWorld, valveNumberOfPossibleTurns, valveMass);
      
      spinnerHandleThickness = 1.5 * spokesThickness;
   }
   
   public ContactableSteeringWheelRobot(String name, ValveType valveType, double valveNumberOfPossibleTurns, FramePose valvePoseInWorld)
   {
      this(name, 7.0 * 0.0254, valveType.getValveOffsetFromWall(), valveType.getValveThickness(), valveType.getNumberOfSpokes(), valveType
            .getSpokesThickness(), valvePoseInWorld, valveNumberOfPossibleTurns, valveType.getValveMass());
   }

   public void createValveRobot()
   {
      super.createValveRobot();

      addSpinnerHandleToRobot();
   }
   
   private void addSpinnerHandleToRobot()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBodyTransform invertTransform = new RigidBodyTransform();
      Quat4d quat = new Quat4d();

      RotationFunctions.setQuaternionBasedOnYawPitchRoll(quat, 0.0, Math.PI / 2.0, 0.0);
      transform.setRotation(quat);
      transform.setTranslation(-valveRadius, 0.0, valveRadius - spokesThickness / 2.0);
      
      invertTransform.set(transform);
      invertTransform.invert();

      FrameCylinder3d spokeCylinder = new FrameCylinder3d(valveFrame, transform, valveRadius - spokesThickness / 2.0, spinnerHandleThickness / 2.0);
      spokesCylinders.add(spokeCylinder);

      valveLinkGraphics.transform(transform);
      valveLinkGraphics.addCylinder(valveRadius - spokesThickness / 2.0, spokesThickness / 2.0, YoAppearance.IndianRed());
      valveLinkGraphics.transform(invertTransform);

      valveLink.setLinkGraphics(valveLinkGraphics);
   }
}
