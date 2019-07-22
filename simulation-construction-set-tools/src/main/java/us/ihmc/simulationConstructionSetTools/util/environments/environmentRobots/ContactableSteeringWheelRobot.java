package us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots;

import java.util.ArrayList;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameCylinder3d;
import us.ihmc.robotics.geometry.shapes.FrameTorus3d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class ContactableSteeringWheelRobot extends ContactablePinJointRobot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double DEFAULT_DAMPING = 3;

   private String name;

   protected double steeringWheelRadius;
   private double steeringColunmLength;
   private double steeringWheelThickness;

   protected double spokesThickness;

   private FramePose3D steeringWheelPoseInWorld = new FramePose3D();

   private double totalNumberOfPossibleTurns;

   private final YoDouble steeringWheelAngleAsAbsolutePercentageOfRangeOfMotion;

   private double mass;
   private Matrix3D inertiaMatrix;

   private FrameTorus3d steeringWheelTorus;
   protected ArrayList<FrameCylinder3d> spokesCylinders = new ArrayList<FrameCylinder3d>();

   protected Link steeringWheelLink;
   private PinJoint steeringWheelPinJoint;
   protected Graphics3DObject steeringWheelLinkGraphics = new Graphics3DObject();
   private final YoDouble steeringDamping;

   protected PoseReferenceFrame steeringWheelFrame;

   private final RigidBodyTransform originalSteeringWheelPose = new RigidBodyTransform();

   public static ContactableSteeringWheelRobot createPolarisSteeringWheel(double x, double y, double z, double yaw, double pitch)
   {
      FramePose3D steeringWheelPoseInWorld = new FramePose3D(worldFrame);
      steeringWheelPoseInWorld.setPosition(x, y, z);
      steeringWheelPoseInWorld.setOrientationYawPitchRoll(yaw, pitch, 0.0);
      double steeringWheelRadius = 0.175;
      double steerigColunmLength = 0.20;
      double steeringWheelThickness = 0.03;
      double spokeThickness = 0.03;
      double totalNumberOfPossibleTurns = 3.25;
      double mass = 5.0;
      return new ContactableSteeringWheelRobot("PolarisSteeringWheel", steeringWheelRadius, steerigColunmLength, steeringWheelThickness, spokeThickness, steeringWheelPoseInWorld, totalNumberOfPossibleTurns, mass);
   }

   public ContactableSteeringWheelRobot(String name, double steeringWheelRadius, double steerigColunmLength, double steeringWheelThickness, double spokesThickness,
         FramePose3D steeringWheelPoseInWorld, double totalNumberOfPossibleTurns, double mass)
   {
      super(name);
      this.name = name;
      setProperties(steeringWheelRadius, steerigColunmLength, steeringWheelThickness, spokesThickness, totalNumberOfPossibleTurns, mass);
      setPoseInWorld(steeringWheelPoseInWorld);
      setMass(mass);
      steeringDamping = new YoDouble(getName() + "SteeringDamping", yoVariableRegistry);
      steeringDamping.set(DEFAULT_DAMPING);
      steeringWheelAngleAsAbsolutePercentageOfRangeOfMotion = new YoDouble("steeringWheelAngleAsAbsolutePercentageOfRangeOfMotion", yoVariableRegistry);
      steeringWheelAngleAsAbsolutePercentageOfRangeOfMotion.set(0.0);
   }

   public void setProperties(double steeringWheelRadius, double steerinWheelColunmLength, double steeringWheelThickness, double spokesThickness,
         double totalNumberOfPossibleTurns, double mass)
   {
      this.steeringWheelRadius = steeringWheelRadius;
      this.steeringColunmLength = steerinWheelColunmLength;
      this.steeringWheelThickness = steeringWheelThickness;

      this.spokesThickness = spokesThickness;

      this.totalNumberOfPossibleTurns = totalNumberOfPossibleTurns;

      this.mass = mass;
   }

   private FramePoint3D spinnerHandleCenter = null;

   public void addSpinnerHandle(double percentOfSteeringWheelRadius)
   {
      addSpinnerHandle(0.0, percentOfSteeringWheelRadius, 0.15, spokesThickness / 2.0, 0.0);
   }

   public void addSpinnerHandle(double angleOnSteeringWheelingInDegrees, double percentOfSteeringWheelRadius, double handleLength, double handleRadius,
         double distanceFromWheel)
   {

      double angleOnSteeringWheel = Math.toRadians(angleOnSteeringWheelingInDegrees);
      double distanceFromCenter = percentOfSteeringWheelRadius * steeringWheelRadius;
      double xHandle = distanceFromCenter * Math.cos(angleOnSteeringWheel);
      double yHandle = distanceFromCenter * Math.sin(angleOnSteeringWheel);

      Point3D translation = new Point3D(xHandle, yHandle, distanceFromWheel);
      
      FrameCylinder3d spinnerHandleCylinder = new FrameCylinder3d(steeringWheelFrame, translation, Axis.Z, handleLength, handleRadius);
      spokesCylinders.add(spinnerHandleCylinder);

      steeringWheelLinkGraphics.translate(translation);
      steeringWheelLinkGraphics.addCylinder(handleLength, handleRadius, YoAppearance.IndianRed());
      translation.negate();
      steeringWheelLinkGraphics.translate(translation);

      steeringWheelLink.setLinkGraphics(steeringWheelLinkGraphics);

      spinnerHandleCenter = new FramePoint3D(steeringWheelFrame, xHandle, yHandle, handleLength / 2.0);
   }
   
   public void addCrossBar()
   {
      double height = 2.0 * steeringWheelRadius;
      double radius = 0.015;
      double heightAboveWheel = 0.1;
      
      FramePose3D crossBar = new FramePose3D(steeringWheelFrame);
      GeometryTools.rotatePoseAboutAxis(steeringWheelFrame, Axis.X, Math.PI / 2.0, crossBar);
      GeometryTools.rotatePoseAboutAxis(steeringWheelFrame, Axis.Z, Math.PI / 2.0, crossBar);
      crossBar.setPosition(new Vector3D(-height/2.0, 0.0, heightAboveWheel));
      
      RigidBodyTransform transform = new RigidBodyTransform();
      crossBar.get(transform);
      
      FrameCylinder3d spinnerHandleCylinder = new FrameCylinder3d(steeringWheelFrame, height, radius);
      spinnerHandleCylinder.getCylinder3d().applyTransform(transform);
      spokesCylinders.add(spinnerHandleCylinder);
      
      steeringWheelLinkGraphics.transform(transform);
      steeringWheelLinkGraphics.addCylinder(height, radius, YoAppearance.IndianRed());
      transform.invert();
      steeringWheelLinkGraphics.transform(transform);
      steeringWheelLink.setLinkGraphics(steeringWheelLinkGraphics);
   }

   public void createSteeringWheelRobot()
   {
      YoGraphicsListRegistry graphListRegistry = new YoGraphicsListRegistry();

      steeringWheelFrame = new PoseReferenceFrame("steeringWheelFrame", steeringWheelPoseInWorld);
      steeringWheelFrame.getPose(originalSteeringWheelPose);

      //creating the pinJoint 
      Vector3D jointAxisVector = new Vector3D(0.0, 0.0, 1.0);
      RigidBodyTransform steeringWheelTransformToWorld = new RigidBodyTransform();
      steeringWheelFrame.getTransformToDesiredFrame(steeringWheelTransformToWorld, worldFrame);
      steeringWheelTransformToWorld.transform(jointAxisVector);

      Vector3D steeringWheelPositionInWorld = new Vector3D(steeringWheelPoseInWorld.getPosition());
      steeringWheelPinJoint = new PinJoint("steeringWheelPinJoint", steeringWheelPositionInWorld, this, jointAxisVector);
      steeringWheelPinJoint.setLimitStops(-totalNumberOfPossibleTurns * Math.PI, totalNumberOfPossibleTurns * Math.PI, 1000, 100);
      steeringWheelPinJoint.setDamping(steeringDamping.getDoubleValue());

      //put the graphics frame in the proper orientation
      RotationMatrix rotationMatrix = new RotationMatrix(steeringWheelPoseInWorld.getOrientation());
      steeringWheelLinkGraphics.rotate(rotationMatrix);
      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      rotationTransform.setRotation(rotationMatrix);

      //Creating the physical link for the simulation
      steeringWheelLink = new Link("steerinWheelLink");
      steeringWheelLink.setMass(mass);
      steeringWheelLink.setComOffset(new Vector3D(0.0, 0.0, 0.0));

      inertiaMatrix = RotationalInertiaCalculator.getRotationalInertiaMatrixOfTorus(mass, steeringWheelRadius, steeringWheelThickness);
      steeringWheelLink.setMomentOfInertia(inertiaMatrix);
      steeringWheelPinJoint.setLink(steeringWheelLink);
      this.addRootJoint(steeringWheelPinJoint);

      //torus and offsetCylinder
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBodyTransform invertTransform = new RigidBodyTransform();

      invertTransform.set(transform);
      invertTransform.invert();

      steeringWheelTorus = new FrameTorus3d(steeringWheelFrame, steeringWheelRadius, steeringWheelThickness / 2.0);
      steeringWheelTorus.getTorus3d().applyTransform(transform);
      steeringWheelLinkGraphics.transform(transform);
      steeringWheelLinkGraphics.addArcTorus(0.0, 2 * Math.PI, steeringWheelRadius, steeringWheelThickness / 2.0, YoAppearance.DarkRed());
      steeringWheelLinkGraphics.translate(0.0, 0.0, -steeringColunmLength);
      steeringWheelLinkGraphics.addCylinder(steeringColunmLength, spokesThickness / 2.0, YoAppearance.DarkRed());
      steeringWheelLinkGraphics.translate(0.0, 0.0, steeringColunmLength);
      steeringWheelLinkGraphics.transform(invertTransform);

      Quaternion quat = new Quaternion();
      //spokes
      for (int i = 0; i < 3; i++)
      {
         quat.setYawPitchRoll(0.0, Math.PI / 2.0, i * 2.0 * Math.PI / 4.0 + Math.PI / 2.0);
         transform.setRotation(quat);
         invertTransform.set(transform);
         invertTransform.invert();

         RigidBodyTransform yoGraphicTransform = new RigidBodyTransform(rotationTransform);
         yoGraphicTransform.multiply(transform);

         FrameCylinder3d spokeCylinder = new FrameCylinder3d(steeringWheelFrame, steeringWheelRadius, spokesThickness / 2.0);
         spokeCylinder.applyTransform(transform);
         spokesCylinders.add(spokeCylinder);

         steeringWheelLinkGraphics.transform(transform);
         steeringWheelLinkGraphics.addCylinder(steeringWheelRadius, spokesThickness / 2.0, YoAppearance.DarkRed());
         steeringWheelLinkGraphics.transform(invertTransform);
      }

      //setting the graphics for the link
      steeringWheelLink.setLinkGraphics(steeringWheelLinkGraphics);

      yoGraphicsListRegistries.add(graphListRegistry);

      steeringWheelPinJoint.getQYoVariable().addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double rangeOfMotion = 2 * Math.PI * totalNumberOfPossibleTurns;
            steeringWheelAngleAsAbsolutePercentageOfRangeOfMotion.set((Math.abs(steeringWheelPinJoint.getQYoVariable().getDoubleValue()) / (0.5 * rangeOfMotion)) * 100);
         }
      });
   }

   @Override
   public void updateAllGroundContactPointVelocities()
   {
      RigidBodyTransform pinJointTransform = new RigidBodyTransform();
      RigidBodyTransform newPose = new RigidBodyTransform();
      pinJointTransform.setRotationYawAndZeroTranslation(steeringWheelPinJoint.getQYoVariable().getDoubleValue());
      newPose.set(originalSteeringWheelPose);
      newPose.multiply(pinJointTransform);
      steeringWheelFrame.setPoseAndUpdate(newPose);

      super.updateAllGroundContactPointVelocities();
   }

   private final FramePoint3D pointToCheck = new FramePoint3D();

   @Override
   public boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      pointToCheck.setIncludingFrame(worldFrame, pointInWorldToCheck);
      pointToCheck.changeFrame(steeringWheelFrame);

      if (steeringWheelTorus.isInsideOrOnSurface(pointToCheck))
         return true;
      for (int i = 0; i < spokesCylinders.size(); i++)
      {
         if (spokesCylinders.get(i).isInsideOrOnSurface(pointToCheck))
            return true;
      }
      return false;
   }

   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      FramePoint3D pointToCheck = new FramePoint3D(worldFrame, pointInWorldToCheck);
      pointToCheck.changeFrame(steeringWheelFrame);

      if (steeringWheelTorus.checkIfInside(pointToCheck, intersectionToPack, normalToPack))
         return;
      for (int i = 0; i < spokesCylinders.size(); i++)
      {
         if (spokesCylinders.get(i).checkIfInside(pointToCheck, intersectionToPack, normalToPack))
            return;
      }
   }

   @Override
   public PinJoint getPinJoint()
   {
      return steeringWheelPinJoint;
   }

   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.set(originalSteeringWheelPose);
   }

   public double getSteeringWheelRadius()
   {
      return steeringWheelRadius;
   }

   public double getSteeringWheelAngleAsAbsolutePercentageOfRangeOfMotion()
   {
      return steeringWheelAngleAsAbsolutePercentageOfRangeOfMotion.getDoubleValue();
   }

   public PoseReferenceFrame getSteeringWheelFrame()
   {
      return steeringWheelFrame;
   }

   public FramePoint3D getSpinnerHandleCenter()
   {
      return new FramePoint3D(spinnerHandleCenter);
   }

   public FrameVector3D getSteeringWheelAxis()
   {
      return new FrameVector3D(steeringWheelFrame, 0.0, 0.0, 1.0);
   }

   public FramePoint3D getSteeringWheelCenter()
   {
      return new FramePoint3D(steeringWheelFrame, 0.0, 0.0, 0.0);
   }

   public double getNumberOfPossibleTurns()
   {
      return totalNumberOfPossibleTurns;
   }

   @Override
   public void setMass(double mass)
   {
      this.mass = mass;
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      inertiaMatrix.setM00(Ixx);
      inertiaMatrix.setM01(0.0);
      inertiaMatrix.setM02(0.0);
      inertiaMatrix.setM10(0.0);
      inertiaMatrix.setM11(Iyy);
      inertiaMatrix.setM12(0.0);
      inertiaMatrix.setM20(0.0);
      inertiaMatrix.setM21(0.0);
      inertiaMatrix.setM22(Izz);
   }

   public void setPoseInWorld(FramePose3D poseInWorld)
   {
      this.steeringWheelPoseInWorld.set(poseInWorld);
   }

   public void setDamping(double dampingValue)
   {
      steeringDamping.set(dampingValue);
   }

   public void setSteeringAngleInDegrees(double steeringAngle)
   {
      steeringWheelPinJoint.setQ(steeringAngle);
   }
}
