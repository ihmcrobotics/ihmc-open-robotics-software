package us.ihmc.simulationconstructionset.util.environments;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameCylinder3d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ContactableButtonRobot extends ContactableSliderJointRobot {

   private String name;

   private static final double DEFAULT_RADIUS = 0.035 / 2.0;     // [m]
   private static final double DEFAULT_THICKNESS = 0.05;         // [m]
   private static final double DEFAULT_BUTTONLIMIT = 0.005;      // [m]
   private static final double DEFAULT_MASS = 0.05;              // [kg]
   private static final double DEFAULT_DAMPING = 0.5;            // [N s / m]
   private static final double DEFAULT_KP = 500;                 // [N / m]
   private static final double DEFAULT_CASEWIDTH = 0.2;          // [m]
   private static final double DEFAULT_CASEDEPTH = 0.1;          // [m]

   private double buttonRadius;
   private double buttonThickness;
   private double buttonLimit;
   private double buttonMass;
   private double buttonDamping;
   private double buttonKp;
   private double caseWidth;
   private double caseDepth;
   
   private BooleanYoVariable buttonStatus;
   private boolean buttonIsSwitchable = true;
   private double buttonSwitchLimit = 0.95;
   
   private Vector3d buttonPushVector;
   private Vector3d buttonOffset;

   private RigidBodyTransform rootJointTransform;

   private SliderJoint buttonSliderJoint;
   private Link buttonLink, caseLink;
   private Graphics3DObject buttonLinkGraphics, caseLinkGraphics;

   private FrameCylinder3d cylinderFrame;
   private PoseReferenceFrame buttonFrame;

   private RigidBodyTransform originalButtonTransform;
   private FramePose buttonPoseInWorld;

   public ContactableButtonRobot(String name, RigidBodyTransform rootTransform, Vector3d pushVector)
   {
      this(name, rootTransform, pushVector, DEFAULT_RADIUS, DEFAULT_THICKNESS, DEFAULT_MASS, DEFAULT_DAMPING, DEFAULT_KP, DEFAULT_CASEWIDTH, DEFAULT_CASEDEPTH, DEFAULT_BUTTONLIMIT);
   }

   public ContactableButtonRobot(String name, RigidBodyTransform rootTransform, Vector3d pushVector, double buttonRadius, double buttonThickness, double buttonMass, double buttonDamping, double buttonKp, double caseWidth, double caseDepth, double buttonLimit)
   {
      super(name);
      this.name = name;
      this.buttonStatus = new BooleanYoVariable(this.name + "_Status", yoVariableRegistry);
      buttonStatus.set(true);
     
      this.name = name;
      this.rootJointTransform = rootTransform;
      this.buttonPushVector = new Vector3d(pushVector);
      this.buttonPushVector.normalize();
      buttonPoseInWorld = new FramePose();
      originalButtonTransform = new RigidBodyTransform(rootJointTransform); 
      
      // Initialize original button pose to rootJointTransform
      buttonPoseInWorld.setPose(originalButtonTransform);
      
      // Create a new reference frame attached to the button
      buttonFrame =  new PoseReferenceFrame("buttonFrame", buttonPoseInWorld);

      setButtonProperties(buttonRadius, buttonThickness, buttonMass, buttonDamping, buttonKp, caseWidth, caseDepth, buttonLimit);
   }

   public void setButtonProperties(double buttonRadius, double buttonThickness, double buttonMass, double buttonDamping, double buttonKp, double caseWidth, double caseDepth, double buttonLimit)
   {
      this.buttonRadius = buttonRadius;
      this.buttonThickness = buttonThickness;
      this.buttonMass = buttonMass;
      this.buttonDamping = buttonDamping;
      this.buttonKp = buttonKp;
      this.caseWidth = caseWidth;
      this.caseDepth = caseDepth;
      this.buttonLimit = buttonLimit;
   }

   public void createButtonRobot() {

      // Transform the button axis with the rootTransform
      buttonOffset = new Vector3d();
      rootJointTransform.getTranslation(buttonOffset);

      // Create the buttonSliderJoint
      buttonSliderJoint = new SliderJoint("buttonSliderJoint", buttonOffset, this, buttonPushVector);
      buttonSliderJoint.setLimitStops(0, buttonLimit, 1000.0, 10.0);
      buttonSliderJoint.setDamping(buttonDamping);
      buttonSliderJoint.setKp(buttonKp);

      // Create the buttonLink
      buttonLink = new Link("buttonLink");
      buttonLink.setMass(buttonMass);

      Matrix3d buttonInertiaMatrix = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(buttonMass, buttonRadius, buttonThickness, Axis.X);
      buttonLink.setMomentOfInertia(buttonInertiaMatrix);

      Vector3d buttonComOffset = new Vector3d();
      buttonComOffset.scale(1 / 2.0, buttonPushVector);

      buttonLink.setComOffset(buttonComOffset);
      buttonSliderJoint.setLink(buttonLink);

      this.addRootJoint(buttonSliderJoint);
      cylinderFrame = new FrameCylinder3d(ReferenceFrame.getWorldFrame(), rootJointTransform, buttonThickness, buttonRadius);

      // Create the Graphics
      buttonLinkGraphics = new Graphics3DObject();
      Matrix3d rotationMatrix = new Matrix3d();
      rootJointTransform.getRotation(rotationMatrix);

      buttonLinkGraphics.rotate(rotationMatrix);
      buttonLinkGraphics.addCylinder(buttonThickness / 2.0, buttonRadius, YoAppearance.Red());
      buttonLinkGraphics.translate(new Vector3d(0.0, 0.0, buttonThickness / 2.0));
      buttonLinkGraphics.addCylinder(buttonThickness / 2.0 - buttonLimit, buttonRadius, YoAppearance.White());
      buttonLinkGraphics.translate(new Vector3d(0.0, 0.0, buttonThickness / 2.0 - buttonLimit));
      buttonLinkGraphics.addCylinder(buttonLimit, buttonRadius, YoAppearance.Black());

      buttonLink.setLinkGraphics(buttonLinkGraphics);

      // Create the case of the button
      caseLink = new Link("caseLink");
      
      Vector3d caseOffset = new Vector3d(buttonPushVector);
      caseOffset.scale(buttonThickness);
      caseOffset.add(buttonOffset);
      rootJointTransform.getRotation(rotationMatrix);

      caseLinkGraphics = new Graphics3DObject();
      caseLinkGraphics.translate(caseOffset);
      caseLinkGraphics.rotate(rotationMatrix);
      caseLinkGraphics.addCube(caseWidth, caseWidth, caseDepth, YoAppearance.Yellow());
      caseLink.setLinkGraphics(caseLinkGraphics);
      
      this.addStaticLink(caseLink);

      // Add listener to the button
      buttonSliderJoint.getQYoVariable().addVariableChangedListener(
            new VariableChangedListener()
            {
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  if (buttonIsSwitchable == true && buttonSliderJoint.getQYoVariable().getDoubleValue() > buttonLimit * buttonSwitchLimit)
                  {
                     // Button is switched
                     if(buttonStatus.getBooleanValue() == true)
                     {
                        buttonStatus.set(false);
                        buttonIsSwitchable = false;
                     }
   
                     else if(buttonStatus.getBooleanValue() == false)
                     {
                        buttonStatus.set(true);
                        buttonIsSwitchable = false;
                     }
                  }
                  
                  else if (buttonIsSwitchable == false && buttonSliderJoint.getQYoVariable().getDoubleValue() < 0.0005)
                  {
                     // Button is released, can be switched again
                     buttonIsSwitchable = true;
                  }
               }
            }
            );
   }
   
   public boolean getButtonStatus()
   {
      return buttonStatus.getBooleanValue();
   }

   @Override
   public void updateAllGroundContactPointVelocities() {
      RigidBodyTransform sliderJointTransform = new RigidBodyTransform();
      RigidBodyTransform newButtonPose = new RigidBodyTransform();
      buttonPushVector.scale(buttonSliderJoint.getQYoVariable().getDoubleValue());
      sliderJointTransform.setTranslationAndIdentityRotation(buttonPushVector);
      buttonPushVector.normalize();
      newButtonPose.multiply(originalButtonTransform, sliderJointTransform);
      buttonFrame.setPoseAndUpdate(newButtonPose);

      super.updateAllGroundContactPointVelocities();
   }

   @Override
   public boolean isClose(Point3d pointInWorldToCheck) {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public boolean isPointOnOrInside(Point3d pointInWorldToCheck)
   {
      FramePoint pointToCheck = new FramePoint(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);

      if (cylinderFrame.isInsideOrOnSurface(pointToCheck))
      {
         return true;
      }
      return false;
   }

   @Override
   public void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
   {
      cylinderFrame.checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
   }

   @Override
   public SliderJoint getSliderJoint() {

      return buttonSliderJoint;
   }

   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld) {
      // TODO Auto-generated method stub
   }

   @Override
   public void setMass(double buttonMass) {
      this.buttonMass =  buttonMass;
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz) {
      this.buttonLink.setMomentOfInertia(Ixx, Iyy, Izz);
   }

}
