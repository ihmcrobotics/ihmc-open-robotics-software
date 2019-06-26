package us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameCylinder3d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

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
   
   private YoBoolean buttonStatus;
   private boolean buttonIsSwitchable = true;
   private double buttonSwitchLimit = 0.95;
   
   private Vector3D buttonPushVector;
   private Vector3D buttonOffset;

   private RigidBodyTransform rootJointTransform;

   private SliderJoint buttonSliderJoint;
   private Link buttonLink, caseLink;
   private Graphics3DObject buttonLinkGraphics, caseLinkGraphics;

   private FrameCylinder3d cylinderFrame;
   private PoseReferenceFrame buttonFrame;

   private RigidBodyTransform originalButtonTransform;
   private FramePose3D buttonPoseInWorld;

   public ContactableButtonRobot(String name, RigidBodyTransform rootTransform, Vector3D pushVector)
   {
      this(name, rootTransform, pushVector, DEFAULT_RADIUS, DEFAULT_THICKNESS, DEFAULT_MASS, DEFAULT_DAMPING, DEFAULT_KP, DEFAULT_CASEWIDTH, DEFAULT_CASEDEPTH, DEFAULT_BUTTONLIMIT);
   }

   public ContactableButtonRobot(String name, RigidBodyTransform rootTransform, Vector3D pushVector, double buttonRadius, double buttonThickness, double buttonMass, double buttonDamping, double buttonKp, double caseWidth, double caseDepth, double buttonLimit)
   {
      super(name);
      this.name = name;
      this.buttonStatus = new YoBoolean(this.name + "_Status", yoVariableRegistry);
      buttonStatus.set(true);
     
      this.name = name;
      this.rootJointTransform = rootTransform;
      this.buttonPushVector = new Vector3D(pushVector);
      this.buttonPushVector.normalize();
      buttonPoseInWorld = new FramePose3D();
      originalButtonTransform = new RigidBodyTransform(rootJointTransform); 
      
      // Initialize original button pose to rootJointTransform
      buttonPoseInWorld.set(originalButtonTransform);
      
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
      buttonOffset = new Vector3D();
      rootJointTransform.getTranslation(buttonOffset);

      // Create the buttonSliderJoint
      buttonSliderJoint = new SliderJoint("buttonSliderJoint", buttonOffset, this, buttonPushVector);
      buttonSliderJoint.setLimitStops(0, buttonLimit, 1000.0, 10.0);
      buttonSliderJoint.setDamping(buttonDamping);
      buttonSliderJoint.setKp(buttonKp);

      // Create the buttonLink
      buttonLink = new Link("buttonLink");
      buttonLink.setMass(buttonMass);

      Matrix3D buttonInertiaMatrix = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(buttonMass, buttonRadius, buttonThickness, Axis.X);
      buttonLink.setMomentOfInertia(buttonInertiaMatrix);

      Vector3D buttonComOffset = new Vector3D();
      buttonComOffset.setAndScale(1 / 2.0, buttonPushVector);

      buttonLink.setComOffset(buttonComOffset);
      buttonSliderJoint.setLink(buttonLink);

      this.addRootJoint(buttonSliderJoint);
      cylinderFrame = new FrameCylinder3d(ReferenceFrame.getWorldFrame(), buttonThickness, buttonRadius);
      cylinderFrame.getCylinder3d().applyTransform(rootJointTransform);

      // Create the Graphics
      buttonLinkGraphics = new Graphics3DObject();
      RotationMatrix rotationMatrix = new RotationMatrix();
      rootJointTransform.getRotation(rotationMatrix);

      buttonLinkGraphics.rotate(rotationMatrix);
      buttonLinkGraphics.addCylinder(buttonThickness / 2.0, buttonRadius, YoAppearance.Red());
      buttonLinkGraphics.translate(new Vector3D(0.0, 0.0, buttonThickness / 2.0));
      buttonLinkGraphics.addCylinder(buttonThickness / 2.0 - buttonLimit, buttonRadius, YoAppearance.White());
      buttonLinkGraphics.translate(new Vector3D(0.0, 0.0, buttonThickness / 2.0 - buttonLimit));
      buttonLinkGraphics.addCylinder(buttonLimit, buttonRadius, YoAppearance.Black());

      buttonLink.setLinkGraphics(buttonLinkGraphics);

      // Create the case of the button
      caseLink = new Link("caseLink");
      
      Vector3D caseOffset = new Vector3D(buttonPushVector);
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
               public void notifyOfVariableChange(YoVariable<?> v)
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
      newButtonPose.set(originalButtonTransform);
      newButtonPose.multiply(sliderJointTransform);
      buttonFrame.setPoseAndUpdate(newButtonPose);

      super.updateAllGroundContactPointVelocities();
   }

   @Override
   public boolean isClose(Point3D pointInWorldToCheck) {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      FramePoint3D pointToCheck = new FramePoint3D(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);

      if (cylinderFrame.isInsideOrOnSurface(pointToCheck))
      {
         return true;
      }
      return false;
   }

   @Override
   public void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      cylinderFrame.checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
   }

   @Override
   public SliderJoint getSliderJoint() {

      return buttonSliderJoint;
   }

   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld) {
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
