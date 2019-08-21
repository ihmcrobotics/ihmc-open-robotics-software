package us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots;

import java.util.ArrayList;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameCylinder3d;
import us.ihmc.robotics.geometry.shapes.FrameTorus3d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObject;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.environments.ValveType;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class ContactableValveRobot extends ContactablePinJointRobot implements SelectableObject, SelectedListener
{
   private static final double DEFAULT_DAMPING = 3;

   protected double valveRadius;
   private double valveOffsetFromWall;
   private double valveThickness;

   private int numberOfSpokes;
   protected double spokesThickness;

   private FramePose3D valvePoseInWorld = new FramePose3D();

   private double valveNumberOfPossibleTurns;

   private final YoDouble valveClosePercentage;
   
   private double valveMass;
   private Matrix3D inertiaMatrix;

   private FrameTorus3d valveTorus;
   protected ArrayList<FrameCylinder3d> spokesCylinders = new ArrayList<FrameCylinder3d>();

   protected Link valveLink;
   private PinJoint valvePinJoint;
   protected Graphics3DObject valveLinkGraphics = new Graphics3DObject();
   private final YoDouble valveDamping;

   protected PoseReferenceFrame valveFrame;

   private final RigidBodyTransform originalValvePose = new RigidBodyTransform();

   public ContactableValveRobot(String name, double valveRadius, double valveOffsetFromWall, double valveThickness, int numberOfSpokes, double spokesThickness,
         FramePose3D valvePoseInWorld, double valveNumberOfPossibleTurns, double valveMass)
   {
      super(name);
      setValveProperties(valveRadius, valveOffsetFromWall, valveThickness, numberOfSpokes, spokesThickness, valveNumberOfPossibleTurns, valveMass);
      setPoseInWorld(valvePoseInWorld);
      setMass(valveMass);
      valveDamping = new YoDouble(getName() + "ValveDamping", yoVariableRegistry);
      valveDamping.set(DEFAULT_DAMPING);
      valveClosePercentage = new YoDouble("valveClosePercentage", yoVariableRegistry);
      valveClosePercentage.set(0.0);
   }

   public ContactableValveRobot(String name, double valveRadius, double valveOffsetFromWall, double valveThickness, int numberOfSpokes, double spokesThickness,
         Point3D valvePosition, Quaternion valveOrientation, double valveNumberOfPossibleTurns, double valveMass)
   {
      this(name, valveRadius, valveOffsetFromWall, valveThickness, numberOfSpokes, spokesThickness, new FramePose3D(ReferenceFrame.getWorldFrame(),
            valvePosition, valveOrientation), valveNumberOfPossibleTurns, valveMass);
   }

   public ContactableValveRobot(String name, ValveType valveType, double valveNumberOfPossibleTurns, FramePose3D valvePoseInWorld)
   {
      this(name, valveType.getValveRadius(), valveType.getValveOffsetFromWall(), valveType.getValveThickness(), valveType.getNumberOfSpokes(), valveType
            .getSpokesThickness(), valvePoseInWorld, valveNumberOfPossibleTurns, valveType.getValveMass());
   }

   public void setValveProperties(double valveRadius, double valveOffsetFromWall, double valveThickness, int numberOfSpokes, double spokesThickness,
         double valveNumberOfPossibleTurns, double valveMass)
   {
      this.valveRadius = valveRadius;
      this.valveOffsetFromWall = valveOffsetFromWall;
      this.valveThickness = valveThickness;

      this.numberOfSpokes = numberOfSpokes;
      this.spokesThickness = spokesThickness;

      this.valveNumberOfPossibleTurns = valveNumberOfPossibleTurns;

      this.valveMass = valveMass;
   }

   public void createValveRobot()
   {
      YoGraphicsListRegistry graphListRegistry = new YoGraphicsListRegistry();

      valveFrame = new PoseReferenceFrame("valveFrame", valvePoseInWorld);
      valveFrame.getPose(originalValvePose);

      //creating the pinJoint 
      Vector3D jointAxisVector = new Vector3D(1.0, 0.0, 0.0);
      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveFrame.getTransformToDesiredFrame(valveTransformToWorld, ReferenceFrame.getWorldFrame());
      valveTransformToWorld.transform(jointAxisVector);
            
      Vector3D valvePositionInWorld = new Vector3D(valvePoseInWorld.getPosition());
      valvePinJoint = new PinJoint("valvePinJoint", valvePositionInWorld, this, jointAxisVector);
      valvePinJoint.setLimitStops(0.0, valveNumberOfPossibleTurns * 2 * Math.PI, 1000, 100);
      valvePinJoint.setDamping(valveDamping.getDoubleValue());
      
      //put the graphics frame in the proper orientation
      RotationMatrix rotationMatrix = new RotationMatrix(valvePoseInWorld.getOrientation());
      valveLinkGraphics.rotate(rotationMatrix);
      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      rotationTransform.setRotation(rotationMatrix);

      //Creating the physical link for the simulation
      valveLink = new Link("valveLink");
      valveLink.setMass(valveMass);
      valveLink.setComOffset(new Vector3D(0.0, 0.0, 0.0));

      inertiaMatrix = RotationalInertiaCalculator.getRotationalInertiaMatrixOfTorus(valveMass, valveRadius, valveThickness);
      valveLink.setMomentOfInertia(inertiaMatrix);
      valvePinJoint.setLink(valveLink);
      this.addRootJoint(valvePinJoint);
      
      //torus and offsetCylinder
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBodyTransform invertTransform = new RigidBodyTransform();

      transform.setRotationYawPitchRoll(0.0, Math.PI / 2.0, 0.0);
      invertTransform.set(transform);
      invertTransform.invert();

      valveTorus = new FrameTorus3d(valveFrame, valveRadius - valveThickness / 2.0, valveThickness / 2.0);
      valveTorus.applyTransform(transform);
      valveLinkGraphics.transform(transform);
      valveLinkGraphics.addArcTorus(0.0, 2 * Math.PI, valveRadius - valveThickness / 2.0, valveThickness / 2.0, YoAppearance.DarkRed());
      valveLinkGraphics.addCylinder(valveOffsetFromWall, spokesThickness / 2.0, YoAppearance.DarkRed());
      valveLinkGraphics.transform(invertTransform);

      //spokes
      for (int i = 0; i < numberOfSpokes; i++)
      {
         double spokeLength = valveRadius - spokesThickness / 2.0;
         transform.setRotationYawPitchRollAndZeroTranslation(0.0, 0.0, i * 2.0 * Math.PI / numberOfSpokes);
         transform.appendTranslation(0.0, 0.0, -spokeLength);
         invertTransform.set(transform);
         invertTransform.invert();

         RigidBodyTransform yoGraphicTransform = new RigidBodyTransform(rotationTransform);
         yoGraphicTransform.multiply(transform);

         FrameCylinder3d spokeCylinder = new FrameCylinder3d(valveFrame, spokeLength, spokesThickness / 2.0);
         spokeCylinder.getCylinder3d().applyTransform(transform);
         spokesCylinders.add(spokeCylinder);

         valveLinkGraphics.transform(transform);
         valveLinkGraphics.addCylinder(spokeLength, spokesThickness / 2.0, YoAppearance.DarkRed());
         valveLinkGraphics.transform(invertTransform);
      }

      //setting the graphics for the link
      valveLink.setLinkGraphics(valveLinkGraphics);

      yoGraphicsListRegistries.add(graphListRegistry);
      
      valvePinJoint.getQYoVariable().addVariableChangedListener(new VariableChangedListener()
      {
         
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            valveClosePercentage.set(valvePinJoint.getQYoVariable().getDoubleValue()/(2*Math.PI)*100/valveNumberOfPossibleTurns);
         }
      });
   }

   @Override
   public void updateAllGroundContactPointVelocities()
   {
      RigidBodyTransform pinJointTransform = new RigidBodyTransform();
      RigidBodyTransform newValvePose = new RigidBodyTransform();
      pinJointTransform.setRotationRollAndZeroTranslation(valvePinJoint.getQYoVariable().getDoubleValue());
      newValvePose.set(originalValvePose);
      newValvePose.multiply(pinJointTransform);
      valveFrame.setPoseAndUpdate(newValvePose);

      super.updateAllGroundContactPointVelocities();
   }

   private final FramePoint3D pointToCheck = new FramePoint3D();
   @Override
   public boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      pointToCheck.setIncludingFrame(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);
      pointToCheck.changeFrame(valveFrame);

      if (valveTorus.isInsideOrOnSurface(pointToCheck))
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
      FramePoint3D pointToCheck = new FramePoint3D(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);
      pointToCheck.changeFrame(valveFrame);

      if (valveTorus.checkIfInside(pointToCheck, intersectionToPack, normalToPack))
         return;
      for (int i = 0; i < spokesCylinders.size(); i++)
      {
         if (spokesCylinders.get(i).checkIfInside(pointToCheck, intersectionToPack, normalToPack))
            return;
      }
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3DReadOnly location, Point3DReadOnly cameraLocation,
         QuaternionReadOnly cameraRotation)
   {

   }

   @Override
   public void select()
   {

   }

   @Override
   public void unSelect(boolean reset)
   {

   }

   @Override
   public void addSelectedListeners(SelectableObjectListener selectedListener)
   {

   }
   
   @Override
   public PinJoint getPinJoint()
   {
      return valvePinJoint;
   }
   
   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.set(originalValvePose);
   }

   public double getValveRadius()
   {
      return valveRadius;
   }
   
   public double getClosePercentage()
   {
      return valveClosePercentage.getDoubleValue();
   }
   
   public double getNumberOfPossibleTurns()
   {
      return valveNumberOfPossibleTurns;
   }
   
   @Override
   public void setMass(double mass)
   {
      this.valveMass = mass;
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

   public void setPoseInWorld(FramePose3D valvePoseInWorld)
   {
      this.valvePoseInWorld.set(valvePoseInWorld);
   }

   public void setPoseInWorld(Point3D position, Quaternion orientation)
   {
      this.valvePoseInWorld.set(position, orientation);
   }

   public void setDamping(double dampingValue)
   {
      valveDamping.set(dampingValue);
   }

   public void setClosePercentage(double percentage)
   {
      valveClosePercentage.set(percentage);
      valvePinJoint.setQ(valveNumberOfPossibleTurns* 2 * Math.PI * percentage/100 );
   }
}
