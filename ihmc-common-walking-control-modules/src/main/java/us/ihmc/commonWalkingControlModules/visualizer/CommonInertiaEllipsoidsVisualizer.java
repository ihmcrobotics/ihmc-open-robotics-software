package us.ihmc.commonWalkingControlModules.visualizer;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.robotics.robotDescription.InertiaTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CommonInertiaEllipsoidsVisualizer implements Updatable, RobotController
{
   private final String name = getClass().getSimpleName();

   private final YoRegistry registry = new YoRegistry(name);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameVector3D inertiaEllipsoidGhostOffset = new YoFrameVector3D("inertiaEllipsoidGhostOffset", "", worldFrame, registry);

   private final ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>();

   private final YoDouble minimumMassOfRigidBodies = new YoDouble("minimumMassOfRigidBodies", registry);
   private final YoDouble maximumMassOfRigidBodies = new YoDouble("maximumMassOfRigidBodies", registry);

   private class RigidBodyVisualizationData
   {
      public RigidBodyBasics rigidBody;
      public YoFramePoint3D position;
      public YoFrameYawPitchRoll orientation;

      public RigidBodyVisualizationData(RigidBodyBasics rigidBody, YoFramePoint3D position, YoFrameYawPitchRoll orientation)
      {
         this.rigidBody = rigidBody;
         this.position = position;
         this.orientation = orientation;
      }

   }

   private final ArrayList<RigidBodyVisualizationData> centerOfMassData = new ArrayList<RigidBodyVisualizationData>();

   public CommonInertiaEllipsoidsVisualizer(RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this(rootBody, yoGraphicsListRegistry);
      parentRegistry.addChild(registry);
   }

   public CommonInertiaEllipsoidsVisualizer(RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      inertiaEllipsoidGhostOffset.set(0, 0.0, 0.0);

      findMinimumAndMaximumMassOfRigidBodies(rootBody);
      addRigidBodyAndChilderenToVisualization(rootBody);
      yoGraphicsListRegistry.registerYoGraphics(name, yoGraphics);
      update();

   }

   private void findMinimumAndMaximumMassOfRigidBodies(RigidBodyBasics body)
   {
      SpatialInertiaBasics inertia = body.getInertia();
      if (inertia != null)
      {
         double mass = body.getInertia().getMass();

         if (mass < minimumMassOfRigidBodies.getDoubleValue() && mass > 1e-3)
            minimumMassOfRigidBodies.set(mass);

         if (mass > maximumMassOfRigidBodies.getDoubleValue())
            maximumMassOfRigidBodies.set(mass);

      }

      if (body.hasChildrenJoints())
      {
         List<? extends JointBasics> childJoints = body.getChildrenJoints();

         for (JointBasics joint : childJoints)
         {
            RigidBodyBasics nextBody = joint.getSuccessor();
            if (nextBody != null)
               findMinimumAndMaximumMassOfRigidBodies(nextBody);
         }

      }
   }

   public Color getColor(double mass)
   {
      // Color from 
      // http://stackoverflow.com/questions/340209/generate-colors-between-red-and-green-for-a-power-meter

      if (mass < minimumMassOfRigidBodies.getDoubleValue())
         mass = minimumMassOfRigidBodies.getDoubleValue();

      float massScale = (float) ((mass - minimumMassOfRigidBodies.getDoubleValue())
            / (maximumMassOfRigidBodies.getDoubleValue() - minimumMassOfRigidBodies.getDoubleValue()));

      float H = (1.0f - massScale) * 0.4f;
      float S = 0.9f;
      float B = 0.9f;

      return Color.getHSBColor(H, S, B);
   }

   private void addRigidBodyAndChilderenToVisualization(RigidBodyBasics currentRigidBody)
   {

      SpatialInertiaBasics inertia = currentRigidBody.getInertia();

      if (inertia != null)
      {
         Matrix3D inertiaMatrix = new Matrix3D(inertia.getMomentOfInertia());
         double mass = inertia.getMass();

         //         Vector3d principalMomentsOfInertia = new Vector3d(inertiaMatrix.m00, inertiaMatrix.m11, inertiaMatrix.m22);
         //         Vector3d radii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);
         //         if(radii.length() > 1e-4)
         //         {
         String rigidBodyName = currentRigidBody.getName();
         YoFramePoint3D comPosition = new YoFramePoint3D("centerOfMassPosition", rigidBodyName, worldFrame, registry);
         YoFrameYawPitchRoll comOrientation = new YoFrameYawPitchRoll("rigidBodyOrientation", rigidBodyName, worldFrame, registry);
         RigidBodyVisualizationData comData = new RigidBodyVisualizationData(currentRigidBody, comPosition, comOrientation);
         centerOfMassData.add(comData);

         AppearanceDefinition appearance = YoAppearance.Color(getColor(currentRigidBody.getInertia().getMass()));
         appearance.setTransparency(0.6);

         //            new YoGraphicShape(rigidBodyName, linkGraphics, framePose, scale)
         YoGraphicShape comViz = new YoGraphicShape(rigidBodyName + "CoMEllipsoid", createEllipsoid(inertiaMatrix, mass, appearance), comPosition,
               comOrientation, 1.0);
         yoGraphics.add(comViz);
         //         }
      }

      if (currentRigidBody.hasChildrenJoints())
      {
         List<? extends JointBasics> childJoints = currentRigidBody.getChildrenJoints();

         for (JointBasics joint : childJoints)
         {
            RigidBodyBasics nextRigidBody = joint.getSuccessor();
            if (nextRigidBody != null)
               addRigidBodyAndChilderenToVisualization(nextRigidBody);
         }

      }

   }

   public void update(double time)
   {
      update();
   }

   public void update()
   {

      FramePoint3D tempCoMPosition = new FramePoint3D(worldFrame);
      for (RigidBodyVisualizationData comData : centerOfMassData)
      {
         comData.rigidBody.getCenterOfMass(tempCoMPosition);
         tempCoMPosition.changeFrame(worldFrame);
         tempCoMPosition.add(inertiaEllipsoidGhostOffset);

         FrameQuaternion orientation = new FrameQuaternion(comData.rigidBody.getBodyFixedFrame());
         orientation.changeFrame(worldFrame);

         comData.position.set(tempCoMPosition);
         comData.orientation.set(orientation);
      }
   }

   public void initialize()
   {
      update();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      update();
   }

   private Graphics3DObject createEllipsoid(Matrix3D inertia, double mass, AppearanceDefinition appearance)
   {
      RotationMatrix rotationMatrix3d = new RotationMatrix();
      Vector3D principalMomentsOfInertiaToPack = new Vector3D();
      InertiaTools.computePrincipalMomentsOfInertia(inertia, rotationMatrix3d, principalMomentsOfInertiaToPack);

      double a = 5.0 * principalMomentsOfInertiaToPack.getX() / mass;
      double b = 5.0 * principalMomentsOfInertiaToPack.getY() / mass;
      double c = 5.0 * principalMomentsOfInertiaToPack.getZ() / mass;
      double rx = Math.sqrt(0.5 * (-a + b + c));
      double ry = Math.sqrt(0.5 * (a - b + c));
      double rz = Math.sqrt(0.5 * (a + b - c));

      Graphics3DObject graphics = new Graphics3DObject();
      graphics.identity();
      graphics.rotate(rotationMatrix3d);
      graphics.addEllipsoid(rx, ry, rz, appearance);

      return graphics;
   }
}
