package us.ihmc.commonWalkingControlModules.visualizer;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;

public class CommonInertiaEllipsoidsVisualizer implements Updatable, RobotController
{
   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameVector inertiaEllipsoidGhostOffset = new YoFrameVector("inertiaEllipsoidGhostOffset", "", worldFrame, registry);

   private final ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>();

   private final DoubleYoVariable minimumMassOfRigidBodies = new DoubleYoVariable("minimumMassOfRigidBodies", registry);
   private final DoubleYoVariable maximumMassOfRigidBodies = new DoubleYoVariable("maximumMassOfRigidBodies", registry);

   private class RigidBodyVisualizationData
   {
      public RigidBody rigidBody;
      public YoFramePoint position;
      public YoFrameOrientation orientation;

      public RigidBodyVisualizationData(RigidBody rigidBody, YoFramePoint position, YoFrameOrientation orientation)
      {
         this.rigidBody = rigidBody;
         this.position = position;
         this.orientation = orientation;
      }

   }

   private final ArrayList<RigidBodyVisualizationData> centerOfMassData = new ArrayList<RigidBodyVisualizationData>();

   public CommonInertiaEllipsoidsVisualizer(RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(rootBody, yoGraphicsListRegistry);
      parentRegistry.addChild(registry);
   }

   public CommonInertiaEllipsoidsVisualizer(RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      inertiaEllipsoidGhostOffset.set(0, 0.0, 0.0);

      findMinimumAndMaximumMassOfRigidBodies(rootBody);
      addRigidBodyAndChilderenToVisualization(rootBody);
      yoGraphicsListRegistry.registerYoGraphics(name, yoGraphics);
      update();

   }

   private void findMinimumAndMaximumMassOfRigidBodies(RigidBody body)
   {
      RigidBodyInertia inertia = body.getInertia();
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
         List<InverseDynamicsJoint> childJoints = body.getChildrenJoints();

         for (InverseDynamicsJoint joint : childJoints)
         {
            RigidBody nextBody = joint.getSuccessor();
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

   private void addRigidBodyAndChilderenToVisualization(RigidBody currentRigidBody)
   {

      RigidBodyInertia inertia = currentRigidBody.getInertia();

      if (inertia != null)
      {
         Matrix3D inertiaMatrix = inertia.getMassMomentOfInertiaPartCopy();
         double mass = inertia.getMass();

         //         Vector3d principalMomentsOfInertia = new Vector3d(inertiaMatrix.m00, inertiaMatrix.m11, inertiaMatrix.m22);
         //         Vector3d radii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);
         //         if(radii.length() > 1e-4)
         //         {
         String rigidBodyName = currentRigidBody.getName();
         YoFramePoint comPosition = new YoFramePoint("centerOfMassPosition", rigidBodyName, worldFrame, registry);
         YoFrameOrientation comOrientation = new YoFrameOrientation("rigidBodyOrientation", rigidBodyName, worldFrame, registry);
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
         List<InverseDynamicsJoint> childJoints = currentRigidBody.getChildrenJoints();

         for (InverseDynamicsJoint joint : childJoints)
         {
            RigidBody nextRigidBody = joint.getSuccessor();
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

      FramePoint tempCoMPosition = new FramePoint(worldFrame);
      for (RigidBodyVisualizationData comData : centerOfMassData)
      {
         comData.rigidBody.getCoMOffset(tempCoMPosition);
         tempCoMPosition.changeFrame(worldFrame);
         tempCoMPosition.add(inertiaEllipsoidGhostOffset.getFrameVectorCopy());

         FrameOrientation orientation = new FrameOrientation(comData.rigidBody.getBodyFixedFrame());
         orientation.changeFrame(worldFrame);

         comData.position.set(tempCoMPosition);
         comData.orientation.set(orientation);
      }
   }

   public void initialize()
   {
      update();
   }

   public YoVariableRegistry getYoVariableRegistry()
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
