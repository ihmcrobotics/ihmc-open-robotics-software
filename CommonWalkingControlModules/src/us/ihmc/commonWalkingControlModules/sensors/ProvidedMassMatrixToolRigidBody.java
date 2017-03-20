package us.ihmc.commonWalkingControlModules.sensors;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * @author unknownid
 *
 */
public class ProvidedMassMatrixToolRigidBody
{
   private final YoVariableRegistry registry;

   private final PoseReferenceFrame toolFrame;
   private final RigidBody toolBody;

   private final ReferenceFrame handFixedFrame;
   private final ReferenceFrame handControlFrame;

   private final YoFramePoint objectCenterOfMass;
   private final YoFramePoint objectCenterOfMassInWorld;
   private final YoFrameVector objectForceInWorld;

   private final DoubleYoVariable objectMass;

   private final FullRobotModel fullRobotModel;
   private final double gravity;
   private InverseDynamicsCalculator inverseDynamicsCalculator;
   private final SixDoFJoint toolJoint;
   private final ReferenceFrame elevatorFrame;

   private final FramePoint temporaryPoint = new FramePoint();
   private final FrameVector temporaryVector = new FrameVector();
   private final SpatialAccelerationVector toolAcceleration = new SpatialAccelerationVector();

   public ProvidedMassMatrixToolRigidBody(RobotSide robotSide, final FullHumanoidRobotModel fullRobotModel, double gravity, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String name = robotSide.getCamelCaseNameForStartOfExpression() + "Tool";
      this.registry = new YoVariableRegistry(name);
      this.fullRobotModel = fullRobotModel;
      this.gravity = gravity;

      this.handFixedFrame = fullRobotModel.getHand(robotSide).getBodyFixedFrame();
      this.handControlFrame = fullRobotModel.getHandControlFrame(robotSide);

      this.elevatorFrame = fullRobotModel.getElevatorFrame();
      toolFrame = new PoseReferenceFrame(name + "Frame", elevatorFrame);

      RigidBodyInertia inertia = new RigidBodyInertia(toolFrame, new Matrix3D(), 0.0);

      this.toolJoint = new SixDoFJoint(name + "Joint", fullRobotModel.getElevator(), fullRobotModel.getElevator().getBodyFixedFrame());
      this.toolBody = new RigidBody(name + "Body", inertia, toolJoint);

      objectCenterOfMass = new YoFramePoint(name + "CoMOffset", handControlFrame, registry);
      objectMass = new DoubleYoVariable(name + "ObjectMass", registry);
      objectForceInWorld = new YoFrameVector(name + "Force", ReferenceFrame.getWorldFrame(), registry);

      this.objectCenterOfMassInWorld = new YoFramePoint(name + "CoMInWorld", ReferenceFrame.getWorldFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {

         YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
         YoGraphic comViz = new YoGraphicPosition(name + "CenterOfMassViz", objectCenterOfMassInWorld, 0.05, YoAppearance.Red());
         yoGraphicsList.add(comViz);

         YoGraphic vectorViz = new YoGraphicVector(name + "ForceViz", objectCenterOfMassInWorld, objectForceInWorld, YoAppearance.Yellow());
         yoGraphicsList.add(vectorViz);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      }
      parentRegistry.addChild(registry);
   }

   private boolean hasBeenInitialized = false;

   private void initialize()
   {
      hasBeenInitialized = true;

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), toolBody);

      boolean doVelocityTerms = true;
      boolean useDesireds = false;
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(toolBody, elevatorFrame,
            ScrewTools.createGravitationalSpatialAcceleration(fullRobotModel.getElevator(), gravity), twistCalculator, doVelocityTerms, useDesireds);

      ArrayList<InverseDynamicsJoint> jointsToIgnore = new ArrayList<InverseDynamicsJoint>();
      jointsToIgnore.addAll(twistCalculator.getRootBody().getChildrenJoints());
      jointsToIgnore.remove(toolJoint);

      inverseDynamicsCalculator = new InverseDynamicsCalculator(ReferenceFrame.getWorldFrame(), new LinkedHashMap<RigidBody, Wrench>(), jointsToIgnore,
            spatialAccelerationCalculator, twistCalculator, doVelocityTerms);
   }

   private final FramePoint toolFramePoint = new FramePoint();

   public void update()
   {
      toolBody.getInertia().setMass(objectMass.getDoubleValue());

      temporaryPoint.setIncludingFrame(objectCenterOfMass.getFrameTuple());
      temporaryPoint.changeFrame(elevatorFrame);
      toolFrame.setPositionAndUpdate(temporaryPoint);

      // Visualization
      toolFramePoint.setToZero(toolFrame);
      toolFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      objectCenterOfMassInWorld.set(toolFramePoint);
   }

   public void control(SpatialAccelerationVector handSpatialAccelerationVector, Wrench toolWrench)
   {
      if (!hasBeenInitialized)
      {
         update();
         initialize();
      }

      update();

      toolAcceleration.set(handSpatialAccelerationVector);
      toolAcceleration.changeFrameNoRelativeMotion(toolJoint.getFrameAfterJoint());

      // TODO: Take relative acceleration between uTorsoCoM and elevator in account
      toolAcceleration.changeBaseFrameNoRelativeAcceleration(elevatorFrame);
      toolAcceleration.changeBodyFrameNoRelativeAcceleration(toolJoint.getFrameAfterJoint());

      toolJoint.setDesiredAcceleration(toolAcceleration);
      inverseDynamicsCalculator.compute();
      inverseDynamicsCalculator.getJointWrench(toolJoint, toolWrench);

      toolWrench.negate();
      toolWrench.changeFrame(handFixedFrame);
      toolWrench.changeBodyFrameAttachedToSameBody(handFixedFrame);

      // Visualization
      temporaryVector.setIncludingFrame(handFixedFrame, toolWrench.getLinearPartX(), toolWrench.getLinearPartY(), toolWrench.getLinearPartZ());
      temporaryVector.changeFrame(ReferenceFrame.getWorldFrame());
      temporaryVector.scale(0.01);
      objectForceInWorld.set(temporaryVector);

   }

   public void setMass(double mass)
   {
      objectMass.set(mass);
   }
}
