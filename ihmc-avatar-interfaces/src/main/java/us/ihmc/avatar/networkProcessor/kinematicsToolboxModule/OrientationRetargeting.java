package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.communication.PostureOptimizerState;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.euclid.filters.RateLimitedYoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Retargets the pelvis/chest/arm orientation objectives during multi-contact
 */
public class OrientationRetargeting
{
   private static final boolean VISUALIZE = true;
   private final RateLimitedYoFrameQuaternion desiredOrientationRateLimited;

   private final FrameQuaternion nominalOrientation = new FrameQuaternion();
   private final AxisAngle optimizedOrientation = new AxisAngle();
   private final FrameQuaternion orientationDifference = new FrameQuaternion();

   private final YoFrameQuaternion yoNominalOrientation;
   private final YoFrameQuaternion yoOptimizedOrientation;
   private final YoFrameQuaternion yoActualOrientation;
   private final YoFramePoint3D controlFrameOrigin;

   private final FramePose3D controlFramePose = new FramePose3D();
   private final MovingReferenceFrame controlFrame;
   private final double integrationDT;
   private final double maxAngle;
   private final RigidBodyBasics rigidBody;

   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final Vector3D rotationVector = new Vector3D();
   private final Twist tempTwist = new Twist();
   private final FrameQuaternion tempQuaternion = new FrameQuaternion();
   private final AxisAngle axisAngle = new AxisAngle();
   private final FrameQuaternion differenceToNominal = new FrameQuaternion();

   private final FrameVector3D filteredAngularVelocity = new FrameVector3D();

   private final YoDouble retargetDelta;

   public OrientationRetargeting(String name, RigidBodyBasics rigidBody, double integrationDT, double maxRate, double maxAngle, YoGraphicsListRegistry graphicsListRegistry, YoRegistry registry)
   {
      desiredOrientationRateLimited = new RateLimitedYoFrameQuaternion(name, "DesiredOrientation", registry, maxRate, integrationDT, ReferenceFrame.getWorldFrame());
      this.integrationDT = integrationDT;
      this.maxAngle = maxAngle;
      this.controlFramePose.setToZero(rigidBody.getBodyFixedFrame());
      this.rigidBody = rigidBody;
      this.retargetDelta = new YoDouble(name + "RetargetDelta", registry);

      this.controlFrame = new MovingReferenceFrame(name + "ControlFrameRetarget", rigidBody.getBodyFixedFrame(), null, false, true)
      {
         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
         }

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            controlFramePose.checkReferenceFrameMatch(getParent());
            controlFramePose.get(transformToParent);
         }
      };

      if (VISUALIZE)
      {
         yoNominalOrientation = new YoFrameQuaternion(name + "NominalOrientation", ReferenceFrame.getWorldFrame(), registry);
         yoOptimizedOrientation = new YoFrameQuaternion(name + "OptimizedOrientation", ReferenceFrame.getWorldFrame(), registry);
         yoActualOrientation = new YoFrameQuaternion(name + "ActualOrientation", ReferenceFrame.getWorldFrame(), registry);

         controlFrameOrigin = new YoFramePoint3D(name + "ControlFrameOrigin", ReferenceFrame.getWorldFrame(), registry);

         YoGraphicCoordinateSystem nominalOrientation = new YoGraphicCoordinateSystem(name + "nominalOrientation", controlFrameOrigin, yoNominalOrientation, 0.35, YoAppearance.Green());
         YoGraphicCoordinateSystem optimizedOrientation = new YoGraphicCoordinateSystem(name + "optimizedOrientation", controlFrameOrigin, yoOptimizedOrientation, 0.35, YoAppearance.Red());
         YoGraphicCoordinateSystem actualOrientation = new YoGraphicCoordinateSystem(name + "actualOrientation", controlFrameOrigin, yoActualOrientation, 0.35, YoAppearance.Black());

         graphicsListRegistry.registerYoGraphic("Coordinate Debug", nominalOrientation);
         graphicsListRegistry.registerYoGraphic("Coordinate Debug", optimizedOrientation);
         graphicsListRegistry.registerYoGraphic("Coordinate Debug", actualOrientation);
      }
      else
      {
         yoNominalOrientation = null;
         yoOptimizedOrientation = null;
         yoActualOrientation = null;
      }
   }

   public void initialize()
   {
      controlFrame.update();
      nominalOrientation.setFromReferenceFrame(controlFrame);

      // call once to avoid edge cases in rate limiting
      this.desiredOrientationRateLimited.set(nominalOrientation);
      this.desiredOrientationRateLimited.update(nominalOrientation);

      optimizedOrientation.set(nominalOrientation);
      retargetDelta.set(0.0);
   }

   public void updateNominalOrientation(FrameOrientation3DReadOnly nominalOrientation, FramePose3DReadOnly controlFrame)
   {
      this.nominalOrientation.setIncludingFrame(nominalOrientation);
      this.controlFramePose.setIncludingFrame(controlFrame);
      controlFramePose.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public void update(PostureOptimizerState optimizerState)
   {
      controlFrame.update();

      if (optimizerState == PostureOptimizerState.NOMINAL)
      {
         desiredOrientationRateLimited.update(nominalOrientation);

         // reset optimized to current desired
         optimizedOrientation.set(desiredOrientationRateLimited);
         filteredAngularVelocity.setToZero();
      }
      else if (optimizerState == PostureOptimizerState.OPTIMIZER)
      {
         integrate();

         double filterAlpha = 0.25;
         filteredAngularVelocity.interpolate(angularVelocity, filterAlpha);

         tempQuaternion.set(optimizedOrientation);
         desiredOrientationRateLimited.set(tempQuaternion); // here the integrator acts as the filter
      }
      else
      {
         // freeze, no update (just call this to avoid edge cases in rate limiting)
         desiredOrientationRateLimited.update(desiredOrientationRateLimited);
         filteredAngularVelocity.setToZero();
      }

      if (VISUALIZE)
      {
         controlFrameOrigin.setFromReferenceFrame(controlFrame);
         yoNominalOrientation.set(nominalOrientation);
         yoOptimizedOrientation.set(optimizedOrientation);
         yoActualOrientation.setFromReferenceFrame(controlFrame);
      }

      orientationDifference.difference(nominalOrientation, desiredOrientationRateLimited);
      retargetDelta.set(orientationDifference.angle());
   }

   public FixedFrameQuaternionBasics getDesiredOrientation()
   {
      return desiredOrientationRateLimited;
   }

   public void integrate()
   {
      controlFrame.getTwistRelativeToOther(ReferenceFrame.getWorldFrame(), tempTwist);
      tempTwist.changeFrame(controlFrame);
      angularVelocity.set(ReferenceFrame.getWorldFrame(), tempTwist.getAngularPart());
      rotationVector.setAndScale(integrationDT, angularVelocity);

      axisAngle.setRotationVector(rotationVector);
      optimizedOrientation.multiply(axisAngle);

      // clamp angle relative to nominal, if enabled
      if (maxAngle > 0.0)
      {
         clampToAngle(tempQuaternion, optimizedOrientation, differenceToNominal, nominalOrientation, axisAngle, maxAngle);
      }
   }

   public FrameVector3D getAngularVelocity()
   {
      return filteredAngularVelocity;
   }

   private static void clampToAngle(FrameQuaternion tempQuaternion,
                                    AxisAngle optimizedOrientation,
                                    FrameQuaternion differenceToNominal,
                                    FrameQuaternion nominalOrientation,
                                    AxisAngle axisAngle,
                                    double maxAngle)
   {
      tempQuaternion.set(optimizedOrientation);
      differenceToNominal.difference(tempQuaternion, nominalOrientation);
      axisAngle.set(differenceToNominal);
      axisAngle.setAngle(EuclidCoreTools.clamp(axisAngle.getAngle(), maxAngle));
      differenceToNominal.set(axisAngle);
      QuaternionTools.multiplyConjugateRight(nominalOrientation, differenceToNominal, tempQuaternion);
      optimizedOrientation.set(tempQuaternion);
   }

   public static void main(String[] args)
   {
      // TODO make this into a test

      AxisAngle nominalOrientationAA = new AxisAngle(Axis3D.Z, -0.5);
      AxisAngle optimizedOrientation = new AxisAngle(Axis3D.Z, -0.8);
      FrameQuaternion nominalOrientation = new FrameQuaternion();
      nominalOrientation.set(nominalOrientationAA);
      double maxAngle = 0.1;

      AxisAngle axisAngle = new AxisAngle();
      FrameQuaternion tempQuaternion = new FrameQuaternion();
      FrameQuaternion differenceToNominal = new FrameQuaternion();

      // clamp angle relative to nominal
      clampToAngle(tempQuaternion, optimizedOrientation, differenceToNominal, nominalOrientation, axisAngle, maxAngle);

      System.out.println(optimizedOrientation.getAxis());
      System.out.println(optimizedOrientation.getAngle());
   }
}
