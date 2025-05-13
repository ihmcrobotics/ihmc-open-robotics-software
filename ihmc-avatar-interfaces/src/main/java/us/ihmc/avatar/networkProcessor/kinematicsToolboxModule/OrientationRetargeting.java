package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.communication.PostureOptimizerState;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Retargets the pelvis/chest/arm orientation objectives during multi-contact
 */
public class OrientationRetargeting
{
   static final boolean VISUALIZE = true;
   static final double BLEND_DURATION = 1.0;
   private final YoFrameQuaternion desiredOrientation;

   private final FramePoint3D nominalPosition = new FramePoint3D();
   private final FramePoint3D desiredPosition = new FramePoint3D();

   private final FrameQuaternion nominalOrientation = new FrameQuaternion();
   private final AxisAngle optimizedOrientation = new AxisAngle();
   private final FrameQuaternion orientationDifference = new FrameQuaternion();

   private final YoFrameQuaternion yoNominalOrientation;
   private final YoFrameQuaternion yoOptimizedOrientation;
   private final YoFrameQuaternion yoActualOrientation;
   private final YoFramePoint3D yoNominalPosition;
   private final YoFramePoint3D yoDesiredPosition;

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

   private final FrameQuaternion blendOrientation = new FrameQuaternion();
   private final FrameVector3D filteredAngularVelocity = new FrameVector3D();

   private final YoDouble retargetDelta;
   private double blendStartTime;
   private final DoubleProvider timeProvider;

   private boolean updatePositionsFromControlFrame = true;
   private final FrameVector3D offset = new FrameVector3D();

   public OrientationRetargeting(String name,
                                 RigidBodyBasics rigidBody,
                                 double integrationDT,
                                 DoubleProvider timeProvider,
                                 double maxAngle,
                                 FramePose3DReadOnly defaultControlFramePoseInBody,
                                 YoGraphicsListRegistry graphicsListRegistry,
                                 YoRegistry registry)
   {
      desiredOrientation = new YoFrameQuaternion(name, ReferenceFrame.getWorldFrame(), registry);
      this.integrationDT = integrationDT;
      this.maxAngle = maxAngle;
      this.controlFramePose.setIncludingFrame(rigidBody.getBodyFixedFrame(), defaultControlFramePoseInBody);
      this.rigidBody = rigidBody;
      this.retargetDelta = new YoDouble(name + "RetargetDelta", registry);
      this.timeProvider = timeProvider;

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
         //         yoActualOrientation = new YoFrameQuaternion(name + "ActualOrientation", ReferenceFrame.getWorldFrame(), registry);
         yoActualOrientation = null;

         yoNominalPosition = new YoFramePoint3D(name + "ControlFrameOrigin", ReferenceFrame.getWorldFrame(), registry);
         yoDesiredPosition = new YoFramePoint3D(name + "DesiredFrameOrigin", ReferenceFrame.getWorldFrame(), registry);

         YoGraphicCoordinateSystem nominalOrientation = new YoGraphicCoordinateSystem(name + "nominalOrientation", yoNominalPosition,
                                                                                      yoNominalOrientation,
                                                                                      0.35,
                                                                                      YoAppearance.Green());
         YoGraphicCoordinateSystem desiredOrientation = new YoGraphicCoordinateSystem(name + "desiredOrientation", yoDesiredPosition, this.desiredOrientation,
                                                                                        0.35,
                                                                                        YoAppearance.Red());
         //         YoGraphicCoordinateSystem actualOrientation = new YoGraphicCoordinateSystem(name + "actualOrientation", controlFrameOrigin, yoActualOrientation, 0.35, YoAppearance.Black());

         graphicsListRegistry.registerYoGraphic("Coordinate Debug", nominalOrientation);
         graphicsListRegistry.registerYoGraphic("Coordinate Debug", desiredOrientation);
         //         graphicsListRegistry.registerYoGraphic("Coordinate Debug", actualOrientation);
      }
      else
      {
         yoNominalOrientation = null;
         yoOptimizedOrientation = null;
         yoActualOrientation = null;
         yoNominalPosition = null;
      }
   }

   public void initialize()
   {
      controlFrame.update();
      nominalOrientation.setFromReferenceFrame(controlFrame);

      this.desiredOrientation.set(nominalOrientation);
      blendOrientation.set(nominalOrientation);

      optimizedOrientation.set(nominalOrientation);
      retargetDelta.set(0.0);
   }

   public void updateNominalOrientation(FrameOrientation3DReadOnly nominalOrientation, boolean hasDesiredVelocity, SpatialVector desiredVelocity, FramePose3DReadOnly controlFrame)
   {
      this.nominalOrientation.setIncludingFrame(nominalOrientation);

      this.controlFramePose.setIncludingFrame(controlFrame);
      controlFramePose.changeFrame(rigidBody.getBodyFixedFrame());

      if (hasDesiredVelocity)
         filteredAngularVelocity.set(desiredVelocity.getAngularPart());
   }

   public void updateNominalPosition(FramePoint3DReadOnly nominalPosition)
   {
      this.nominalPosition.setIncludingFrame(nominalPosition);
   }

   public void updateDesiredPosition(FramePoint3DReadOnly desiredPosition)
   {
      this.desiredPosition.setIncludingFrame(desiredPosition);
      updatePositionsFromControlFrame = false;
   }

   public void updatePositionOffset(FrameVector3DReadOnly offset)
   {
      updatePositionsFromControlFrame = true;
      this.offset.set(offset);
   }

   public void hideViz()
   {
//      yoNominalPosition.setToNaN();
//      yoDesiredPosition.setToNaN();
   }

   public void update(PostureOptimizerState optimizerState, PostureOptimizerState previousState)
   {
      controlFrame.update();

      if (updatePositionsFromControlFrame)
      {
         nominalPosition.setFromReferenceFrame(controlFrame);
         desiredPosition.setFromReferenceFrame(controlFrame);

         nominalPosition.sub(offset);
      }

      boolean isNewState = previousState != optimizerState;
      if (isNewState && optimizerState == PostureOptimizerState.OPTIMIZER)
      {
         optimizedOrientation.set(desiredOrientation);
      }
      else if (isNewState && optimizerState == PostureOptimizerState.NOMINAL)
      {
         blendOrientation.set(desiredOrientation);
         blendStartTime = timeProvider.getValue();
      }

      if (optimizerState == PostureOptimizerState.NOMINAL)
      {
         double blendAlpha = EuclidCoreTools.clamp((timeProvider.getValue() - blendStartTime) / BLEND_DURATION, 0.0, 1.0);
         desiredOrientation.interpolate(blendOrientation, nominalOrientation, blendAlpha);
      }
      else if (optimizerState == PostureOptimizerState.OPTIMIZER)
      {
         integrate();

         desiredOrientation.set(optimizedOrientation);

         double filterAlpha = 0.25;
         filteredAngularVelocity.interpolate(angularVelocity, filterAlpha);
      }
      else
      {
         // freeze, no update (just call this to avoid edge cases in rate limiting)
         filteredAngularVelocity.setToZero();
      }

      if (VISUALIZE)
      {
         yoNominalPosition.set(nominalPosition);
         yoDesiredPosition.set(desiredPosition);

         yoNominalOrientation.set(nominalOrientation);
         yoOptimizedOrientation.set(optimizedOrientation);
//         yoActualOrientation.setFromReferenceFrame(controlFrame);
      }

      orientationDifference.difference(nominalOrientation, desiredOrientation);
      retargetDelta.set(orientationDifference.angle());
   }

   public FixedFrameQuaternionBasics getDesiredOrientation()
   {
      return desiredOrientation;
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
