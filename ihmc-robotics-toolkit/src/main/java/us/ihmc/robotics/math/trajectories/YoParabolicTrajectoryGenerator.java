package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;


public class YoParabolicTrajectoryGenerator
{
   private final String nameSuffix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final ReferenceFrame referenceFrame;
   private final YoFrameVector c0, c1, c2;

   private final FrameVector3D tempInitialize;
   private final FramePoint3D tempPackPosition;

   public YoParabolicTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + nameSuffix);
      this.referenceFrame = referenceFrame;

      c0 = new YoFrameVector("c0", "", referenceFrame, registry);
      c1 = new YoFrameVector("c1", "", referenceFrame, registry);
      c2 = new YoFrameVector("c2", "", referenceFrame, registry);

      tempInitialize = new FrameVector3D(referenceFrame);
      tempPackPosition = new FramePoint3D(referenceFrame);

      parentRegistry.addChild(registry);
   }

   public void initialize(FramePoint3DReadOnly initialPosition, FramePoint3DReadOnly finalPosition, double heightAtParameter, double parameter)
   {
      double q = parameter;
      MathTools.checkIntervalContains(q, 0.0, 1.0);

      FramePoint3D tempInitialPosition = new FramePoint3D(initialPosition);
      FramePoint3D tempFinalPosition = new FramePoint3D(finalPosition);
      tempInitialPosition.changeFrame(referenceFrame);
      tempFinalPosition.changeFrame(referenceFrame);

      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame);
      intermediatePosition.setX(tempInitialPosition.getX() + q * (tempFinalPosition.getX() - tempInitialPosition.getX()));
      intermediatePosition.setY(tempInitialPosition.getY() + q * (tempFinalPosition.getY() - tempInitialPosition.getY()));
      intermediatePosition.setZ(heightAtParameter);
      initialize(tempInitialPosition, intermediatePosition, tempFinalPosition, q);
   }
   
   public void initialize(FramePoint3D initialPosition, FramePoint3D intermediatePosition, FramePoint3D finalPosition, double intermediateParameter)
   {
      initialPosition.changeFrame(referenceFrame);
      intermediatePosition.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);

      final double q = intermediateParameter;
      MathTools.checkIntervalContains(q, 0.0, 1.0);

      c0.set(initialPosition);

      c2.set(intermediatePosition);
      c2.sub(initialPosition);
      tempInitialize.set(finalPosition);
      tempInitialize.sub(initialPosition);
      tempInitialize.scale(q);
      c2.sub(tempInitialize);
      c2.scale(1.0 / (MathTools.square(q) - q));

      c1.set(finalPosition);
      c1.sub(initialPosition);
      c1.sub(c2);
   }

   public void initialize(FramePoint3D initialPosition, FrameVector3D initialVelocity, FramePoint3D finalPosition)
   {
      initialPosition.changeFrame(referenceFrame);
      initialVelocity.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);
      
      c0.set(initialPosition);
      c1.set(initialVelocity);
      c2.set(finalPosition);
      c2.sub(initialPosition);
      c2.sub(initialVelocity);
   }
   
   public void getPosition(FramePoint3D positionToPack, double parameter)
   {
      double q = parameter;
      MathTools.checkIntervalContains(q, 0.0, 1.0);
      positionToPack.setToZero(referenceFrame);

      // c2 * q^2
      positionToPack.set(c2);
      positionToPack.scale(MathTools.square(q));

      // c1 * q
      tempPackPosition.set(c1);
      tempPackPosition.scale(q);
      positionToPack.add(tempPackPosition);
      
      // c0
      positionToPack.add(c0);
   }

   public void getVelocity(FrameVector3D velocityToPack, double parameter)
   {      
      double q = parameter;
      MathTools.checkIntervalContains(q, 0.0, 1.0);
      velocityToPack.setToZero(referenceFrame);

      // 2 * c2 * q
      velocityToPack.set(c2);
      velocityToPack.scale(2.0 * q);
      
      // c1
      velocityToPack.add(c1);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setToZero(referenceFrame);
      
      // 2 * c2
      accelerationToPack.set(c2);
      accelerationToPack.scale(2.0);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
