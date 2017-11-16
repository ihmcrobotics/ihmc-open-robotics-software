package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;


public class YoParabolicTrajectoryGenerator
{
   private final String nameSuffix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final ReferenceFrame referenceFrame;
   private final YoFrameVector c0, c1, c2;

   private final FrameVector3D tempInitialize;
   private final FramePoint3D tempPackPosition, tempPackPosition2;
   private final FrameVector3D tempPackVelocity;

   public YoParabolicTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + nameSuffix);
      this.referenceFrame = referenceFrame;

      c0 = new YoFrameVector("c0", "", referenceFrame, registry);
      c1 = new YoFrameVector("c1", "", referenceFrame, registry);
      c2 = new YoFrameVector("c2", "", referenceFrame, registry);

      tempInitialize = new FrameVector3D(referenceFrame);
      tempPackPosition = new FramePoint3D(referenceFrame);
      tempPackPosition2 = new FramePoint3D(referenceFrame);
      tempPackVelocity = new FrameVector3D(referenceFrame);

      parentRegistry.addChild(registry);
   }

   public void initialize(FramePoint3D initialPosition, FramePoint3D finalPosition, double heightAtParameter, double parameter)
   {
      double q = parameter;
      MathTools.checkIntervalContains(q, 0.0, 1.0);

      initialPosition.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);

      FramePoint3D intermediatePosition = new FramePoint3D(referenceFrame);
      intermediatePosition.setX(initialPosition.getX() + q * (finalPosition.getX() - initialPosition.getX()));
      intermediatePosition.setY(initialPosition.getY() + q * (finalPosition.getY() - initialPosition.getY()));
      intermediatePosition.setZ(heightAtParameter);
      initialize(initialPosition, intermediatePosition, finalPosition, q);
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
      c2.getFrameTuple(positionToPack);
      positionToPack.scale(MathTools.square(q));

      // c1 * q
      c1.getFrameTuple(tempPackPosition);
      tempPackPosition.scale(q);
      positionToPack.add(tempPackPosition);
      
      // c0
      c0.getFrameTuple(tempPackPosition2);
      positionToPack.add(tempPackPosition2);
   }

   public void getVelocity(FrameVector3D velocityToPack, double parameter)
   {      
      double q = parameter;
      MathTools.checkIntervalContains(q, 0.0, 1.0);
      velocityToPack.setToZero(referenceFrame);

      // 2 * c2 * q
      c2.getFrameTuple(velocityToPack);
      velocityToPack.scale(2.0 * q);
      
      // c1
      c1.getFrameTuple(tempPackVelocity);
      velocityToPack.add(tempPackVelocity);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setToZero(referenceFrame);
      
      // 2 * c2
      c2.getFrameTuple(accelerationToPack);
      accelerationToPack.scale(2.0);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
