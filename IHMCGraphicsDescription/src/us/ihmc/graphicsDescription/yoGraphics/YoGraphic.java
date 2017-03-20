package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public abstract class YoGraphic
{
   private static final boolean USE_JESPERS_BUGGY_HACK_TO_TRY_TO_PUT_YOGRAPHICS_WITH_RESPECT_TO_ROBOT = false;

   private final String name;

   private boolean showGraphicObject = true;
   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final AffineTransform objectTransform;
   private final AffineTransform transform = new AffineTransform();

   protected DoubleProvider globalScaleProvider;

   protected abstract void computeRotationTranslation(AffineTransform transform3D);

   protected abstract boolean containsNaN();

   public abstract Graphics3DObject getLinkGraphics();

   public abstract Artifact createArtifact();

   public YoGraphic(String name)
   {
      this.name = name;
      objectTransform = new AffineTransform();
   }

   public void setGlobalScaleProvider(DoubleProvider globalScaleProvider)
   {
      this.globalScaleProvider = globalScaleProvider;
   }

   public void showGraphicObject()
   {
      showGraphicObject = true;
   }

   public void hideGraphicObject()
   {
      showGraphicObject = false;
   }

   public void setVisible(boolean visible)
   {
      showGraphicObject = visible;
   }

   public boolean isGraphicObjectShowing()
   {
      return showGraphicObject;
   }

   public String getName()
   {
      return name;
   }
   
   public void setRootTransform(RigidBodyTransform transform)
   {
      rootTransform.set(transform);
   }

   public final AffineTransform getTransform()
   {
      if (showGraphicObject && !containsNaN())
      {
         computeRotationTranslation(objectTransform);

         if (USE_JESPERS_BUGGY_HACK_TO_TRY_TO_PUT_YOGRAPHICS_WITH_RESPECT_TO_ROBOT)
         {
            // This is a buggy attempt to make the graphic objects go where the robot is, rather than the estimated robot.
            // It works when simulating, but not when rewinding.
            transform.set(objectTransform);
            transform.preMultiply(rootTransform);
         }
         else
         {
            transform.set(objectTransform);
         }
      }
      else
      {
         transform.setIdentity();
         transform.setScale(0.0);
      }

      return transform;
   }

   /**
    * Overwrite the update method if an object needs special updating. But it is up to each user to update their objects. 
    * The internals only update the position and orientation transform...
    * 
    * This is generally use to update YoGraphics based on non-yovariablized data (referenceframes etc). Not doing this
    * will break rewind-playback.
    */
   public void update()
   {
   }

   @Override
   public String toString()
   {
      return name;
   }
}
