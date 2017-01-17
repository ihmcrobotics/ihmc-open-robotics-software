package us.ihmc.simulationconstructionset.gui.camera;

import javax.vecmath.Point3d;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraTrackingAndDollyPositionHolder;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class CameraTrackAndDollyYoVariablesHolder implements CameraTrackingAndDollyPositionHolder
{
   private DoubleYoVariable track_x_var, track_y_var, track_z_var, dolly_x_var, dolly_y_var, dolly_z_var;
   private DoubleYoVariable field_of_view_var;
   
   public CameraTrackAndDollyYoVariablesHolder(YoVariableHolder holder)
   { 
      if (holder != null)
      {
         if (holder.hasUniqueVariable("q_x"))
         {
            setTrackXVar((DoubleYoVariable) holder.getVariable("q_x"));
            setDollyXVar((DoubleYoVariable) holder.getVariable("q_x"));
         }

         if (holder.hasUniqueVariable("q_y"))
         {
            setTrackYVar((DoubleYoVariable) holder.getVariable("q_y"));
            setDollyYVar((DoubleYoVariable) holder.getVariable("q_y"));
         }

         if (holder.hasUniqueVariable("q_z"))
         {
            setTrackZVar((DoubleYoVariable) holder.getVariable("q_z"));
            setDollyZVar((DoubleYoVariable) holder.getVariable("q_z"));
         }
      }

   }

   public void getTrackingPosition(Point3d trackPositionToPack)
   {
      if (track_x_var != null)
      {
         trackPositionToPack.setX(track_x_var.getDoubleValue());
      }
      if (track_y_var != null)
      {
         trackPositionToPack.setY(track_y_var.getDoubleValue());
      }
      if (track_z_var != null)
      {
         trackPositionToPack.setZ(track_z_var.getDoubleValue());
      }
   }

   public void getDollyPosition(Point3d dollyPositionToPack)
   {
      if (dolly_x_var != null)
      {
         dollyPositionToPack.setX(dolly_x_var.getDoubleValue());
      }
      if (dolly_y_var != null)
      {
         dollyPositionToPack.setY(dolly_y_var.getDoubleValue());
      }
      if (dolly_z_var != null)
      {
         dollyPositionToPack.setZ(dolly_z_var.getDoubleValue());
      }
      
   }
   
   
   public void setTrackingVars(DoubleYoVariable xVar, DoubleYoVariable yVar, DoubleYoVariable zVar)
   {
      if (xVar != null)
         track_x_var = xVar;
      if (yVar != null)
         track_y_var = yVar;
      if (zVar != null)
         track_z_var = zVar;
   }

   public void setDollyVars(DoubleYoVariable xVar, DoubleYoVariable yVar, DoubleYoVariable zVar)
   {
      if (xVar != null)
         dolly_x_var = xVar;
      if (yVar != null)
         dolly_y_var = yVar;
      if (zVar != null)
         dolly_z_var = zVar;
   }
   
   public void setTrackXVar(DoubleYoVariable track_x_var)
   {
      this.track_x_var = track_x_var;
   }

   public void setTrackYVar(DoubleYoVariable track_y_var)
   {
      this.track_y_var = track_y_var;
   }

   public void setTrackZVar(DoubleYoVariable track_z_var)
   {
      this.track_z_var = track_z_var;
   }

   public void setDollyXVar(DoubleYoVariable dolly_x_var)
   {
      this.dolly_x_var = dolly_x_var;
   }

   public void setDollyYVar(DoubleYoVariable dolly_y_var)
   {
      this.dolly_y_var = dolly_y_var;
   }

   public void setDollyZVar(DoubleYoVariable dolly_z_var)
   {
      this.dolly_z_var = dolly_z_var;
   }

   public void setFieldOfViewVar(DoubleYoVariable field_of_view_var)
   {
      this.field_of_view_var = field_of_view_var;
   }

   public double getFieldOfView()
   {
      if (field_of_view_var == null)
      {
         return Double.NaN;
      }
      
      else return field_of_view_var.getDoubleValue();
   }

   
   
   public double getTrackingX()
   {
      if (track_x_var != null)
         return track_x_var.getDoubleValue();
      else
         return 0.0;
   }

   public double getTrackingY()
   {
      if (track_y_var != null)
         return track_y_var.getDoubleValue();
      else
         return 0.0;
   }

   public double getTrackingZ()
   {
      if (track_z_var != null)
         return track_z_var.getDoubleValue();
      else
         return 0.0;
   }

   public double getDollyX()
   {
      if (dolly_x_var != null)
         return dolly_x_var.getDoubleValue();
      else
         return 0.0;
   }

   public double getDollyY()
   {
      if (dolly_y_var != null)
         return dolly_y_var.getDoubleValue();
      else
         return 0.0;
   }

   public double getDollyZ()
   {
      if (dolly_z_var != null)
         return dolly_z_var.getDoubleValue();
      else
         return 0.0;
   }

   public void closeAndDispose()
   {
      track_x_var = track_y_var = track_z_var = dolly_x_var = dolly_y_var = dolly_z_var = null;
      field_of_view_var = null;
   }

}
