package us.ihmc.robotics.geometry;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

public enum YawPitchRollAxis
{
   YAW(Axis3D.Z), PITCH(Axis3D.Y), ROLL(Axis3D.X);

   private final Axis3D axis3D;
   private final String lowerCasedName;
   private final String pascalCasedName;

   YawPitchRollAxis(Axis3D axis3D)
   {
      this.axis3D = axis3D;
      lowerCasedName = name().toLowerCase();
      pascalCasedName = StringUtils.capitalize(lowerCasedName);
   }

   public YawPitchRoll createYawPitchRoll(double angle)
   {
      YawPitchRoll yawPitchRoll = new YawPitchRoll();
      switch (this)
      {
         case YAW -> yawPitchRoll.setYaw(angle);
         case PITCH -> yawPitchRoll.setPitch(angle);
         case ROLL -> yawPitchRoll.setRoll(angle);
      }
      return yawPitchRoll;
   }

   public double getFromYawPitchRoll(YawPitchRollReadOnly yawPitchRollReadOnly)
   {
      return switch (this)
      {
         case YAW -> yawPitchRollReadOnly.getYaw();
         case PITCH -> yawPitchRollReadOnly.getPitch();
         case ROLL -> yawPitchRollReadOnly.getRoll();
      };
   }

   public Axis3D getAxis3D()
   {
      return axis3D;
   }

   public String getLowerCasedName()
   {
      return lowerCasedName;
   }

   public String getPascalCasedName()
   {
      return pascalCasedName;
   }
}
