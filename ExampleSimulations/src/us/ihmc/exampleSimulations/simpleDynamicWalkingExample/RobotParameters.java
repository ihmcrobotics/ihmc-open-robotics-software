package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import java.util.EnumMap;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;

public class RobotParameters
{
   public enum JointNames
   {
      BODY, HIP, KNEE;

      public String getName()
      {
         switch (this)
         {
         case HIP:
            return "hipJoint";
         case KNEE:
            return "kneeJoint";
         case BODY:
            return "bodyJoint";
         default:
            throw new RuntimeException("Should not get there");
         }
      }

      public static JointNames[] values = new JointNames[] { HIP, KNEE, BODY };
   };

   public enum LinkNames
   {
      BODY_LINK, UPPER_LINK, LOWER_LINK;

      public String getName()
      {
         switch (this)
         {
         case BODY_LINK:
            return "bodyLink";
         case UPPER_LINK:
            return "upperLink";
         case LOWER_LINK:
            return "lowerLink";
         default:
            throw new RuntimeException("Should not get there");
         }
      }

      public static LinkNames[] values = new LinkNames[] { BODY_LINK, UPPER_LINK, LOWER_LINK };
   };

   public final static EnumMap<LinkNames, Double> MASSES = new EnumMap<LinkNames, Double>(LinkNames.class); //MASSES is the name of the enumMap object 
   static
   {
      MASSES.put(LinkNames.BODY_LINK, 20.0); //you can do: MASSES.get(BodyNames.FEMUR);
      MASSES.put(LinkNames.UPPER_LINK, 7.0);
      MASSES.put(LinkNames.LOWER_LINK, 4.0);
   }

   public final static EnumMap<LinkNames, Double> LENGTHS = new EnumMap<LinkNames, Double>(LinkNames.class);
   static
   {
      LENGTHS.put(LinkNames.UPPER_LINK, 0.9);
      LENGTHS.put(LinkNames.LOWER_LINK, 0.8);
      LENGTHS.put(LinkNames.BODY_LINK, 0.0); //TODO try removing
   }

   public final static EnumMap<LinkNames, Double> RADII = new EnumMap<LinkNames, Double>(LinkNames.class);
   static
   {
      RADII.put(LinkNames.UPPER_LINK, 0.15);
      RADII.put(LinkNames.LOWER_LINK, 0.1);
      RADII.put(LinkNames.BODY_LINK, 0.0); //TODO try removing
   }

   public final static EnumMap<Axis, Double> BODY_DIMENSIONS = new EnumMap<Axis, Double>(Axis.class);
   static
   {
      BODY_DIMENSIONS.put(Axis.X, 0.8); 
      BODY_DIMENSIONS.put(Axis.Y, 0.8);
      BODY_DIMENSIONS.put(Axis.Z, 0.8);
   }

   public final static EnumMap<LinkNames, Vector3d> COMs = new EnumMap<LinkNames, Vector3d>(LinkNames.class);
   static
   {
      for (LinkNames linkName : LENGTHS.keySet())
      {
         if (LinkNames.BODY_LINK.getName() == "bodyLink")
         {
            //Check stuff
//            System.out.println("i am in coms");
//            System.out.println(new Vector3d(0.0, 0.0, BODY_DIMENSIONS.get(Axis.Z)/2.0));
//            System.out.println("name" + linkName);
            COMs.put(linkName, new Vector3d(0.0, 0.0, BODY_DIMENSIONS.get(Axis.Z)/2.0));
         }
         else
         {
            COMs.put(linkName, new Vector3d(0.0, 0.0, -LENGTHS.get(linkName) / 2.0)); 
         }

      }
   }

   public final static EnumMap<LinkNames, AppearanceDefinition> APPEARANCE = new EnumMap<LinkNames, AppearanceDefinition>(LinkNames.class);
   static
   {
      APPEARANCE.put(LinkNames.BODY_LINK, YoAppearance.Glass());
      APPEARANCE.put(LinkNames.UPPER_LINK, YoAppearance.Glass());
      APPEARANCE.put(LinkNames.LOWER_LINK, YoAppearance.Glass());
   }
}
