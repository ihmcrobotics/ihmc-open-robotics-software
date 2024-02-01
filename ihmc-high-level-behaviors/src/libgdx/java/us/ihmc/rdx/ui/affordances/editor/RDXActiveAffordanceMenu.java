package us.ihmc.rdx.ui.affordances.editor;

public enum RDXActiveAffordanceMenu
{
   NONE, PRE_GRASP, GRASP, POST_GRASP;

   public boolean equals(RDXActiveAffordanceMenu other)
   {
      return this.name().equals(other.name()) && this.ordinal() == other.ordinal();
   }
}