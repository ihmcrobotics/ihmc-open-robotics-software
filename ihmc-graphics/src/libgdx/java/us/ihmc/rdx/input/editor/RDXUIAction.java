package us.ihmc.rdx.input.editor;

@FunctionalInterface
public interface RDXUIAction
{
   void doAction(RDXUITrigger trigger);
}
