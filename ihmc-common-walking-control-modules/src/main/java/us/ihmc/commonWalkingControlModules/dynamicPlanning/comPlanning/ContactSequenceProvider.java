package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import java.util.List;

public interface ContactSequenceProvider
{
   List<? extends ContactStateProvider> getContactSequence();

   List<? extends ContactStateProvider> getAbsoluteContactSequence();
}
