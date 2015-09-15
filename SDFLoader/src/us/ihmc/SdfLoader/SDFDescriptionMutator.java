package us.ihmc.SdfLoader;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface SDFDescriptionMutator
{
   void mutateJointForModel(String modelName, SDFJointHolder jointHolder);

   void mutateLinkForModel(String modelName, SDFLinkHolder linkHolder);
}
